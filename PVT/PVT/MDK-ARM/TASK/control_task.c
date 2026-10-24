#include "control_task.h"
#include "dr16.h"
#include "bsp_can_DM.h"
#include "FreeRTOS.h"
#include "task.h"
#include "custom_ctrl.h"

#define PI              3.14159265f
#define TWO_PI          6.28318530f
#define GM6020_MID_POS  6215.0f

#define SW_DOWN         1
#define SW_MID          2
#define SW_UP           3

extern GM6020_t GM6020;
extern void GM6020_SendVoltage(int16_t voltage);

static uint8_t motor_enabled = 0;
static uint8_t master_locked = 0;
static float master_ready_pose[5] = {0};

const float FRONT_READY[5] = { 0.173f, -0.417f, -0.896f, 1.594f, 2.613f };

// 关节映射与极性配置
static const uint8_t JOINT_MAP[5] = { 4, 3, 2, 1, 0 };
static const float AXIS_DIR[5]    = { -13.0f, -1.20f, 1.0f, -1.0f, 1.0f };

// 软件软限位范围 (rad)，在硬限位前留出安全余量，防止机械撞死堵转
static const float ARM_MIN_LIMIT[5] = { -1.90f, -0.70f, -3.40f, -2.80f, -0.20f }; 
static const float ARM_MAX_LIMIT[5] = {  2.57f,  1.40f,  -0.00f,  3.00f,  2.60f }; 

// 调试探针
volatile int8_t  dbg_test_axis     = -1; // -1: 全轴联动, 0~4: 单轴调试, 5: GM6020
volatile uint8_t dbg_current_state = 0;  // 0: 急停, 1: 准备位, 2: 随动
volatile float   dbg_master_target[5] = {0};
volatile float   dbg_master_diff[5]   = {0};
volatile float   dbg_motor_target[5]  = {0};
volatile uint32_t dbg_mcu_heartbeat   = 0;

static inline float Wrap_Angle(float angle)
{
    while (angle >  PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

static inline float Clamp(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

static float Get_GM6020_Linear_Ecd(uint16_t ecd)
{
    return (ecd < 3800) ? ((float)ecd + 8192.0f) : (float)ecd;
}

static int16_t GM6020_PID_Calc(float target, float current, int16_t now_rpm)
{
    float error = target - current;
    float output = 32.0f * error - 2.0f * (float)now_rpm;

    if (output >  25000.0f) output =  25000.0f;
    if (output < -25000.0f) output = -25000.0f;
    return (int16_t)output;
}

void Control_Task(void *argument)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    for (int i = 0; i < 5; i++) {
        CAN_1.target_vel[i] = 8.0f;
        CAN_1.target_cur[i] = 5.5f; // 限制为5.5A，降低堵转过流风险
        CAN_1.target_pos[i] = FRONT_READY[i];
    }

    while (1)
    {
        dbg_mcu_heartbeat++;

        // 自定义控制器心跳检测
        if (HAL_GetTick() - custom_ctrl.last_update_time > 300) {
            custom_ctrl.online = 0;
        }

        uint8_t rc_online = (rc_ctrl.rc.s[0] != 0 && rc_ctrl.rc.s[1] != 0);

        // 模式 1: 左下右上 -> 自定义控制器随动
        if (rc_online && (rc_ctrl.rc.s[1] == SW_MID && rc_ctrl.rc.s[0] == SW_DOWN))
        {
            dbg_current_state = 2;

            if (custom_ctrl.online)
            {
                if (!motor_enabled) {
                    Motor_enable();
                    motor_enabled = 1;
                }

                // 切入时锁存基准零点
                if (!master_locked) {
                    for (int j = 0; j < 5; j++) {
                        master_ready_pose[j] = custom_ctrl.arm_target[j];
                    }
                    master_locked = 1;
                }

                for (int i = 0; i < 5; i++)
                {
                    uint8_t map_idx = JOINT_MAP[i];
                    float delta = Wrap_Angle(custom_ctrl.arm_target[map_idx] - master_ready_pose[map_idx]);

                    dbg_master_target[map_idx] = custom_ctrl.arm_target[map_idx];
                    dbg_master_diff[map_idx]   = delta;

                    float raw_target = FRONT_READY[i];
                    if (dbg_test_axis == -1 || dbg_test_axis == i) {
                        raw_target = FRONT_READY[i] + (AXIS_DIR[i] * delta);
                    }

                    // 软件限位截断保护，杜绝电机顶死在物理限位上
                    CAN_1.target_pos[i] = Clamp(raw_target, ARM_MIN_LIMIT[i], ARM_MAX_LIMIT[i]);
                    dbg_motor_target[i] = CAN_1.target_pos[i];
                }

                Motor_control();

                if (dbg_test_axis == 5) {
                    float cur = Get_GM6020_Linear_Ecd(GM6020.ecd);
                    GM6020_SendVoltage(GM6020_PID_Calc(custom_ctrl.gm6020_target, cur, GM6020.speed_rpm));
                } else {
                    GM6020_SendVoltage(0);
                }
            }
            else
            {
                GM6020_SendVoltage(0);
                if (motor_enabled) {
                    Motor_disable();
                    motor_enabled = 0;
                    master_locked = 0;
                }
            }
        }
        // 模式 2: 左中右上 -> 平滑回归准备位
        else if (rc_online && (rc_ctrl.rc.s[1] == SW_UP && rc_ctrl.rc.s[0] == SW_DOWN))
        {
            dbg_current_state = 1;
            master_locked = 0;

            if (!motor_enabled) {
                Motor_enable();
                motor_enabled = 1;
            }

            for (int i = 0; i < 5; i++)
            {
                if (CAN_1.target_pos[i] < FRONT_READY[i] - 0.002f) {
                    CAN_1.target_pos[i] += 0.0015f;
                } else if (CAN_1.target_pos[i] > FRONT_READY[i] + 0.002f) {
                    CAN_1.target_pos[i] -= 0.0015f;
                } else {
                    CAN_1.target_pos[i] = FRONT_READY[i];
                }
                dbg_motor_target[i] = CAN_1.target_pos[i];
            }

            Motor_control();
            GM6020_SendVoltage(0);
        }
        // 模式 0: 其他档位 -> 急停下电
        else
        {
            dbg_current_state = 0;
            master_locked = 0;
            GM6020_SendVoltage(0);

            if (motor_enabled) {
                Motor_disable();
                motor_enabled = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
