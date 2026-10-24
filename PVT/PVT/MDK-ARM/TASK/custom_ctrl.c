#include "custom_ctrl.h"
#include "usart.h"

#define DEG_TO_RAD  0.0174532925f

Custom_Ctrl_t custom_ctrl = {0};
static uint8_t zigbee_rx_buf[32]; 

extern UART_HandleTypeDef huart6;

void Custom_Ctrl_Init(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    __HAL_UART_CLEAR_NEFLAG(&huart6);
    __HAL_UART_CLEAR_FEFLAG(&huart6);
    __HAL_UART_CLEAR_PEFLAG(&huart6);

    volatile uint32_t dummy = huart6.Instance->SR;
    dummy = huart6.Instance->DR;
    (void)dummy;

    huart6.ErrorCode = HAL_UART_ERROR_NONE;
    huart6.RxState   = HAL_UART_STATE_READY;

    HAL_UARTEx_ReceiveToIdle_IT(&huart6, zigbee_rx_buf, sizeof(zigbee_rx_buf));
}

void Custom_Ctrl_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        volatile uint32_t dummy = huart->Instance->SR;
        dummy = huart->Instance->DR;
        (void)dummy;

        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState   = HAL_UART_STATE_READY;

        HAL_UARTEx_ReceiveToIdle_IT(&huart6, zigbee_rx_buf, sizeof(zigbee_rx_buf));
    }
}

void Custom_Ctrl_RxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART6)
    {
        if (Size >= sizeof(Master_Arm_Data_t)) {
            Custom_Ctrl_Decode(zigbee_rx_buf, Size);
        }

        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);

        HAL_UARTEx_ReceiveToIdle_IT(&huart6, zigbee_rx_buf, sizeof(zigbee_rx_buf));
    }
}

void Custom_Ctrl_Decode(uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i <= len - 14; i++)
    {
        if (buf[i] == 0xAA && buf[i + 1] == 0x55)
        {
            // 达妙 5 轴关节解析
            for (int j = 0; j < 5; j++)
            {
                float deg = 0.0f;

                if (j == 4) {
                    // 关节 4 为 12-bit 绝对值编码器 (0~4095)
                    uint16_t raw_enc = (uint16_t)((buf[i + 4 + j * 2] << 8) | buf[i + 5 + j * 2]) & 0x0FFF;
                    deg = ((float)raw_enc / 4096.0f) * 360.0f;
                } else {
                    int16_t raw = (int16_t)((buf[i + 4 + j * 2] << 8) | buf[i + 5 + j * 2]);
                    deg = (float)raw;
                }

                // 转换至 [-180, +180] 范围
                if (deg >  180.0f) deg -= 360.0f;
                if (deg < -180.0f) deg += 360.0f;

                // 转换为弧度
                custom_ctrl.arm_target[j] = deg * DEG_TO_RAD;

                // 限幅保护
                if (custom_ctrl.arm_target[j] >  3.14f) custom_ctrl.arm_target[j] =  3.14f;
                if (custom_ctrl.arm_target[j] < -3.14f) custom_ctrl.arm_target[j] = -3.14f;
            }

            // GM6020 手掌挥拍关节解析
            int16_t palm_raw = (int16_t)((buf[i + 2] << 8) | buf[i + 3]);
            float palm_deg   = (float)palm_raw;
            if (palm_deg > 180.0f) palm_deg -= 360.0f;

            custom_ctrl.gm6020_target = 6215.0f - (palm_deg / 90.0f) * 2000.0f;

            if (custom_ctrl.gm6020_target < 4103.0f) custom_ctrl.gm6020_target = 4103.0f;
            if (custom_ctrl.gm6020_target > 8237.0f) custom_ctrl.gm6020_target = 8237.0f;

            custom_ctrl.last_update_time = HAL_GetTick();
            custom_ctrl.online = 1;
            return;
        }
    }
}
