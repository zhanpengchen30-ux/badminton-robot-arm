#include "dr16.h"
#include <string.h>

RC_ctrl_t rc_ctrl;

// 计数器（供 Watch 窗口观察）
volatile uint32_t dr16_recv_count = 0;
volatile uint32_t dr16_valid_count = 0;

void DR16_Decode(volatile const uint8_t *buff) {
    if (buff == NULL) return;

    // 1. 解析 4 个摇杆通道 (范围: -660 ~ 660, 中位 0)
    rc_ctrl.rc.ch[0] = ((buff[0] | (buff[1] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[1] = (((buff[1] >> 3) | (buff[2] << 5)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[2] = (((buff[2] >> 6) | (buff[3] << 2) | (buff[4] << 10)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[3] = (((buff[4] >> 1) | (buff[5] << 7)) & 0x07FF) - RC_CH_VALUE_OFFSET;

    // 2. 解析 2 个三位开关 (1:下, 3:上, 2:中)
    rc_ctrl.rc.s[0] = ((buff[5] >> 4) & 0x0003);        // S1 (左侧开关)
    rc_ctrl.rc.s[1] = ((buff[5] >> 6) & 0x0003);        // S2 (右侧开关)

    // 3. 解析左侧上方侧边拨轮通道 (通道 4)
    rc_ctrl.rc.ch[4] = ((buff[16] | (buff[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;
}
