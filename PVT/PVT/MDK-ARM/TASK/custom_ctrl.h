#ifndef __CUSTOM_CTRL_H
#define __CUSTOM_CTRL_H

#include "main.h"

#pragma pack(push, 1)
typedef struct {
    uint8_t head1;       // 0xAA
    uint8_t head2;       // 0x55
    int16_t angles[6];   // angles[0]: 手掌; angles[1~5]: 5个关节
} Master_Arm_Data_t;
#pragma pack(pop)

typedef struct {
    float    arm_target[5];     // 5轴目标角度 (rad)
    float    gm6020_target;     // GM6020目标位置
    uint32_t last_update_time;  // 接收时间戳
    uint8_t  online;            // 在线状态标志
} Custom_Ctrl_t;

extern Custom_Ctrl_t custom_ctrl;

void Custom_Ctrl_Init(void);
void Custom_Ctrl_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);
void Custom_Ctrl_ErrorCallback(UART_HandleTypeDef *huart);
void Custom_Ctrl_Decode(uint8_t *buf, uint16_t len);

#endif
