#include "bsp_usart.h"
#include "dr16.h"

uint8_t sbus_rx_buf[36];

extern UART_HandleTypeDef huart3;
extern void Custom_Ctrl_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);
extern void Custom_Ctrl_ErrorCallback(UART_HandleTypeDef *huart);

extern volatile uint32_t dr16_recv_count;
extern volatile uint32_t dr16_valid_count;

void RC_Init(void)
{
    HAL_UART_DMAStop(&huart3);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_rx_buf, sizeof(sbus_rx_buf));

    if (huart3.hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        dr16_recv_count++;

        if (Size == 18) {
            dr16_valid_count++;
            DR16_Decode(sbus_rx_buf);
        }

        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, sbus_rx_buf, sizeof(sbus_rx_buf));
        if (huart->hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        }
    }
    else if (huart->Instance == USART6)
    {
        Custom_Ctrl_RxCallback(huart, Size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
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

        HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, sbus_rx_buf, sizeof(sbus_rx_buf));
        if (huart->hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        }
    }
    else if (huart->Instance == USART6)
    {
        Custom_Ctrl_ErrorCallback(huart);
    }
}
