#include "Vofa.h"
#include "usart.h"
#include <cstring>
#include <stdint.h>



namespace {
constexpr uint8_t kChannelCount = 6;
constexpr uint8_t kPacketSize = kChannelCount * sizeof(float) + 4;

// DMA 发送期间不能修改该缓冲区，因此使用静态缓冲区。
uint8_t tx_buf[kPacketSize];
volatile uint8_t tx_busy = 0;
}

extern "C" void VofaSendJustFloat6(float vel_ref,
                                   float vel_fdb,
                                   float vel_out,
                                   float pos_ref,
                                   float pos_fdb,
                                   float pos_out)
{
    if (tx_busy) {
        return;
    }

    const float values[kChannelCount] = {
        vel_ref, vel_fdb, vel_out,
        pos_ref, pos_fdb, pos_out
    };

    for (uint8_t i = 0; i < kChannelCount; ++i) {
        std::memcpy(&tx_buf[i * sizeof(float)], &values[i], sizeof(float));
    }

    // VOFA+ JustFloat 帧尾：00 00 80 7F
    tx_buf[kChannelCount * sizeof(float) + 0] = 0x00;
    tx_buf[kChannelCount * sizeof(float) + 1] = 0x00;
    tx_buf[kChannelCount * sizeof(float) + 2] = 0x80;
    tx_buf[kChannelCount * sizeof(float) + 3] = 0x7F;

    tx_busy = 1;
    if (HAL_UART_Transmit_DMA(&huart1, tx_buf, sizeof(tx_buf)) != HAL_OK) {
        tx_busy = 0;
    }
}

// USART1 DMA 发送完成时释放缓冲区。
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        tx_busy = 0;
    }
}

