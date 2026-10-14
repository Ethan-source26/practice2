#ifndef VOFA_H
#define VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

// JustFloat 六通道：发送成功时启动 USART1 DMA，忙时跳过本次发送
void VofaSendJustFloat6(float vel_ref,
                        float vel_fdb,
                        float vel_out,
                        float pos_ref,
                        float pos_fdb,
                        float pos_out);

#ifdef __cplusplus
}
#endif

#endif // VOFA_H