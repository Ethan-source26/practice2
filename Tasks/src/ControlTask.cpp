#include "ControlTask.h"
#include "main.h"
#include "Gm6020.h"
#include "Pid.h"
#include "Vofa.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <cmath>

#include "tim.h"
#include "can.h"
#include "iwdg.h"
#include "usart.h"


#ifdef __cplusplus
extern "C" {
#endif

static Gm6020 motor_vel(1);   //速度
static Gm6020 motor_pos(2);   //位置

static Pid pid_vel(200.0f, 450.0f,0.0f, 16384.0f, -16384.0f);
static Pid pid_pos(0.0f, 0.0f, 0.0f, 16384.0f, -16384.0f);

//正弦曲线参数
static constexpr float kPi        = 3.14159265358979323846f;
static constexpr float kVelAmp    = 10.0f;
static constexpr float kVelFreq   = 0.5f;
static constexpr float kPosAmp    = 1.57f;
static constexpr float kPosFreq   = 0.2f;

 volatile float v_ref_trace = 0.0f;
 volatile float v_fdb_trace = 0.0f;
 volatile float u_vel_trace = 0.0f;
 volatile float p_ref_trace = 0.0f;
 volatile float p_fdb_trace = 0.0f;
 volatile float u_pos_trace = 0.0f;

static float time_ms = 0.0f;

static uint8_t rx_byte = 0;

void ControlTaskInit(void){
    CAN_FilterTypeDef filter = {0};
    filter.FilterIdHigh     = (0x200 << 5) & 0xFFFF;
    filter.FilterIdLow      = 0x0000;
    filter.FilterMaskIdHigh = (0x7F0 << 5) & 0xFFFF;
    filter.FilterMaskIdLow  = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank       = 0;
    filter.FilterMode       = CAN_FILTERMODE_IDMASK;
    filter.FilterScale      = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;   
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_TIM_Base_Start_IT(&htim6);
}


void MainTask(void){
    time_ms += 0.001f;
    
    //速度部分考核
         volatile float v_ref = kVelAmp * sinf(2.0f * kPi * kVelFreq * time_ms);
         volatile float v_fdb = motor_vel.vel();
         float Fc_pos = 55.0f;
         float Fc_neg = 70.0f;
         float fc_amp = (v_ref > 0.0f) ? Fc_pos : Fc_neg;
         float fc = fc_amp * v_ref / (fabsf(v_ref) + 0.3f);
         volatile float u_vel = pid_vel.calc(v_ref, v_fdb) +31 * v_ref + fc;
         motor_vel.setVoltage(static_cast<int16_t>(u_vel));
         v_ref_trace = v_ref;
         v_fdb_trace = v_fdb;
         u_vel_trace = u_vel;

    //位置部分考核
         volatile float theta_ref = kPosAmp * sinf(2.0f * kPi * kPosFreq * time_ms);
         volatile float theta_fdb = motor_pos.angle();
         float u_pos = pid_pos.calc(theta_ref, theta_fdb);
         motor_pos.setVoltage(static_cast<int16_t>(u_pos));
         p_ref_trace = theta_ref;
         p_fdb_trace = theta_fdb;
         u_pos_trace = u_pos;

     uint8_t tx_data[8] = {0};
     motor_vel.encode(tx_data);   // 填充 [0,1]
     motor_pos.encode(tx_data);   // 填充 [2,3]

     CAN_TxHeaderTypeDef tx_header = {0};
     tx_header.StdId = 0x1FE;
     tx_header.IDE   = CAN_ID_STD;
     tx_header.RTR   = CAN_RTR_DATA;
     tx_header.DLC   = 8;
     uint32_t tx_mailbox;
     HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);

     HAL_IWDG_Refresh(&hiwdg);
}

void SetVelPidParams(float kp, float ki, float kd) {
    pid_vel.setParams(kp, ki, kd);
    pid_vel.reset();
}

void SetPosPidParams(float kp, float ki, float kd) {
    pid_pos.setParams(kp, ki, kd);
    pid_pos.reset();
}

void CanFeedbackCallback(uint32_t std_id, const uint8_t *data){
    if (std_id >= 0x205 && std_id <= 0x20B) {
        uint8_t id = static_cast<uint8_t>(std_id - 0x204);
        if (id == 1) {
            motor_vel.decode(data);
        } else if (id == 2) {
            motor_pos.decode(data);
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        MainTask();   
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
            if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK&& rx_header.IDE == CAN_ID_STD) {
                CanFeedbackCallback(rx_header.StdId, rx_data);
            }
        }
    }
}




#ifdef __cplusplus
}
#endif