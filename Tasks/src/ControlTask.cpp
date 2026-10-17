//由于原先不知道电机是电流控制模式，后面写的都是电压控制的变量名，所有有关电压的量对应的都是电流量

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
static Pid pid_pos_ins(200.0f, 450.0f, 0.0f, 16384.0f, -16384.0f);
static Pid pid_pos(0.0f, 0.0f, 0.0f, 16384.0f, -16384.0f);
volatile float kp_out   = 5.0f;
static bool  pos_init   = false;
static float pos_offset = 0.0f;

//正弦曲线参数
static constexpr float kPi        = 3.14159265358979323846f;
static constexpr float kVelAmp    = 10.0f;
static constexpr float kVelFreq   = 0.5f;
static constexpr float kPosAmp    = 10.0f/kPi;
static constexpr float kPosFreq   = 0.5f;
//ozone跟踪参数
volatile float v_ref_trace = 0.0f;
volatile float v_fdb_trace = 0.0f;
volatile float u_vel_trace = 0.0f;
volatile float p_ref_trace = 0.0f;
volatile float p_fdb_trace = 0.0f;
volatile float u_pos_trace = 0.0f;
volatile float v_inr_trace = 0.0f;
volatile float v_inf_trace = 0.0f;

//时间
static float time_ms = 0.0f;

void ControlTaskInit(void){
    //初始化滤波器，can通信，计时器
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
         float feedfor = 31.0f; //补偿系数
         float bemf_voltage = feedfor * v_ref; //反电动势补偿
         float Fc_pos = 55.0f; //正向转动的摩擦补偿
         float Fc_neg = 70.0f; //反向转动的摩擦补偿
         float fc_amp = (v_ref > 0.0f) ? Fc_pos : Fc_neg; //判断是正向转动还是反向转动
         float fc = fc_amp * v_ref / (fabsf(v_ref) + 0.3f); //平滑函数（符号函数sign的平滑版本，在0附近不会从1到-1跳变）
         volatile float u_vel = pid_vel.calc(v_ref, v_fdb) + bemf_voltage + fc; // 输出的电流
         motor_vel.setVoltage(static_cast<int16_t>(u_vel));
         v_ref_trace = v_ref;
         v_fdb_trace = v_fdb;
         u_vel_trace = u_vel;

    //位置部分考核
        if (!pos_init && motor_pos.inited()) {
        pos_offset = motor_pos.angle() + kPosAmp;  // 让 θ_ref(0) 正好等于上电时的角度
        pos_init = true;
        }
        volatile float theta_ref = pos_offset + kPosAmp * -cosf(2.0f * kPi * kPosFreq * time_ms);
        volatile float theta_fdb = motor_pos.angle();
        volatile float om_ref = kVelAmp * sinf(2.0f * kPi * kVelFreq * time_ms);
        volatile float om_fdb = motor_pos.vel();
        //外环
        float om_cmd = om_ref + kp_out * (theta_ref - theta_fdb);
        if (om_cmd >  10.0f) om_cmd =  10.0f;
        if (om_cmd < -10.0f) om_cmd = -10.0f;
        //内环(与速度环一模一样)
        float feedfor_pos = 31.0f;
        float bemf_voltage_pos = feedfor_pos * om_cmd;
        float Fc_pos_pos = 55.0f;
        float Fc_neg_pos = 70.0f;
        float fc_amp_pos = (om_ref > 0.0f) ? Fc_pos_pos : Fc_neg_pos;
        float fc_pos = fc_amp_pos * om_ref / (fabsf(om_ref) + 0.3f);
        float u_pos = 0.0f;
        if (pos_init) {
        u_pos = pid_pos_ins.calc(om_cmd, om_fdb) + bemf_voltage_pos + fc_pos;
        }
         if (u_pos >  16384.0f) u_pos =  16384.0f;   // 先在 float 里限幅，再转 int16
         if (u_pos < -16384.0f) u_pos = -16384.0f;
         motor_pos.setVoltage(static_cast<int16_t>(u_pos));
         v_inr_trace = om_ref;
         v_inf_trace = om_fdb;
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