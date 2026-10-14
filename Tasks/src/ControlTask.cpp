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

static Pid pid_vel(1000.0f, 0.0f, 0.0f, 25000.0f, -25000.0f);
static Pid pid_pos(0.0f, 0.0f, 0.0f, 25000.0f, -25000.0f);

//正弦曲线参数
static constexpr float kPi        = 3.14159265358979323846f;
static constexpr float kVelAmp    = 10.0f;
static constexpr float kVelFreq   = 0.5f;
static constexpr float kPosAmp    = 1.57f;
static constexpr float kPosFreq   = 0.2f;

 volatile float v_ref_trace = 0.0f;
 volatile float v_fdb_trace = 0.0f;
 volatile float u_vel_trace = 0.0f;

static float time_ms = 0.0f;

static uint8_t rx_byte = 0;
/*static char cmd_buf[64];
static uint8_t cmd_idx = 0;*/

/*static bool cmd_overflow = false;
static volatile bool restart_pending = false;*/

/*static constexpr uint8_t KP_UPDATE = 0x01;  //0000 0001
static constexpr uint8_t KI_UPDATE = 0x02;  //0000 0010
static constexpr uint8_t KD_UPDATE = 0x04;  //0000 0100*/

/*struct PidParamUpdate {
    volatile float kp;
    volatile float ki;
    volatile float kd;
    volatile uint8_t mask;
};

static PidParamUpdate vel_update = {};
static PidParamUpdate pos_update = {};*/

// 解析单参数命令，例如 V,kp,10.0
/*static void parse_pid_command(const char *cmd)
{
    
    if (strcmp(cmd, "R") == 0 || strcmp(cmd, "r") == 0) {
        restart_pending = true;
        return;
    }
    
    char type = '\0';
    char param[3] = {};    //参数名称
    float value = 0.0f;    //新参数数值
    int end = 0;

    if (sscanf(cmd, "%c,%2[^,],%f%n",
               &type, param, &value, &end) != 3) {
        return;
    }

    // 拒绝数值后面的多余字符，以及 NaN、Inf
    if (cmd[end] != '\0' || !std::isfinite(value)) {
        return;
    }

    PidParamUpdate *update = nullptr;

    if (type == 'V' || type == 'v') {
        update = &vel_update;
    } else if (type == 'P' || type == 'p') {
        update = &pos_update;
    } else {
        return;
    }

    if (strcmp(param, "kp") == 0) {
        update->kp = value;
        update->mask |= KP_UPDATE;
    } else if (strcmp(param, "ki") == 0) {
        update->ki = value;
        update->mask |= KI_UPDATE;
    } else if (strcmp(param, "kd") == 0) {
        update->kd = value;
        update->mask |= KD_UPDATE;
    }
}*/

// 在控制周期开始时应用待更新参数
/*static void apply_pid_update(Pid &pid, PidParamUpdate &update)
{
    uint32_t primask = __get_PRIMASK();  //保留原状态
    __disable_irq();                     //进入屏蔽可屏蔽中断的状态（当前pid调参优先）

    uint8_t mask = update.mask;          //更新标记，后续用于调参

    if (mask != 0U) {
        float kp = (mask & KP_UPDATE) ? update.kp : pid.kp();
        float ki = (mask & KI_UPDATE) ? update.ki : pid.ki();
        float kd = (mask & KD_UPDATE) ? update.kd : pid.kd();

        pid.setParams(kp, ki, kd);
        update.mask = 0;
    }

    __set_PRIMASK(primask);//恢复之前的状态
}*/

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
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}


void MainTask(void){
    time_ms += 0.001f;

    /*apply_pid_update(pid_vel, vel_update);
    apply_pid_update(pid_pos, pos_update);*/

    /*uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool do_restart = restart_pending;
    restart_pending = false;
    __set_PRIMASK(primask);

    if (do_restart) {
        pid_vel.reset();
        pid_pos.reset();
    }*/

    //速度部分考核
         volatile float v_ref = kVelAmp * sinf(2.0f * kPi * kVelFreq * time_ms);
         volatile float v_fdb = motor_vel.vel();
         volatile float u_vel = pid_vel.calc(v_ref, v_fdb);
        motor_vel.setVoltage(static_cast<int16_t>(u_vel));
        v_ref_trace = v_ref;
        v_fdb_trace = v_fdb;
        u_vel_trace = u_vel;

    //位置部分考核
        static volatile float theta_ref = kPosAmp * sinf(2.0f * kPi * kPosFreq * time_ms);
        static volatile float theta_fdb = motor_pos.angle();
        float u_pos = pid_pos.calc(theta_ref, theta_fdb);
        motor_pos.setVoltage(static_cast<int16_t>(u_pos));

    uint8_t tx_data[8] = {0};
    motor_vel.encode(tx_data);   // 填充 [0,1]
    motor_pos.encode(tx_data);   // 填充 [2,3]

    CAN_TxHeaderTypeDef tx_header = {0};
    tx_header.StdId = 0x1FF;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 8;
    uint32_t tx_mailbox;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);

    /*static uint8_t vofa_divider = 0;
    if (++vofa_divider >= 10) {
        vofa_divider = 0;
        VofaSendJustFloat6(v_ref, v_fdb, u_vel,
                           theta_ref, theta_fdb, u_pos);
    }*/

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

/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }

    char ch = static_cast<char>(rx_byte);

    if (ch == '\n' || ch == '\r') {
        if (!cmd_overflow && cmd_idx > 0U) {
            cmd_buf[cmd_idx] = '\0';
            parse_pid_command(cmd_buf);
        }

        cmd_idx = 0;
        cmd_overflow = false;
    } else if (!cmd_overflow) {
        if (cmd_idx < sizeof(cmd_buf) - 1U) {
            cmd_buf[cmd_idx++] = ch;
        } else {
            // 命令过长时丢弃整行，等待下一个结束符
            cmd_overflow = true;
        }
    }

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}*/
/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        char ch = (char)rx_byte;
        
        //判断是否收到结束符 (换行 \n 或回车 \r)
        if (ch == '\n' || ch == '\r') {
            cmd_buf[cmd_idx] = '\0'; // 在末尾加字符串结束符，方便后续解析
            
            if (cmd_idx > 0) {
                char type = cmd_buf[0]; // 取第一个字符，判断是调速度环(V)还是位置环(P)
                float kp, ki, kd;
                
                //解析格式化字符串："V,10.0,0.5,0.01"
                if (sscanf(cmd_buf, "%c,%f,%f,%f", &type, &kp, &ki, &kd) == 4) {
                    if (type == 'V' || type == 'v') {
                        SetVelPidParams(kp, ki, kd);
                    } else if (type == 'P' || type == 'p') {
                        SetPosPidParams(kp, ki, kd);
                    }   
                }
            }
            cmd_idx = 0; //指令处理完，清空缓冲区，准备接收下一句
        } else {
            //还没收到结束符，把字节存入缓冲区
            if (cmd_idx < sizeof(cmd_buf) - 1) {
                cmd_buf[cmd_idx++] = ch;
            }
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}*/

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