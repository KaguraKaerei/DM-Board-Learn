#include "d_HTDM_Driver.h"
#include "ht_motor.h"
#include "ht_motor_config.h"
#include "ht_motor_control.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"

#include <stdio.h>

/* ========================= 私 有 变 量 ========================= */

// 记录每个电机当前模式
static HT_Mode_e s_ht_mode[8] = { 0 };
static DM_Mode_e s_dm_mode[8] = { 0 };

/* ========================= 私 有 函 数 声 明 ========================= */

static Motor_Type_e Get_MotorType(Motor_ID_e id);
static uint8_t Get_HT_ID(Motor_ID_e id);
static uint8_t Get_DM_ID(Motor_ID_e id);
static void DM_Apply_All_And_Start(uint8_t dm_idx);

/* ========================= 接 口 实 现 ========================= */

/**
 * @brief 高擎与达妙电机驱动初始化函数
 * @note  达妙板子的 CAN 口是可控电源接口, 需要 PC14 置高电平
 */
void Motor_HTDM_Driver_Init(void)
{
    HAL_GPIO_WritePin(CAN1_EN_GPIO_Port, CAN1_EN_Pin, GPIO_PIN_SET);

    if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE) != HAL_OK){
        Error_Handler();
    }
    if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK){
        Error_Handler();
    }
    if(HAL_FDCAN_ActivateNotification(&hfdcan1,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
        0) != HAL_OK){
        Error_Handler();
    }

    HAL_Delay(1000);
    dm_motor_init();
}

void Motor_HT_Set_DQVolt(Motor_ID_e id, float dq_v)
{
    if(Get_MotorType(id) != MOTOR_TYPE_HT) return;
    uint8_t hid = Get_HT_ID(id);
    if(!hid) return;
    s_ht_mode[hid] = HT_MODE_DQ_VOLT;
    motor_set_dq_vlot(PORT1, hid, dq_v);
}

void Motor_HT_Set_DQCurrent(Motor_ID_e id, float dq_i)
{
    if(Get_MotorType(id) != MOTOR_TYPE_HT) return;
    uint8_t hid = Get_HT_ID(id);
    if(!hid) return;
    s_ht_mode[hid] = HT_MODE_DQ_CURRENT;
    motor_set_dq_current(PORT1, hid, dq_i);
}

void Motor_HT_Set_Position(Motor_ID_e id, float pos_rad)
{
    if(Get_MotorType(id) != MOTOR_TYPE_HT) return;
    uint8_t hid = Get_HT_ID(id);
    if(!hid) return;
    s_ht_mode[hid] = HT_MODE_POS;
    motor_set_pos(PORT1, hid, pos_rad);
}

void Motor_HT_Set_Velocity(Motor_ID_e id, float vel_rad_s)
{
    if(Get_MotorType(id) != MOTOR_TYPE_HT) return;
    uint8_t hid = Get_HT_ID(id);
    if(!hid) return;
    s_ht_mode[hid] = HT_MODE_VEL;
    motor_set_vel(PORT1, hid, vel_rad_s);
}

void Motor_HT_Set_Torque(Motor_ID_e id, float torque)
{
    if(Get_MotorType(id) != MOTOR_TYPE_HT) return;
    uint8_t hid = Get_HT_ID(id);
    if(!hid) return;
    s_ht_mode[hid] = HT_MODE_TORQUE;
    motor_set_tqe(PORT1, hid, torque);
}

void Motor_HT_Set_PosVel(Motor_ID_e id, float pos_rad, float vel_rad_s)
{
    if(Get_MotorType(id) != MOTOR_TYPE_HT) return;
    uint8_t hid = Get_HT_ID(id);
    if(!hid) return;
    s_ht_mode[hid] = HT_MODE_POS_VEL;
    motor_set_pos_vel(PORT1, hid, pos_rad, vel_rad_s);
}

void Motor_HT_Set_PosVelAcc(Motor_ID_e id, float pos_rad, float vel_rad_s, float acc_rad_s2)
{
    if(Get_MotorType(id) != MOTOR_TYPE_HT) return;
    uint8_t hid = Get_HT_ID(id);
    if(!hid) return;
    s_ht_mode[hid] = HT_MODE_TRAPEZOID;
    motor_set_pos_vel_acc(PORT1, hid, pos_rad, vel_rad_s, acc_rad_s2);
}

void Motor_HT_Set_MIT(Motor_ID_e id, float pos_rad, float vel_rad_s, float torque, float kp, float kd)
{
    if(Get_MotorType(id) != MOTOR_TYPE_HT) return;
    uint8_t hid = Get_HT_ID(id);
    if(!hid) return;
    s_ht_mode[hid] = HT_MODE_MIT;
    motor_set_pos_vel_tqe_kp_kd(PORT1, hid, pos_rad, vel_rad_s, torque, kp, kd);
}

void Motor_HT_Stop(Motor_ID_e id)
{
    if(Get_MotorType(id) != MOTOR_TYPE_HT) return;
    uint8_t hid = Get_HT_ID(id);
    if(!hid) return;
    s_ht_mode[hid] = HT_MODE_STOP;
    motor_set_stop(PORT1, hid);
}

void Motor_HT_Brake(Motor_ID_e id)
{
    if(Get_MotorType(id) != MOTOR_TYPE_HT) return;
    uint8_t hid = Get_HT_ID(id);
    if(!hid) return;
    s_ht_mode[hid] = HT_MODE_BRAKE;
    motor_set_brake(PORT1, hid);
}

void Motor_DM_Set_MIT(Motor_ID_e id, float pos_rad, float vel_rad_s, float torque, float kp, float kd)
{
    if(Get_MotorType(id) != MOTOR_TYPE_DM) return;
    uint8_t idx = Get_DM_ID(id);

    s_dm_mode[idx] = DM_MODE_MIT;
    dm_motor[idx].ctrl.mode = mit_mode;
    dm_motor[idx].ctrl.pos_set = pos_rad;
    dm_motor[idx].ctrl.vel_set = vel_rad_s;
    dm_motor[idx].ctrl.tor_set = torque;
    dm_motor[idx].ctrl.cur_set = 0.0f;
    dm_motor[idx].ctrl.kp_set = kp;
    dm_motor[idx].ctrl.kd_set = kd;

    DM_Apply_All_And_Start(idx);
}

void Motor_DM_Set_PosVel(Motor_ID_e id, float pos_rad, float vel_rad_s)
{
    if(Get_MotorType(id) != MOTOR_TYPE_DM) return;
    uint8_t idx = Get_DM_ID(id);

    s_dm_mode[idx] = DM_MODE_POS;
    dm_motor[idx].ctrl.mode = pos_mode;
    dm_motor[idx].ctrl.pos_set = pos_rad;
    dm_motor[idx].ctrl.vel_set = vel_rad_s;

    DM_Apply_All_And_Start(idx);
}

void Motor_DM_Set_Velocity(Motor_ID_e id, float vel_rad_s)
{
    if(Get_MotorType(id) != MOTOR_TYPE_DM) return;
    uint8_t idx = Get_DM_ID(id);

    s_dm_mode[idx] = DM_MODE_SPD;
    dm_motor[idx].ctrl.mode = spd_mode;
    dm_motor[idx].ctrl.vel_set = vel_rad_s;

    DM_Apply_All_And_Start(idx);
}

void Motor_DM_Set_PosVelCur(Motor_ID_e id, float pos_rad, float vel_rad_s, float cur_amp)
{
    if(Get_MotorType(id) != MOTOR_TYPE_DM) return;
    uint8_t idx = Get_DM_ID(id);

    s_dm_mode[idx] = DM_MODE_PSI;
    dm_motor[idx].ctrl.mode = psi_mode;
    dm_motor[idx].ctrl.pos_set = pos_rad;
    dm_motor[idx].ctrl.vel_set = vel_rad_s;
    dm_motor[idx].ctrl.cur_set = cur_amp;

    DM_Apply_All_And_Start(idx);
}

float Motor_Get_Position(Motor_ID_e id)
{
    Motor_Type_e type = Get_MotorType(id);

    if(type == MOTOR_TYPE_HT){
        uint8_t hid = Get_HT_ID(id);
        if(!hid) return 0.0f;
        motor_get_state_send(PORT1, hid);
        p_motor_state_s st = motor_get_state(PORT1, hid);
        return st ? st->position : 0.0f;
    }
    else{
        uint8_t idx = Get_DM_ID(id);
        // 上面的发送读取实时反馈信息命令, 下面的发送读取电机参数命令
        read_motor_ctrl_fbdata(dm_motor[idx].id);
        read_all_motor_data(&dm_motor[idx]);
        return dm_motor[idx].para.pos;
    }
}

float Motor_Get_Velocity(Motor_ID_e id)
{
    Motor_Type_e type = Get_MotorType(id);

    if(type == MOTOR_TYPE_HT){
        uint8_t hid = Get_HT_ID(id);
        if(!hid) return 0.0f;
        motor_get_state_send(PORT1, hid);
        p_motor_state_s st = motor_get_state(PORT1, hid);
        return st ? st->velocity : 0.0f;
    }
    else{
        uint8_t idx = Get_DM_ID(id);
        read_motor_ctrl_fbdata(dm_motor[idx].id);
        read_all_motor_data(&dm_motor[idx]);
        return dm_motor[idx].para.vel;
    }
}

float Motor_Get_Torque(Motor_ID_e id)
{
    Motor_Type_e type = Get_MotorType(id);

    if(type == MOTOR_TYPE_HT){
        uint8_t hid = Get_HT_ID(id);
        if(!hid) return 0.0f;
        motor_get_state_send(PORT1, hid);
        p_motor_state_s st = motor_get_state(PORT1, hid);
        return st ? st->torque : 0.0f;
    }
    else{
        uint8_t idx = Get_DM_ID(id);
        read_motor_ctrl_fbdata(dm_motor[idx].id);
        read_all_motor_data(&dm_motor[idx]);
        return dm_motor[idx].para.tor;
    }
}

/* ========================= 私 有 函 数 实 现 ========================= */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan->Instance == FDCAN1 || hfdcan->Instance == FDCAN2 || hfdcan->Instance == FDCAN3){
        FDCAN_RxHeaderTypeDef rxHeader;
        uint8_t rx_data[64];
        while(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rx_data) == HAL_OK){
            uint8_t len = 0;
            if(rxHeader.DataLength <= FDCAN_DLC_BYTES_8)
                len = rxHeader.DataLength;
            else
                len = 8;

            if(len == 0)
                continue;

            uint16_t can_id = rxHeader.Identifier;

            // 高擎电机反馈判断
            uint8_t ht_id = (uint8_t)(can_id >> 8);
            if(ht_id >= 1 && ht_id <= MOTOR_MAX_NUM){
                motor_process_state_frame(&hfdcan1, ht_id, rx_data, len);
                continue;
            }

            // 达妙电机反馈判断(获取实际读取到的电机ID, 防止异步混乱)
            uint8_t motor_idx_from_data = (rx_data[0] & 0x0F);
            uint8_t actual_idx = motor_idx_from_data - 1;
            if(can_id == dm_motor[actual_idx].mst_id){
                dm_motor_fbdata(&dm_motor[actual_idx], rx_data);
                receive_motor_data(&dm_motor[actual_idx], rx_data);
                continue;
            }
        }
    }
}

static void DM_Apply_All_And_Start(uint8_t dm_idx)
{
    mode_e mode = dm_motor[dm_idx].ctrl.mode;

    write_motor_data(dm_motor[dm_idx].id, 10, (uint8_t)mode, 0, 0, 0);
    HAL_Delay(1);
    dm_motor_ctrl_send(&hfdcan1, &dm_motor[dm_idx]);
    HAL_Delay(1);
}

static Motor_Type_e Get_MotorType(Motor_ID_e id)
{
    if(id >= MOTOR_DM_1 && id <= MOTOR_DM_6){
        return MOTOR_TYPE_DM;
    }
    return MOTOR_TYPE_HT;
}

static uint8_t Get_HT_ID(Motor_ID_e id)
{
    if(id < MOTOR_HT_CHASSIS || id > MOTOR_HT_JOINT5) return 0;
    return (uint8_t)id;
}

static uint8_t Get_DM_ID(Motor_ID_e id)
{
    if(id < MOTOR_DM_1 || id > MOTOR_DM_6) return 0;
    for(uint8_t i = 0; i < num; ++i){
        if(id - MOTOR_DM_1 == i) return i;
    }
}
