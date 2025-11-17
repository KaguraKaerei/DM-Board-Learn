#ifndef _D_HTDM_DRIVER_H_
#define _D_HTDM_DRIVER_H_

#include "main.h"

/* ==================== 电 机 ID 定 义 ==================== */

/**
 * @brief 统一电机ID枚举
 * @note  高擎电机ID范围: 1-100, 达妙电机ID范围: 101-200
 */
typedef enum{
    MOTOR_NONE = 0,
    // 高擎电机 ID
    MOTOR_HT_CHASSIS = 1,
    MOTOR_HT_JOINT1,
    MOTOR_HT_JOINT2,
    MOTOR_HT_JOINT3,
    MOTOR_HT_JOINT4,
    MOTOR_HT_JOINT5,

    // 达妙电机 ID
    MOTOR_DM_1 = 101,
    MOTOR_DM_2,
    MOTOR_DM_3,
    MOTOR_DM_4,
    MOTOR_DM_5,
    MOTOR_DM_6,
} Motor_ID_e;

/* ==================== 电 机 厂 商 类 型 ==================== */

/**
 * @brief 电机厂商类型枚举
 */
typedef enum{
    MOTOR_TYPE_HT,
    MOTOR_TYPE_DM
} Motor_Type_e;

/* ==================== 模 式 定 义 ==================== */

/**
 * @brief 达妙电机控制模式枚举
 */
typedef enum{
    DM_MODE_MIT = 1,   // mit_mode : 位置+速度+力矩+kp+kd
    DM_MODE_POS = 2,   // pos_mode : 位置+速度
    DM_MODE_SPD = 3,   // spd_mode : 速度
    DM_MODE_PSI = 4    // psi_mode : 位置+速度+电流
} DM_Mode_e;

/**
 * @brief 高擎电机控制模式枚举
 */
typedef enum{
    HT_MODE_DQ_VOLT = 1,        // DQ 电压模式
    HT_MODE_DQ_CURRENT,         // DQ 电流模式
    HT_MODE_POS,                // 位置模式
    HT_MODE_VEL,                // 速度模式
    HT_MODE_TORQUE,             // 力矩模式
    HT_MODE_POS_VEL,            // 位置速度模式
    HT_MODE_TRAPEZOID,          // 梯形控制模式
    HT_MODE_MIT,                // MIT 运控模式
    HT_MODE_STOP,               // 停止模式
    HT_MODE_BRAKE               // 刹车模式
} HT_Mode_e;

/* ==================== 电 机 控 制 统 一 接 口 ==================== */

void Motor_HTDM_Driver_Init(void);

void Motor_HT_Set_DQVolt(Motor_ID_e id, float dq_v);
void Motor_HT_Set_DQCurrent(Motor_ID_e id, float dq_i);
void Motor_HT_Set_Position(Motor_ID_e id, float pos_rad);
void Motor_HT_Set_Velocity(Motor_ID_e id, float vel_rad_s);
void Motor_HT_Set_Torque(Motor_ID_e id, float torque);
void Motor_HT_Set_PosVel(Motor_ID_e id, float pos_rad, float vel_rad_s);
void Motor_HT_Set_PosVelAcc(Motor_ID_e id, float pos_rad, float vel_rad_s, float acc_rad_s2);
void Motor_HT_Set_MIT(Motor_ID_e id, float pos_rad, float vel_rad_s, float torque, float kp, float kd);
void Motor_HT_Stop(Motor_ID_e id);
void Motor_HT_Brake(Motor_ID_e id);

void Motor_DM_Set_MIT(Motor_ID_e id, float pos_rad, float vel_rad_s, float torque, float kp, float kd);
void Motor_DM_Set_PosVel(Motor_ID_e id, float pos_rad, float vel_rad_s);
void Motor_DM_Set_Velocity(Motor_ID_e id, float vel_rad_s);
void Motor_DM_Set_PosVelCur(Motor_ID_e id, float pos_rad, float vel_rad_s, float cur_amp);

float Motor_Get_Position(Motor_ID_e id);
float Motor_Get_Velocity(Motor_ID_e id);
float Motor_Get_Torque(Motor_ID_e id);

#endif
