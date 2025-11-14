#ifndef _D_HTDRIVER_H_
#define _D_HTDRIVER_H_

#include "main.h"
#include "motor_control.h"

typedef enum{
    HT_ARM_PART_NONE = 0,
    HT_CHASSIS,
    HT_JOINT1,
    HT_JOINT2,
    HT_JOINT3,
    HT_JOINT4,
    HT_JOINT5,
    HT_MAX_PART
} HT_Arm_Part_t;

/* ==================== 高 擎 电 机 控 制 接 口 ==================== */

void HT_Driver_Init(void);

/* ===== 基本模式 ===== */
// 停止(CAN通道, 电机ID)
#define HT_Stop(portx, id) motor_set_stop(portx, id)
// 刹车(CAN通道, 电机ID)
#define HT_Brake(portx, id) motor_set_brake(portx, id)
// 设置电压(CAN通道, 电机ID, 电压V)
#define HT_Set_dqVolt(portx, id, volt) motor_set_dq_vlot(portx, id, volt)
// 设置电流(CAN通道, 电机ID, 电流A)
#define HT_Set_dqCurrent(portx, id, cur) motor_set_dq_current(portx, id, cur)
// 设置位置(CAN通道, 电机ID, 位置单位: 弧度)
#define HT_Set_Position(portx, id, pos) motor_set_pos(portx, id, pos)
// 设置速度(CAN通道, 电机ID, 速度单位: 弧度每秒)
#define HT_Set_Velocity(portx, id, vel) motor_set_vel(portx, id, vel)
// 设置扭矩(CAN通道, 电机ID, 扭矩单位: 牛米)
#define HT_Set_Torque(portx, id, tqe) motor_set_tqe(portx, id, tqe)

/* ===== 复合模式 ===== */
// 设置位置与速度(CAN通道, 电机ID, 位置单位: 弧度, 速度单位: 弧度每秒)
#define HT_Set_Pos_Vel(portx, id, pos, vel) motor_set_pos_vel(portx, id, pos, vel)
// 设置位置, 速度与最大扭矩(CAN通道, 电机ID, 位置单位: 弧度, 速度单位: 弧度每秒, 扭矩单位: 牛米)
#define HT_Set_Pos_Vel_MAXtqe(portx, id, pos, vel, tqe) motor_set_pos_vel_MAXtqe(portx, id, pos, vel, tqe)
// 设置速度与加速度(CAN通道, 电机ID, 速度单位: 弧度每秒, 加速度单位: 弧度每秒平方)
#define HT_Set_Vel_Acc(portx, id, vel, acc) motor_set_vel_acc(portx, id, vel, acc)
// 设置位置, 速度与加速度(CAN通道, 电机ID, 位置单位: 弧度, 速度单位: 弧度每秒, 加速度单位: 弧度每秒平方)
#define HT_TrapeZoid(portx, id, pos, vel, acc) motor_set_pos_vel_acc(portx, id, pos, vel, acc)
// 设置位置, 速度, 扭矩与PD参数(CAN通道, 电机ID, 位置单位: 弧度, 速度单位: 弧度每秒, 扭矩单位: 牛米, 比例增益, 微分增益)
#define HT_MIT(portx, id, pos, vel, tqe, kp, kd) motor_set_pos_vel_tqe_kp_kd(portx, id, pos, vel, tqe, kp, kd)

/* ==================== 高 擎 电 机 读 取 接 口 ==================== */

// 读取电机状态(不更新)(CAN通道, 电机ID)
#define HT_Get_State(portx, id) motor_get_state(portx, id)
// 读取角度(不更新)(CAN通道, 电机ID)
static inline float HT_Get_Position(port_t portx, HT_Arm_Part_t id)
{
    p_motor_state_s state = HT_Get_State(portx, id);
    return state ? state->position : 0.0f;
}
// 状态更新(所有电机)
#define HT_State_Update() motor_process_state_all()

p_motor_state_s HT_Get_State_With_Update(port_t portx, HT_Arm_Part_t id);
float HT_Get_Position_With_Update(port_t portx, HT_Arm_Part_t id);

#endif
