#include "d_HTDM_Driver.h"
#include "motor.h"
#include "motor_config.h"
#include "motor_control.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"

/* ========================= 私 有 变 量 声 明 ========================= */



/* ========================= 私 有 函 数 声 明 ========================= */



/* ========================= 接 口 函 数 实 现 ========================= */

/**
 * @brief 高擎与达妙电机驱动初始化函数
 * @note 达妙板子的CAN口是可控电源接口, 需要PC14置高电平
 */
void HTDM_Driver_Init(void)
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
    HAL_Delay(10);

    write_motor_data(dm_motor[Motor1].id, 10, mit_mode, 0, 0, 0);
    HAL_Delay(100);
    save_motor_data(dm_motor[Motor1].id, 10);
    HAL_Delay(100);
    dm_motor_enable(&hfdcan1, &dm_motor[Motor1]);
    HAL_Delay(1000);
}

/**
 * @brief 达妙电机周期任务发送函数
 */
void DM_Runner(void)
{
    dm_motor_ctrl_send(&hfdcan1, &dm_motor[Motor1]);
}

/**
 * @brief 读取电机状态(更新)
 * @param portx CAN端口(PORT1, PORT2, PORT3)
 * @param id 电机ID(HT_CHASSIS, HT_JOINT1~HT_JOINT5)
 * @return 电机状态指针
 */
p_motor_state_s HT_Get_State_With_Update(port_t portx, HT_Arm_Part_t id)
{
    motor_get_state_send(portx, id);
    return motor_get_state(portx, id);
}

/**
 * @brief 读取电机位置(更新)
 * @param portx CAN端口(PORT1, PORT2, PORT3)
 * @param id 电机ID(HT_CHASSIS, HT_JOINT1~HT_JOINT5)
 * @return 电机位置(弧度)
 */
float HT_Get_Position_With_Update(port_t portx, HT_Arm_Part_t id)
{
    motor_get_state_send(portx, id);
    p_motor_state_s state = motor_get_state(portx, id);
    return state ? state->position : 0.0f;
}

/* ========================= 私 有 函 数 实 现 ========================= */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)  
{
    if(hfdcan->Instance == FDCAN1 || hfdcan->Instance == FDCAN2 || hfdcan->Instance == FDCAN3)  
    {
        motor_process_state_all();
        if(hfdcan->Instance == FDCAN1) fdcan1_rx_callback();
    }
}
