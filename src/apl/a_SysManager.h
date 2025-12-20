#ifndef _A_SYSMANAGER_H_
#define _A_SYSMANAGER_H_

// c标准库头文件
#include <stdio.h>
// hal库头文件
#include "main.h"
// 驱动层头文件
#include "d_ws2812.h"
#include "d_HTDM_Driver.h"
#include "d_HTDM_CAN.h"
// 服务层头文件
#include "s_LOG.h"
#include "s_BlueTooth.h"
#include "s_DefineTools.h"
// 应用层头文件



/* ==================== 初 始 化 与 轮 询 进 程 ==================== */

/**
 * @brief 系统管理模块初始化函数
 * @note 该函数在系统启动时(hal初始化后, 轮询前)调用一次, 用于初始化各个模块
 */
static inline void SysManager_Init(void)
{
    /* ===== 驱动层初始化部分 ===== */
    // Motor_HTDM_Driver_Init();
    /* ===== 服务层初始化部分 ===== */
    BlueToothInit();
    /* ===== 应用层初始化部分 ===== */

    /* ===== 轮询前执行部分 ===== */

    // HAL_GPIO_WritePin(CAN1_EN_GPIO_Port, CAN1_EN_Pin, GPIO_PIN_SET);

    //     if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
    //     FDCAN_ACCEPT_IN_RX_FIFO0,
    //     FDCAN_ACCEPT_IN_RX_FIFO0,
    //     FDCAN_REJECT_REMOTE,
    //     FDCAN_REJECT_REMOTE) != HAL_OK){
    //     Error_Handler();
    // }
    // if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK){
    //     Error_Handler();
    // }
    // if(HAL_FDCAN_ActivateNotification(&hfdcan1,
    //     FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
    //     0) != HAL_OK){
    //     Error_Handler();
    // }

    // HAL_Delay(1000);

    // // 23 30 30 30 50 30 35 30
    // // 30 54 31 30 30 30 21
    // uint8_t data[2][8] = { {0x23, 0x30, 0x30, 0x30, 0x50, 0x30, 0x35, 0x30},
    //                        {0x30, 0x54, 0x31, 0x30, 0x30, 0x30, 0x21, 0x00} };
    // for(int i = 0; i < 2; i++) can_send(&hfdcan1, 0x008, data[i], sizeof(data[i]));
    
    WS2812_Ctrl(0, 0, 255);
}

/**
 * @brief 系统管理模块轮询进程函数
 * @note 该函数在系统启动后的主循环中调用
 */
static inline void SysManager_Process(void)
{
    /* ===== 状态机驱动部分 ===== */

    BlueToothProcess();

    /* ===== 周期运行部分 ===== */

    PERIODIC_TASK(50, {

        });

    PERIODIC_TASK(1000, {
        printf("SysManager_Process: Heartbeat\r\n");
        });

    /* ===== 事件驱动部分 ===== */

}

#endif
