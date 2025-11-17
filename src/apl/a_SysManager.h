#ifndef _A_SYSMANAGER_H_
#define _A_SYSMANAGER_H_

// c标准库头文件
#include <stdio.h>
// hal库头文件
#include "main.h"
// 驱动层头文件
#include "d_ws2812.h"
#include "d_HTDM_Driver.h"
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
    Motor_HTDM_Driver_Init();
    /* ===== 服务层初始化部分 ===== */
    BlueTooth_Init();
    /* ===== 应用层初始化部分 ===== */

    /* ===== 轮询前执行部分 ===== */
    // Motor_HT_Set_Position(MOTOR_HT_JOINT1, 3.14f);
    Motor_DM_Set_PosVel(MOTOR_DM_1, 6.28f, 6.28f);
}

/**
 * @brief 系统管理模块轮询进程函数
 * @note 该函数在系统启动后的主循环中调用
 */
static inline void SysManager_Process(void)
{
    /* ===== 状态机驱动部分 ===== */

    BlueTooth_Process();

    /* ===== 周期运行部分 ===== */

    PERIODIC_TASK(50, {
        WS2812_Ctrl(255, 0, 0);
    });

    PERIODIC_TASK(10, {
        // float ht_pos = Motor_Get_Position(MOTOR_HT_JOINT1);
        float dm_pos = Motor_Get_Position(MOTOR_DM_1);
        _INFO("DM1: %.3f rad", dm_pos);
    });

    /* ===== 事件驱动部分 ===== */

}

#endif
