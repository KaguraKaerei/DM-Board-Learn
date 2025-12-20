#include "s_BlueTooth.h"

#include <stdio.h>
#include <string.h>

#include "s_RingBuffer.h"

/* ========================= 私 有 变 量 声 明 ========================= */

/* ================================================== */
// 如果启用了DMA接收（如 ReceiveToIdle_DMA），则定义此宏: 
#define BT_USE_DMA_RX
// 未定义时：允许阻塞接收（适合简单调试串口）
// 已定义时：禁止阻塞接收，避免中断DMA流程
/* ================================================== */

#define BT_DMA_RX_BUF_SIZE  256
#define BT_CMD_BUF_SIZE     64

static RingBuffer_t bt_ring_buffer;
static uint8_t bt_buffer[BT_DMA_RX_BUF_SIZE];

#ifdef BT_USE_DMA_RX
    static uint8_t bt_dma_rx_buf[BT_DMA_RX_BUF_SIZE];
#else
    static uint8_t bt_uart_rx_byte;
#endif

static char bt_cmd_buffer[BT_CMD_BUF_SIZE];
static uint8_t bt_cmd_index = 0;
static uint8_t bt_cmd_receiving = 0;

/* ========================= 私 有 函 数 声 明 ========================= */

static void bluetooth_parse_cmd(const char* cmd);

#ifdef BT_USE_DMA_RX
    static void bluetooth_init_dma(void);
#else
    static void bluetooth_init_interrupt(void);
#endif

/* ========================= 接 口 函 数 实 现 ========================= */

/**
 * @brief 蓝牙模块初始化函数
 * @note 该函数在系统初始化时调用一次, 用于初始化蓝牙模块
 */
void BlueToothInit(void)
{
    RingBufInit(&bt_ring_buffer, bt_buffer, sizeof(bt_buffer));
    
#ifdef BT_USE_DMA_RX
    bluetooth_init_dma();
#else
    bluetooth_init_interrupt();
#endif
}

/**
 * @brief 蓝牙模块处理函数
 * @note 该函数在系统轮询过程中周期调用, 用于处理蓝牙数据
 * @return 0: 无错误, -1: 命令缓冲区溢出, -2: 非法数据
 */
int BlueToothProcess(void)
{
    uint8_t data;
    
    while(RingBufRead(&bt_ring_buffer, &data, 1, NOBLOCKREAD) == 1){
        if(data == '$'){
            bt_cmd_receiving = 1;
            bt_cmd_index = 0;
            bt_cmd_buffer[bt_cmd_index++] = data;
        }
        else if(bt_cmd_receiving){
            if(bt_cmd_index < BT_CMD_BUF_SIZE - 1){
                bt_cmd_buffer[bt_cmd_index++] = data;
                
                if(data == '#'){
                    bt_cmd_buffer[bt_cmd_index] = '\0';
                    bt_cmd_receiving = 0;
                    bluetooth_parse_cmd(bt_cmd_buffer);
                    bt_cmd_index = 0;
                }
            }
            else{
                bt_cmd_index = 0;
                bt_cmd_receiving = 0;
                return -1;
            }
        }
        else{
            return -2;
        }
    }
    return 0;
}

/* ========================= 私 有 函 数 实 现 ========================= */

/**
 * @brief 解析并执行接收到的蓝牙命令
 * @param cmd 指向以'$'开头、'#'结尾的命令字符串
 */
static void bluetooth_parse_cmd(const char* cmd)
{
    if(strcmp(cmd, "$LED_ON#") == 0){
        printf("Bluetooth Command: LED ON\r\n");
    }
}

#ifdef BT_USE_DMA_RX

/**
 * @brief 使用DMA初始化蓝牙模块接收
 */
static void bluetooth_init_dma(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, bt_dma_rx_buf, BT_DMA_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/**
 * @brief UART接收事件回调函数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if(huart->Instance == huart1.Instance){
        if(size > 0){
            RingBufWrite(&bt_ring_buffer, bt_dma_rx_buf, size, OVERWRITE);
        }
    }
}

/**
 * @brief UART错误回调函数
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart1.Instance){
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, bt_dma_rx_buf, BT_DMA_RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

#else

/**
 * @brief 使用中断初始化蓝牙模块接收
 */
static void bluetooth_init_interrupt(void)
{
    HAL_UART_Receive_IT(&huart1, &bt_uart_rx_byte, 1);
}

/**
 * @brief UART接收完成中断回调函数
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart1.Instance){
        RingBuffer_Write(&bt_ring_buffer, &bt_uart_rx_byte, 1, OVERWRITE);
        HAL_UART_Receive_IT(&huart1, &bt_uart_rx_byte, 1);
    }
}

/**
 * @brief UART错误回调函数
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart1.Instance){
        HAL_UART_Receive_IT(&huart1, &bt_uart_rx_byte, 1);
    }
}

#endif
