#include "s_RingBuffer.h"

/* ================================================== */
// include 有延时函数的头文件; 并定义延时宏
#include "main.h"
#define DelayMS(x) HAL_Delay(x)
/* ================================================== */

// ! ========================= 变 量 声 明 ========================= ! //



// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化环形缓冲区
 * @param rb 指向环形缓冲区结构体的指针
 * @param buffer 指向用于存储数据的缓冲区的指针
 * @param size 缓冲区的大小
 * @return 0: 成功; -1: 参数错误; -2: 内存分配失败
 */
int RingBufInit(RingBuffer_t* const rb, uint8_t* buffer, uint16_t size)
{
    if(!rb || !buffer || size == 0) return -1;

    rb->size = size;
    rb->buffer = buffer;
    if(!rb->buffer){
        rb->size = 0;
        return -2;
    }
    
    rb->writeIndex = 0;
    rb->readIndex = 0;
    rb->count = 0;

    return 0;
}

/**
 * @brief 获取环形缓冲区状态
 * @param rb 指向环形缓冲区结构体的指针
 * @return -1: 参数错误; 0: 空; 1: 有数据; 2: 满
 */
int RingBufGetStatus(const RingBuffer_t* const rb)
{
    if(!rb) return -1;

    if(rb->count == 0) return 0;
    else if(rb->count == rb->size) return 2;
    else return 1;
}

/**
 * @brief 向环形缓冲区写入数据
 * @param rb 指向环形缓冲区结构体的指针
 * @param data 指向要写入的数据的指针
 * @param length 要写入的数据长度
 * @param mode 写入模式
 * @return 0: 成功; -1: 参数错误; -2: 超时
 */
int RingBufWrite(RingBuffer_t* const rb, const uint8_t* data, uint16_t length, RingBuffer_Mode_t mode)
{
    switch(mode){
        case BLOCKWRITE:
        {
            for(uint16_t i = 0; i < length; ++i){
                uint8_t timeout = 100;
                while(RingBufGetStatus(rb) == 2 && timeout > 0){
                    DelayMS(1);
                    timeout--;
                }
                if(timeout == 0) return -2;
                rb->buffer[rb->writeIndex] = data[i];
                rb->writeIndex = (rb->writeIndex + 1) % rb->size;
                rb->count++;
            }
            return 0;
        }
        case OVERWRITE:
        {
            for(uint16_t i = 0; i < length; ++i){
                if(RingBufGetStatus(rb) == 2){
                    rb->readIndex = (rb->readIndex + 1) % rb->size;
                    rb->count--;
                }
                rb->buffer[rb->writeIndex] = data[i];
                rb->writeIndex = (rb->writeIndex + 1) % rb->size;
                rb->count++;
            }
            return 0;
        }
        default:
            return -1;
    }
}

/**
 * @brief 从环形缓冲区读取数据
 * @param rb 指向环形缓冲区结构体的指针
 * @param data 指向用于存储读取数据的缓冲区的指针
 * @param length 要读取的数据长度
 * @param mode 读取模式
 * @return 读取的字节数; -1: 参数错误; -2: 超时
 */
int RingBufRead(RingBuffer_t* const rb, uint8_t* data, uint16_t length, RingBuffer_Mode_t mode)
{
    switch(mode){
        case BLOCKREAD:
        {
            for(uint16_t i = 0; i < length; ++i){
                uint8_t timeout = 100;
                while(RingBufGetStatus(rb) == 0 && timeout > 0){
                    DelayMS(1);
                    timeout--;
                }
                if(timeout == 0){
                    return -2;
                }
                data[i] = rb->buffer[rb->readIndex];
                rb->readIndex = (rb->readIndex + 1) % rb->size;
                rb->count--;
            }
            return 0;
        }
        case NOBLOCKREAD:
        {
            uint16_t bytesRead = 0;
            for(uint16_t i = 0; i < length; ++i){
                if(RingBufGetStatus(rb) == 0) break;
                data[i] = rb->buffer[rb->readIndex];
                rb->readIndex = (rb->readIndex + 1) % rb->size;
                rb->count--;
                bytesRead++;
            }
            return bytesRead;
        }
        default:
            return -1;
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //


