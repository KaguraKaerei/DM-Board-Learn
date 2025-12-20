#ifndef _S_RINGBUFFER_H_
#define _S_RINGBUFFER_H_

#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 环形缓冲区结构体
 * @param size 缓冲区大小
 * @param buffer 指向缓冲区的指针
 * @param writeIndex 写入索引
 * @param readIndex 读取索引
 * @param count 当前数据量
 */
typedef struct{
    uint16_t size;
    uint8_t* buffer;
    volatile uint16_t writeIndex;
    volatile uint16_t readIndex;
    volatile uint16_t count;
} RingBuffer_t;

/**
 * @brief 环形缓冲区操作模式枚举
 * @brief BLOCKWRITE: 阻塞写入模式
 * @brief OVERWRITE: 覆盖写入模式
 * @brief BLOCKREAD: 阻塞读取模式
 * @brief NOBLOCKREAD: 非阻塞读取模式
 */
typedef enum{
    BLOCKWRITE,
    OVERWRITE,
    BLOCKREAD,
    NOBLOCKREAD
} RingBuffer_Mode_t;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

int RingBufInit(RingBuffer_t* const rb, uint8_t* buffer, uint16_t size);
int RingBufGetStatus(const RingBuffer_t* const rb);
int RingBufWrite(RingBuffer_t* const rb, const uint8_t* data, uint16_t length, RingBuffer_Mode_t mode);
int RingBufRead(RingBuffer_t* const rb, uint8_t* data, uint16_t length, RingBuffer_Mode_t mode);

// ! ========================= 模 版 方 法 实 现 ========================= ! //



#endif
