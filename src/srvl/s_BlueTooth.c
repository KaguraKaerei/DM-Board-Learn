#include "s_BlueTooth.h"
#include <stdio.h>
#include "s_LOG.h"

/* ========================= 私 有 变 量 声 明 ========================= */

static RingBuffer_t btRingBuffer;
static uint8_t btBuffer[256];
static char cmdBuffer[64];
static uint8_t cmdIndex = 0;
static uint8_t cmdReceiving = 0;
static uint8_t rx_byte = 0;

/* ========================= 私 有 函 数 声 明 ========================= */

static void BlueTooth_Callback(void);
static void BlueTooth_Parse(const char* cmd);

static void RingBuffer_Init(RingBuffer_t* const rb, uint8_t* buffer, uint16_t size);
static uint8_t RingBuffer_GetStatus(const RingBuffer_t* const rb);
static uint8_t RingBuffer_Write(RingBuffer_t* const rb, const uint8_t* data, uint16_t length, RingBuffer_Mode_t mode);
static int RingBuffer_Read(RingBuffer_t* const rb, uint8_t* data, uint16_t length, RingBuffer_Mode_t mode);

/* ========================= 接 口 函 数 实 现 ========================= */

void BlueTooth_Init(void)
{
    RingBuffer_Init(&btRingBuffer, btBuffer, sizeof(btBuffer));
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

void BlueTooth_Process(void)
{
    uint8_t data;
    while(RingBuffer_Read(&btRingBuffer, &data, 1, NOBLOCKREAD) == 1){
        if(data == '$'){
            cmdReceiving = 1;
            cmdIndex = 0;
            cmdBuffer[cmdIndex++] = data;
        }
        else if(cmdReceiving){
            if(cmdIndex < sizeof(cmdBuffer) - 1){
                cmdBuffer[cmdIndex++] = data;
                if(data == '#'){
                    cmdBuffer[cmdIndex] = '\0';
                    cmdReceiving = 0;
                    // 处理命令
                    BlueTooth_Parse(cmdBuffer);
                    cmdIndex = 0;
                }
            }
            else{
                // 命令过长，重置
                cmdIndex = 0;
                cmdReceiving = 0;
                _WARN("BlueTooth_Process: Command too long");
            }
        }
        else{
            // 忽略多余数据
            cmdIndex = 0;
            cmdReceiving = 0;
            _WARN("BlueTooth_Process: Ignored data '%c'", data);
        }
    }
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* ========================= 私 有 函 数 实 现 ========================= */

static void BlueTooth_Parse(const char* cmd)
{
    _INFO("Received BT Command: %s", cmd);
}

static void BlueTooth_Callback(void)
{
    RingBuffer_Write(&btRingBuffer, &rx_byte, 1, OVERWRITE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1){
        BlueTooth_Callback();
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
}

static void RingBuffer_Init(RingBuffer_t* const rb, uint8_t* buffer, uint16_t size)
{
    if(!rb || !buffer || size == 0){
        _WARN("RingBuffer_Init: Invalid parameters");
        return;
    }

    rb->size = size;
    rb->buffer = buffer;
    if(!rb->buffer){
        _WARN("RingBuffer_Init: Memory allocation failed");
        rb->size = 0;
        return;
    }
    rb->writeIndex = 0;
    rb->readIndex = 0;
    rb->count = 0;
}

static uint8_t RingBuffer_GetStatus(const RingBuffer_t* const rb)
{
    if(!rb){
        _WARN("RingBuffer_GetStatus: rb is NULL");
        return 0;
    }
    if(rb->count == 0) return 0;                // 空
    else if(rb->count == rb->size) return 2;    // 满
    else return 1;                              // 有数据
}

static uint8_t RingBuffer_Write(RingBuffer_t* const rb, const uint8_t* data, uint16_t length, RingBuffer_Mode_t mode)
{
    switch(mode){
        case BLOCKWRITE:
        {
            for(uint16_t i = 0; i < length; ++i){
                uint8_t timeout = 100;
                while(RingBuffer_GetStatus(rb) == 2 && timeout > 0){
                    HAL_Delay(1);
                    timeout--;
                }
                if(timeout == 0){
                    _WARN("RingBuffer_Write: timeout");
                    return 1;
                }
                rb->buffer[rb->writeIndex] = data[i];
                rb->writeIndex = (rb->writeIndex + 1) % rb->size;
                rb->count++;
            }
            return 0;
        }
        case OVERWRITE:
        {
            for(uint16_t i = 0; i < length; ++i){
                if(RingBuffer_GetStatus(rb) == 2){
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
            _WARN("RingBuffer_Write: Unsupported mode");
            return 1;
    }
}

static int RingBuffer_Read(RingBuffer_t* const rb, uint8_t* data, uint16_t length, RingBuffer_Mode_t mode)
{
    switch(mode){
        case BLOCKREAD:
        {
            for(uint16_t i = 0; i < length; ++i){
                uint8_t timeout = 100;
                while(RingBuffer_GetStatus(rb) == 0 && timeout > 0){
                    HAL_Delay(1);
                    timeout--;
                }
                if(timeout == 0){
                    _WARN("RingBuffer_Read: timeout");
                    return -1;
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
                if(RingBuffer_GetStatus(rb) == 0) break;
                data[i] = rb->buffer[rb->readIndex];
                rb->readIndex = (rb->readIndex + 1) % rb->size;
                rb->count--;
                bytesRead++;
            }
            return bytesRead;
        }
        default:
            _WARN("RingBuffer_Read: Unsupported mode");
            return -1;
    }
}
