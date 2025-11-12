#ifndef _D_WS2812_H_
#define _D_WS2812_H_

#include "main.h" 
#include "spi.h"

#define WS2812_SPI_UNIT hspi6

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);

void tool_test(void);
int tool_test_int(void);

#endif
