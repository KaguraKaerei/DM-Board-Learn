#include "d_ws2812.h"

#define WS2812_LowLevel    0xC0     // 0码
#define WS2812_HighLevel   0xF0     // 1码

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    uint8_t res = 0;
    for(int i = 0; i < 8; i++){
        txbuf[7 - i] = (((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[15 - i] = (((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[23 - i] = (((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    }
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 0, 0xFFFF);
    while(WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);
    for(int i = 0; i < 100; i++){
        HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 1, 0xFFFF);
    }
}

#include "s_DefineTools.h"
void tool_test(void)
{
    printf("test1\r\n");
    NB_DELAY_MS(1000);
    printf("      test2\r\n");
    NB_DELAY_MS(1000);
    printf("            test3\r\n");
}

int tool_test_int(void)
{
    printf("test_int1\r\n");
    NB_DELAY_MS(1000, -1);
    printf("          test_int2\r\n");
    return 0;
}
