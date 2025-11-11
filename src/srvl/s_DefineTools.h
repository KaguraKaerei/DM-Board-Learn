#ifndef _S_DEFINETOOLS_H_
#define _S_DEFINETOOLS_H_

#include "main.h"
#include <stdio.h>
#include <stdarg.h>

/* ==================== 宏 定 义 工 具 ==================== */

/**
 * @brief 定义周期任务执行宏
 * @param period_ms 周期时间, 单位为毫秒
 * @param code_block 代码块, 即需要周期执行的代码, 需要用花括号括起来
 */
#define PERIODIC_TASK(period_ms, code_block)                            \
            do{                                                         \
                static uint32_t timer = 0;                              \
                if(timer == 0) timer = HAL_GetTick();                   \
                if((uint32_t)(HAL_GetTick() - (timer)) >= (period_ms)){ \
                    (timer) = HAL_GetTick();                            \
                    code_block                                          \
                }                                                       \
            } while(0)

/**
 * @brief 定义非阻塞延时宏
 * 
 */
#define NOBLOCK_DELAY_BEGIN     static uint8_t _delay_reset_flag = 0
#define NOBLOCK_DELAY_END       _delay_reset_flag = 1
#define NOBLOCK_DELAY_MS(ms, ...) _NOBLOCK_DELAY_MS_IMPL(__LINE__, ms, ##__VA_ARGS__)
#define _NOBLOCK_DELAY_MS_IMPL(line, ms, ...)                               \
            do{                                                             \
                static uint32_t delayer_##line = 0;                         \
                static uint32_t passed_##line = 0;                          \
                if(_delay_reset_flag){                                      \
                    delayer_##line = 0;                                     \
                    passed_##line = 0;                                      \
                    _delay_reset_flag = 0;                                  \
                }                                                           \
                if(!passed_##line){                                         \
                    if(delayer_##line == 0) delayer_##line = HAL_GetTick(); \
                    if((uint32_t)(HAL_GetTick() - delayer_##line) < (ms)){  \
                        return __VA_ARGS__;                                 \
                    }                                                       \
                    passed_##line = 1;                                      \
                }
            } while(0)


#endif
