#ifndef _S_BLUETOOTH_H_
#define _S_BLUETOOTH_H_

#include "main.h"

extern UART_HandleTypeDef huart1;

/* ========================= 接 口 函 数 声 明 ========================= */

void BlueToothInit(void);
int BlueToothProcess(void);

#endif
