#ifndef SINGLE_ARM_APP_H
#define SINGLE_ARM_APP_H

#include "stm32g4xx_hal.h"

void SingleArmApp_Init(
    UART_HandleTypeDef *host_uart,
    UART_HandleTypeDef *servo_uart
);
void SingleArmApp_Process(void);

#endif /* SINGLE_ARM_APP_H */
