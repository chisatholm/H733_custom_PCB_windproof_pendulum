/*
 * diagnostics.c
 *
 *  Created on: Feb 23, 2026
 *      Author: andrewchisholm
 */

#include "main.h"
#include "diagnostics.h"
#include "sensor_manager.h"

void DIAG_TimerCallback(void) {
    HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
    SENSOR_StartAcquisition();
}
