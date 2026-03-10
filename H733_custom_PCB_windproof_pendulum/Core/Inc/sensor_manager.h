/*
 * sensor_manager.h
 *
 *  Created on: Feb 23, 2026
 *      Author: andrewchisholm
 */
#ifndef INC_SENSOR_MANAGER_H_
#define INC_SENSOR_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>
#include "tim.h"
#include "pendulum.h"

typedef struct {
    float angle_rad;
    int32_t raw_encoder;

    float gyro_z_rps;
    float accel_x_mps2;

    uint32_t timestamp_us;
    bool encoder_valid;
    bool imu_valid;
} SensorData_t;

// Function prototypes
void SENSOR_Init(void);
void SENSOR_TimerCallback(void);
void SENSOR_StartAcquisition(void);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void SENSOR_OnDataReady(void);

void SENSOR_GetLatest(SensorData_t *out_data); // TODO remove if not used

void BMI270_Write_Reg(uint8_t reg, uint8_t data);
uint8_t BMI270_Read_Reg(uint8_t reg);

#endif /* INC_SENSOR_MANAGER_H_ */
