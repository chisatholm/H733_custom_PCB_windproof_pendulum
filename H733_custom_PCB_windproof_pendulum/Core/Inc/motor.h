/*
 * motor.h
 *
 *  Created on: Dec 31, 2025
 *      Author: andrewchisholm
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>
#include <cmsis_os.h> // For TickType_t and osKernelGetTickCount
//#include "config.h"


#define LUT_SIZE 48  // PWM sinusoid lookup table size
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define ELEC_OFFSET 0.0f

// --- 1. Struct definitions
typedef struct {
    TIM_HandleTypeDef *timer;  // Pointer to the HAL timer handle (e.g., &htim8)

    // Enable Pin (DRV8313 Sleep/Enable)
    GPIO_TypeDef* en_port;     // e.g., GPIOB
    uint16_t en_pin;           // e.g., GPIO_PIN_15

    // Channel Mappings
    // Using uint32_t because HAL defines like TIM_CHANNEL_1 are uint32_t
    uint32_t channel_a;
    uint32_t channel_b;
    uint32_t channel_c;

    // Electrical Offset
    // When the pendulum hangs straight down, there are two electrical positions
    // that produce zero torque. One is due to repulsion, the other attraction.
    // We want the attraction one, which wants to retain the pendulum in its current location.
    // The repulsion one wants the pendulum to go away, in either direction. The correct
    // offset therefore has a shallower slope of d torque / d elec_pos
    // elec_offset is added to the elec_position fed into Motor_Drive()
    int16_t elec_offset;
} Motor_t;

// --- 2. Variable declarations     ---
extern const uint16_t pwm_sin[LUT_SIZE];

// --- 3. Function Prototypes (API) ---

/**
 * @brief Starts the timer necessary to control the motor.
 * @param *motor: pointer to the motor instance.
 * @param *htim: pointer to the timer instance.
 * @caution "power" runs from 0 to (whatever the counter period set for TIM_8 may be - 1), currently 1199. See static void MX_TIM8_Init(void) in main.c
 */
void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *htim);

/**
 * @brief Updates PWM instructions to motor
 * @param elec_position: output of elec_pos() function, runs 0-47, used with sinusoidal lookup table
 * @param torque, the torque to be applied to the motor, in the positive or negative direction.
 * @caution "power" runs from 0 to (whatever the counter period set for TIM_8 may be - 1), currently 2399. See static void MX_TIM8_Init(void) in main.c
 */
void Motor_Drive(Motor_t *motor, int16_t elec_position, int16_t torque);

/**
 * @brief Updates PWM instructions to motor
 * @param elec_position: output of elec_pos() function, runs 0-1079, used with sinusoidal lookup table
 * @param torque, the torque to be applied to the motor, in the positive or negative direction.
 * @caution "power" runs from 0 to (whatever the counter period set for TIM_8 may be - 1), currently 2399. See static void MX_TIM8_Init(void) in main.c
 */
void Motor_Driver(Motor_t *motor, uint16_t elec_position, int16_t torque);

/**
 * @brief Returns electrical position of the motor given the physical
 * @param positon_reading: the centred 14 bit position reading from the AS5048A, runs from -8192 to +8191, 0 at centre of swing.
 * @param offset: empirical offset required. When the AS5048A says the position is zero, the electrical position may be anything from 0-47 inclusive. "offset" accommodates this.
 * @retval electrical position from 0 - 47
 */
uint16_t Elec_Pos(int16_t position_reading, uint16_t offset);

/**
 * @brief Returns electrical position of the motor given the physical
 * @param positon_reading: the raw 14 bit position reading from the AS5048A, runs from 0 to +16383.
 * @param offset: empirical offset required.
 * @retval electrical position from 0 - 1079
 */
uint16_t Elec_Pos_1080(int16_t position_reading, uint16_t offset);

/**
 * @brief Maps a value in one range onto a value in another range
 * @param input, the value in the input range to be mapped.
 * @param in_low_val, the low end of the input range
 * @param in_hi_val, the high end of the input range
 * @param out_low_val, the low end of the output range
 * @param out_hi_val, the high end of the output range
 * @retval electrical position from 0 - 47
 */
// TODO could add a guard against out of range input and output
int32_t Mapping(int16_t input, int16_t in_low_val, int16_t in_hi_val, int16_t out_low_val, int16_t out_hi_val);

#endif /* INC_MOTOR_H_ */
