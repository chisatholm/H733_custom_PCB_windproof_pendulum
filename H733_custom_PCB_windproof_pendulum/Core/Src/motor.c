/*
 * motor.c
 *
 *  Created on: Dec 31, 2025
 *      Author: andrewchisholm
 */


#include "main.h" // Includes the TIM_HandleTypeDef definition
#include "motor.h"
//#include "pwm_array.h"

extern TIM_HandleTypeDef htim1;
//extern int16_t torque_debug;
//extern uint16_t elec_pos_debug;
//extern uint32_t ret_val_debug;
//extern int32_t mapping_debug;

const uint16_t pwm_sin[] = { 	127, 	110, 	94, 	78, 	64, 	50, 	37, 	26,
								17, 	10, 	4, 		1, 		0, 		1, 		4, 		10,
								17, 	26, 	37, 	50, 	64, 	79, 	95, 	111,
								128, 	144, 	160, 	176, 	191, 	205, 	218, 	229,
								238, 	245, 	251, 	254, 	255, 	254, 	251, 	245,
								238, 	229, 	218, 	205, 	191, 	176, 	160, 	144};


void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *htim) {
	// 1. Link the timer handle
	    motor->timer = htim;

	// 2. Set all PWM duty cycles to 0 immediately (Safety)
	__HAL_TIM_SET_COMPARE(motor->timer, motor->channel_a, 0);
	__HAL_TIM_SET_COMPARE(motor->timer, motor->channel_b, 0);
	__HAL_TIM_SET_COMPARE(motor->timer, motor->channel_c, 0);

	// 3. Enable the DRV8313 hardware
	if (motor->en_port != NULL) {
		HAL_GPIO_WritePin(motor->en_port, motor->en_pin, GPIO_PIN_SET);
	}

	// 4. Start the PWM hardware
	HAL_TIM_PWM_Start(motor->timer, motor->channel_a);
	HAL_TIM_PWM_Start(motor->timer, motor->channel_b);
	HAL_TIM_PWM_Start(motor->timer, motor->channel_c);
}

void Motor_Drive(Motor_t *motor, int16_t elec_position, int16_t torque){

	// 00, 16 and 32 below come from the choice to divide the electrical position cycle into 48 segments.
	// 48 is somewhat arbitrary - any multiple of 2 x 2 x 3 can work. Higher number -> smoother motion.
	// 48 works well enough, so stick with it.
	// The PWM settings found in the lookup table "pwmSin" for the three phases of the BLDC motor should
	// always be 120 electrical degrees apart.
	// Given 48 possible electrical positions in a full cycle, 120 degrees apart translates to
	// elec_position + (0 / 3) * 48
	// elec_position + (1 / 3) * 48
	// elec_position + (2 / 3) * 48
	// = e_p + 00, e_p + 16, e_p + 32
	// The other magic number below, 255, comes from the choice to make the pwm_sin lookup table
	// an '8 bit' table, running from 0 - 255. It could be any number, again more -> better, but
	// 255 works fine, stick with it.

	elec_position += - 12 + ((torque > 0) ? 24 : 0); // TODO check that +ve torque actually moves in the correct direction
	elec_position += motor->elec_offset;
	if (torque < 0){torque *= -1;}

	__HAL_TIM_SET_COMPARE(motor->timer, motor->channel_a, pwm_sin[(elec_position + 00) % 48] * torque / 255.0);
	__HAL_TIM_SET_COMPARE(motor->timer, motor->channel_b, pwm_sin[(elec_position + 16) % 48] * torque / 255.0);
	__HAL_TIM_SET_COMPARE(motor->timer, motor->channel_c, pwm_sin[(elec_position + 32) % 48] * torque / 255.0);
}

//void Motor_Driver(Motor_t *motor, uint16_t elec_position, int16_t torque){
//
//	// 00, 360 and 720 below come from the choice to divide the electrical position cycle into 1080 segments.
//	// 1080 is somewhat arbitrary - any multiple of 2 x 2 x 3 can work. Higher number -> smoother motion.
//	// 1080 is an upgrade from 48.
//	// The PWM settings, found in the lookup table in pwm_array.c, for the three phases of the BLDC motor should
//	// always be 120 electrical degrees apart.
//	// Given 1080 possible electrical positions in a full cycle, 120 degrees apart translates to
//	// elec_position + (0 / 3) * 1080
//	// elec_position + (1 / 3) * 1080
//	// elec_position + (2 / 3) * 1080
//	// = e_p + 00, e_p + 360, e_p + 720
//	// The other magic number below, 2048, comes from the choice to make the lookup table
//	// an '11 bit' table, running from 0 - 2048. It could be any number, again more -> better, but
//	// 11 bit is an upgrade from 0-255.
//	elec_position += + 270 + ((torque < 0) ? 540 : 0); // TODO check that +ve torque actually moves in the correct direction
//	elec_position += motor->elec_offset;
//	if (torque < 0){torque *= -1;}
//	torque_debug = torque;
//
//	uint32_t A = (Index_to_PWM((elec_position + 000) % 1080) * torque) / 2048;
//	uint32_t B = (Index_to_PWM((elec_position + 360) % 1080) * torque) / 2048;
//	uint32_t C = (Index_to_PWM((elec_position + 720) % 1080) * torque) / 2048;
//
//	__HAL_TIM_SET_COMPARE(motor->timer, motor->channel_a, A);
//	__HAL_TIM_SET_COMPARE(motor->timer, motor->channel_b, B);
//	__HAL_TIM_SET_COMPARE(motor->timer, motor->channel_c, C);
//}



uint16_t Elec_Pos(int16_t position_reading, uint16_t offset) {
	int16_t dummy = Mapping(position_reading, -8192, 8191, 0, 48 * 7);
	dummy += offset;
	dummy = dummy % 48;
	return dummy;
}

uint16_t Elec_Pos_1080(int16_t position_reading, uint16_t offset){

	int32_t dummy = Mapping(position_reading, 0, 16383, 0, 7560); // 7560 = 7 * 1080, 7 electrical revolutions per 1 mechanical revolution
	dummy += offset;
	while (dummy < 0){dummy += 1080;}
	dummy = dummy % 1080;
	dummy = 1080 - (uint16_t)dummy;
	return dummy;
}

int32_t Mapping(int16_t input, int16_t in_low_val, int16_t in_hi_val, int16_t out_low_val, int16_t out_hi_val){
	int32_t in_range = in_hi_val - in_low_val;
	int32_t out_range = out_hi_val - out_low_val;
	int32_t dummy = (input - in_low_val) * out_range / in_range  + out_low_val;
	int32_t ret_val =  dummy;
	return ret_val;
}
