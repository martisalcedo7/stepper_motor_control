/*
 * stepper.c
 *
 *  Created on: Oct 12, 2023
 *      Author: marti
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include "stepper.h"


//#define ABS(x) ((x) > 0 ? (x) : -(x))

#define STEPPER_UNITS 2

// Access htim2 from main.c
extern TIM_HandleTypeDef htim6, htim7;

const static STEPPER_config steppers_config[STEPPER_UNITS] =
{
	// Stepper Motor 1 Configurations
    {
	    .in_gpio = {GPIOD, GPIOD, GPIOD, GPIOD},
		.in_pin = {stp_1_0_Pin, stp_1_1_Pin, stp_1_2_Pin, stp_1_3_Pin},
		.steps_per_rev = 4076,
		.max_index = 7,
		.htim = &htim6,
		.htim_frequency = 250000
	},
    {
	    .in_gpio = {GPIOE, GPIOE, GPIOE, GPIOE},
		.in_pin = {stp_2_0_Pin, stp_2_1_Pin, stp_2_2_Pin, stp_2_3_Pin},
		.steps_per_rev = 4076,
		.max_index = 7,
		.htim = &htim7,
		.htim_frequency = 250000
	}
};


static volatile STEPPER_status steppers_status[STEPPER_UNITS] =
{
		{.steps = 0,
		 .rpm_x10 = 0,
		 .blocked = 0,
		 .step_index = 0,
		 .direction = DIR_CCW},
		{.steps = 0,
		 .rpm_x10 = 0,
		 .blocked = 0,
		 .step_index = 0,
		 .direction = DIR_CCW}
};

void turn_off(uint8_t stepper_index){
	HAL_GPIO_WritePin(steppers_config[stepper_index].in_gpio[0], steppers_config[stepper_index].in_pin[0], 0);
	HAL_GPIO_WritePin(steppers_config[stepper_index].in_gpio[1], steppers_config[stepper_index].in_pin[1], 0);
	HAL_GPIO_WritePin(steppers_config[stepper_index].in_gpio[2], steppers_config[stepper_index].in_pin[2], 0);
	HAL_GPIO_WritePin(steppers_config[stepper_index].in_gpio[3], steppers_config[stepper_index].in_pin[3], 0);
}


void take_one_step(uint8_t stepper_index){
	HAL_GPIO_WritePin(steppers_config[stepper_index].in_gpio[0], steppers_config[stepper_index].in_pin[0], UNIPOLAR_HALF_STEP_PATTERN[steppers_status[stepper_index].step_index][0]);
	HAL_GPIO_WritePin(steppers_config[stepper_index].in_gpio[1], steppers_config[stepper_index].in_pin[1], UNIPOLAR_HALF_STEP_PATTERN[steppers_status[stepper_index].step_index][1]);
	HAL_GPIO_WritePin(steppers_config[stepper_index].in_gpio[2], steppers_config[stepper_index].in_pin[2], UNIPOLAR_HALF_STEP_PATTERN[steppers_status[stepper_index].step_index][2]);
	HAL_GPIO_WritePin(steppers_config[stepper_index].in_gpio[3], steppers_config[stepper_index].in_pin[3], UNIPOLAR_HALF_STEP_PATTERN[steppers_status[stepper_index].step_index][3]);

	// Update & Check The Index
	if(steppers_status[stepper_index].direction == DIR_CW)
	{
		if(steppers_status[stepper_index].step_index == 0)
		{
			steppers_status[stepper_index].step_index = steppers_config[stepper_index].max_index;
		}else{
			steppers_status[stepper_index].step_index--;
		}
	}
	else if(steppers_status[stepper_index].direction == DIR_CCW)
	{

		if(steppers_status[stepper_index].step_index == steppers_config[stepper_index].max_index)
		{
			steppers_status[stepper_index].step_index = 0;
		}else{
			steppers_status[stepper_index].step_index++;
		}
	}
}

uint32_t deg_to_steps(uint16_t degrees_x10, uint16_t steps_per_rev){
	return (degrees_x10 * steps_per_rev)/3600;
}


void set_movement(uint8_t stepper_index, uint16_t degres_x10, uint16_t rpm_x10, uint8_t direction){
	if(degres_x10 == 0 || rpm_x10 == 0){
		return;
	}
	if(is_moving(stepper_index)){
		return;
	}

	steppers_status[stepper_index].steps = deg_to_steps(degres_x10, steppers_config[stepper_index].steps_per_rev);
	steppers_status[stepper_index].rpm_x10 = rpm_x10;
	steppers_status[stepper_index].direction = (direction ? DIR_CCW : DIR_CW);
	set_timer(stepper_index);
}

void set_timer(uint8_t stepper_index){
	// The input argument rpm_x_10 is rpm times 10
	if(steppers_status[stepper_index].rpm_x10 == 0 || steppers_config[stepper_index].steps_per_rev  == 0){
		Error_Handler();
	}

	uint32_t counts = (uint32_t)((600.0 * steppers_config[stepper_index].htim_frequency) / (steppers_status[stepper_index].rpm_x10 * steppers_config[stepper_index].steps_per_rev));
	// Check defined period limits
	if(counts < 1 || counts > 65535){
		Error_Handler();
	}

	steppers_config[stepper_index].htim->Init.Period = counts - 1;
	HAL_TIM_Base_Init(steppers_config[stepper_index].htim);
//	__HAL_TIM_SET_AUTORELOAD(steppers_config[stepper_index].htim, (counts - 1));
}

uint8_t is_moving(uint8_t stepper_index){
	return steppers_config[stepper_index].htim->State == HAL_TIM_STATE_BUSY;
}

void start_movement(uint8_t stepper_index){
	if(steppers_status[stepper_index].steps == 0 || steppers_status[stepper_index].rpm_x10 == 0){
		return;
	}
	if(steppers_config[stepper_index].htim->State != HAL_TIM_STATE_READY){
		return;
	}
	HAL_TIM_Base_Start_IT(steppers_config[stepper_index].htim);
}

void start_all_movements(void){
	for(uint8_t stepper_index = 0; stepper_index < STEPPER_UNITS; stepper_index++){
		start_movement(stepper_index);
	}
}

void stepper_interrupt_call(TIM_HandleTypeDef* htim){
	for(uint8_t stepper_index = 0; stepper_index < STEPPER_UNITS; stepper_index++){
		if(htim == steppers_config[stepper_index].htim){
			if(steppers_status[stepper_index].steps > 0){
				take_one_step(stepper_index);
				steppers_status[stepper_index].steps--;
			}else{
				HAL_TIM_Base_Stop_IT(steppers_config[stepper_index].htim);
			}
		}
	}
}
