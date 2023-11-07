/*
 * stepper.c
 *
 *  Created on: Oct 12, 2023
 *      Author: marti
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include "stepper.h"
#include "math.h"
#include <stdlib.h>


// The angular units are expressed with deg, deg/s and deg/s2

#define STEPPER_UNITS 2

#define INV_SQRT_10 0.31622776601683793319
#define min(a,b) (((a) < (b)) ? (a) : (b))

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
		.htim_frequency = 250000,
		.max_vel_x10 = 840
	},
    {
	    .in_gpio = {GPIOE, GPIOE, GPIOE, GPIOE},
		.in_pin = {stp_2_0_Pin, stp_2_1_Pin, stp_2_2_Pin, stp_2_3_Pin},
		.steps_per_rev = 4076,
		.max_index = 7,
		.htim = &htim7,
		.htim_frequency = 250000,
		.max_vel_x10 = 840
	}
};


static volatile STEPPER_status steppers_status[STEPPER_UNITS] =
{
		{.steps = 0,
		 .target_steps = 0,
		 .vel_x10_p = NULL,
		 .step_index = 0,
		 .direction = DIR_CCW},
		{.steps = 0,
		 .target_steps = 0,
		 .vel_x10_p = NULL,
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

uint32_t degrees_x100_to_steps(uint32_t degrees_x100, uint16_t steps_per_rev){
	return (degrees_x100 * steps_per_rev)/36000;
}


void set_movement(uint8_t stepper_index, uint32_t degres_x100, uint16_t vel_x10, uint16_t accel_x10, uint8_t direction){
	if(degres_x100 == 0 || vel_x10 == 0 || accel_x10 == 0 || vel_x10 > steppers_config[stepper_index].max_vel_x10){
		return;
	}
	if(is_moving(stepper_index)){
		return;
	}

	steppers_status[stepper_index].target_steps = degrees_x100_to_steps(degres_x100, steppers_config[stepper_index].steps_per_rev);
	// In case the steps calculation returns zero do not set any movement
	if(steppers_status[stepper_index].target_steps == 0){
		return;
	}

	steppers_status[stepper_index].direction = (direction ? DIR_CCW : DIR_CW);

	steppers_status[stepper_index].vel_x10_p = (uint16_t*)malloc(steppers_status[stepper_index].target_steps * sizeof(uint16_t));
    if (steppers_status[stepper_index].vel_x10_p == NULL) {
    	Error_Handler();
    }
    // For each step we calculate the corresponding velocity following the trapezoidal trajectory
    vel_x10 = min((uint16_t)(INV_SQRT_10 * sqrtf((float)accel_x10 * degres_x100)), vel_x10);

    float aux = vel_x10 * vel_x10 / (float)accel_x10;
    // Number of steps when the ramp becomes flat
    uint32_t steps_1 = (uint32_t)(0.00013888888888888889 * aux * steppers_config[stepper_index].steps_per_rev);
    // Number of steps when the flat becomes slowing ramp
    uint32_t steps_2 = (uint32_t)(0.00002777777777777778 * (degres_x100 - 5.0 * aux) * steppers_config[stepper_index].steps_per_rev);

    float steps_to_degres_x100 = 36000.0 / steppers_config[stepper_index].steps_per_rev;

    for (uint32_t step = 1; step <= steppers_status[stepper_index].target_steps; ++step) {
    	if(step < steps_1){
    		steppers_status[stepper_index].vel_x10_p[step] = (uint16_t)(sqrtf(accel_x10 * steps_to_degres_x100 * step) * INV_SQRT_10 * M_SQRT2);
    	}else if(steps_1 <= step && step <= steps_2){
    		steppers_status[stepper_index].vel_x10_p[step] = vel_x10;
    	}else if(steps_2 < step){
    		steppers_status[stepper_index].vel_x10_p[step] = (uint16_t)(sqrtf(accel_x10 * (degres_x100 - steps_to_degres_x100 * step)) * INV_SQRT_10 * M_SQRT2);
    	}else{
    		Error_Handler();
    	}
    }

    // Configures timer to set the velocity for the first step

	uint32_t counts = (uint32_t)((3600.0 * steppers_config[stepper_index].htim_frequency) / (steppers_status[stepper_index].vel_x10_p[0] * steppers_config[stepper_index].steps_per_rev));
	// Check defined period limits
	if(counts < 1 || counts > 65535){
		Error_Handler();
	}

//	steppers_config[stepper_index].htim->Init.Period = counts - 1;
//	HAL_TIM_Base_Init(steppers_config[stepper_index].htim);
	__HAL_TIM_SET_AUTORELOAD(steppers_config[stepper_index].htim, (counts - 1));
}

uint8_t is_moving(uint8_t stepper_index){
	return steppers_config[stepper_index].htim->State == HAL_TIM_STATE_BUSY;
}

void start_movement(uint8_t stepper_index){
	if(steppers_status[stepper_index].target_steps == 0 || steppers_status[stepper_index].vel_x10_p == NULL){
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
			if(steppers_status[stepper_index].steps < steppers_status[stepper_index].target_steps){
				take_one_step(stepper_index);

				uint32_t counts = (uint32_t)((3600.0 * steppers_config[stepper_index].htim_frequency) / (steppers_status[stepper_index].vel_x10_p[steppers_status[stepper_index].steps] * steppers_config[stepper_index].steps_per_rev));
				if(counts < 1 || counts > 65535){
					Error_Handler();
				}
				__HAL_TIM_SET_AUTORELOAD(steppers_config[stepper_index].htim, (counts - 1));


				steppers_status[stepper_index].steps++;
			}else{
				steppers_status[stepper_index].steps = 0;
				steppers_status[stepper_index].target_steps = 0;
				// Free the memory of the dynamically allocated array for the velocities
				free(steppers_status[stepper_index].vel_x10_p);
				HAL_TIM_Base_Stop_IT(steppers_config[stepper_index].htim);
			}
		}
	}
}
