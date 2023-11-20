/*
 * stepper.c
 *
 *  Created on: Oct 12, 2023
 *      Author: marti
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include "stepper.h"
#include <math.h>
#include <stdlib.h>
#include "trajectories.h"

// The angular units are expressed with deg, deg/s and deg/s2

#define STEPPER_UNITS 2

//#define INV_SQRT_10 0.31622776601683793319
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
		 .step_index = 0,
		 .direction = DIR_CCW},
		{.steps = 0,
		 .step_index = 0,
		 .direction = DIR_CCW}
};

static volatile MOVEMENT_status movement_status[STEPPER_UNITS] = {
		{
			.taken_steps = 0,
			.array_size = 0,
			.positions = NULL,
			.current_position_index = 0,
			.accumulated_position = 0,
			.accumulated_time = 0,
			.step_time_increment = 0
		},
		{
			.taken_steps = 0,
			.array_size = 0,
			.positions = NULL,
			.current_position_index = 0,
			.accumulated_position = 0,
			.accumulated_time = 0,
			.step_time_increment = 0
		}

};

float get_stepper_position(uint8_t stepper_index){
	return movement_status[stepper_index].taken_steps * STEP_SIZE;
}

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
		if(steppers_status[stepper_index].step_index == steppers_config[stepper_index].max_index)
		{
			steppers_status[stepper_index].step_index = 0;
		}else{
			steppers_status[stepper_index].step_index++;
		}
	}
	else if(steppers_status[stepper_index].direction == DIR_CCW)
	{

		if(steppers_status[stepper_index].step_index == 0)
		{
			steppers_status[stepper_index].step_index = steppers_config[stepper_index].max_index;
		}else{
			steppers_status[stepper_index].step_index--;
		}
	}
}

uint32_t degrees_x100_to_steps(uint32_t degrees_x100, uint16_t steps_per_rev){
	return (degrees_x100 * steps_per_rev)/36000;
}


void set_movement(uint8_t stepper_index, float theta_final, float v_max, float a_max){

	// Calculate array size for the movement

	float t_vmax = 1.875f * fabsf(theta_final - 0.0) / v_max;
	float t_amax = 2.40281141413475f * sqrtf(fabsf(theta_final - 0.0) / a_max);
	float tf = fmaxf(t_vmax, t_amax);

	movement_status[stepper_index].array_size = (uint16_t)(tf * FREQUENCY);

	// Allocate movement arrays
	movement_status[stepper_index].positions = malloc(movement_status[stepper_index].array_size * sizeof(float));

	if (!movement_status[stepper_index].positions) {
		return;
	}

    float total_time = PERIOD * movement_status[stepper_index].array_size;
    for (uint16_t idx = 0; idx < movement_status[stepper_index].array_size; ++idx)
    {
        float t = idx * PERIOD;
        movement_status[stepper_index].positions[idx] = _smooth_trajectory_point(theta_final, 0, t, total_time);
    }

    calculate_next_step_time_increment(movement_status[stepper_index].positions,
    		movement_status[stepper_index].array_size, &movement_status[stepper_index].current_position_index,
			&movement_status[stepper_index].accumulated_position, &movement_status[stepper_index].accumulated_time,
			&steppers_status[stepper_index].direction, &movement_status[stepper_index].step_time_increment);

    // Configures timer to set the velocity for the first step
	uint16_t counts = (uint16_t)(steppers_config[stepper_index].htim_frequency * movement_status[stepper_index].step_time_increment);
	// Check defined period limits
	if(counts < 1 || counts > 65535){
		Error_Handler();
	}
	__HAL_TIM_SET_AUTORELOAD(steppers_config[stepper_index].htim, (counts - 1));
}

void set_cartesian_movement(float xf, float yf, float current_theta_1, float current_theta_2, float v_max_cartesian, float a_max_cartesian){

	uint16_t array_size = smooth_trajectory_size(xf, yf, current_theta_1, current_theta_2, v_max_cartesian, a_max_cartesian);

	movement_status[0].array_size = array_size;
	movement_status[1].array_size = array_size;

	// Allocate movement arrays
	movement_status[0].positions = malloc(movement_status[0].array_size * sizeof(float));
	movement_status[1].positions = malloc(movement_status[1].array_size * sizeof(float));

	if (!movement_status[0].positions || !movement_status[1].positions) {
		return;
	}

	smooth_trajectory_and_inverse_kinematics(xf, yf, current_theta_1, current_theta_2, array_size, movement_status[0].positions, movement_status[1].positions);

    calculate_next_step_time_increment(movement_status[0].positions,
    		movement_status[0].array_size, &movement_status[0].current_position_index,
			&movement_status[0].accumulated_position, &movement_status[0].accumulated_time,
			&steppers_status[0].direction, &movement_status[0].step_time_increment);

    // Configures timer to set the velocity for the first step
	uint16_t counts = (uint16_t)(steppers_config[0].htim_frequency * movement_status[0].step_time_increment);
	// Check defined period limits
	if(counts < 1 || counts > 65535){
		Error_Handler();
	}
	__HAL_TIM_SET_AUTORELOAD(steppers_config[0].htim, (counts - 1));

    calculate_next_step_time_increment(movement_status[1].positions,
    		movement_status[1].array_size, &movement_status[1].current_position_index,
			&movement_status[1].accumulated_position, &movement_status[1].accumulated_time,
			&steppers_status[1].direction, &movement_status[1].step_time_increment);

    // Configures timer to set the velocity for the first step
	counts = (uint16_t)(steppers_config[1].htim_frequency * movement_status[1].step_time_increment);
	// Check defined period limits
	if(counts < 1 || counts > 65535){
		Error_Handler();
	}
	__HAL_TIM_SET_AUTORELOAD(steppers_config[1].htim, (counts - 1));
}

void clear_movement(uint8_t stepper_index){
	movement_status[stepper_index].array_size = 0;
	free(movement_status[stepper_index].positions);
	movement_status[stepper_index].current_position_index = 0;
	movement_status[stepper_index].accumulated_position = 0;
	movement_status[stepper_index].accumulated_time = 0;
	movement_status[stepper_index].step_time_increment = 0;
}

uint8_t is_moving(uint8_t stepper_index){
	return steppers_config[stepper_index].htim->State == HAL_TIM_STATE_BUSY;
}

void start_movement(uint8_t stepper_index){
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

			take_one_step(stepper_index);
			if(steppers_status[stepper_index].direction){
				movement_status[stepper_index].taken_steps--;
			}else{
				movement_status[stepper_index].taken_steps++;
			}

		    calculate_next_step_time_increment(movement_status[stepper_index].positions,
		    		movement_status[stepper_index].array_size, &movement_status[stepper_index].current_position_index,
					&movement_status[stepper_index].accumulated_position, &movement_status[stepper_index].accumulated_time,
					&steppers_status[stepper_index].direction, &movement_status[stepper_index].step_time_increment);


		    if (movement_status[stepper_index].step_time_increment > 0){
				// Configures time to next step
				uint16_t counts = (uint16_t)(steppers_config[stepper_index].htim_frequency * movement_status[stepper_index].step_time_increment);
				// Check defined period limits
				if(counts < 1 || counts > 65535){
					Error_Handler();
				}
				__HAL_TIM_SET_AUTORELOAD(steppers_config[stepper_index].htim, (counts - 1));
		    }else{
		    	clear_movement(stepper_index);
		    	HAL_TIM_Base_Stop_IT(steppers_config[stepper_index].htim);
		    }
		}
	}
}
