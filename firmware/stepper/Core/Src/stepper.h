/*
 * stepper.h
 *
 *  Created on: Oct 12, 2023
 *      Author: marti
 */

#ifndef SRC_STEPPER_H_
#define SRC_STEPPER_H_

#include "stm32f4xx_hal.h"

#define DIR_CCW	0
#define DIR_CW	1

// The angular units are expressed with deg, deg/s and deg/s2

typedef struct
{
	GPIO_TypeDef *     in_gpio[4];
	uint16_t           in_pin[4];
	uint16_t           steps_per_rev;
	uint8_t            max_index;
	TIM_HandleTypeDef* htim;
	uint32_t           htim_frequency;
	uint16_t           max_vel_x10;
}STEPPER_config;


typedef struct
{
	int32_t    steps;
	uint8_t     step_index;
}STEPPER_status;


typedef struct
{
	uint8_t     direction;
	uint16_t    array_size;
	float       *positions;
	uint16_t    current_position_index;
	float       accumulated_position;
	float       accumulated_time;
	float       step_time_increment;
}MOVEMENT_status;


const static uint8_t UNIPOLAR_HALF_STEP_PATTERN[8][4] = {
		{1, 0, 0, 0},
		{1, 1, 0, 0},
		{0, 1, 0, 0},
		{0, 1, 1, 0},
		{0, 0, 1, 0},
		{0, 0, 1, 1},
		{0, 0, 0, 1},
		{1, 0, 0, 1}
};

void turn_off(uint8_t stepper_index);
void take_one_step(uint8_t stepper_index);
void stepper_interrupt_call(TIM_HandleTypeDef* htim);
void set_joint_movement(uint8_t stepper_index, float theta_final, float v_max, float a_max);
void set_cartesian_movement(float xf, float yf, float current_theta_1, float current_theta_2, float v_max_cartesian, float a_max_cartesian);
void start_movement(uint8_t stepper_index);
void start_all_movements(void);
float get_stepper_position(uint8_t stepper_index);
uint8_t is_moving(uint8_t stepper_index);

#endif /* SRC_STEPPER_H_ */
