/*
 * stepper.h
 *
 *  Created on: Oct 12, 2023
 *      Author: marti
 */

#ifndef SRC_STEPPER_H_
#define SRC_STEPPER_H_

#include "stm32f4xx_hal.h"

#define DIR_CW	0
#define DIR_CCW	1


typedef struct
{
	GPIO_TypeDef * in_gpio[4];
	uint16_t       in_pin[4];
	uint16_t       steps_per_rev;
	uint8_t   max_index;
	TIM_HandleTypeDef* htim;
	uint32_t htim_frequency;
}STEPPER_config;


typedef struct
{
	uint32_t  steps;
	uint16_t  rpm_x10;
	uint8_t   blocked;
	uint8_t   step_index;
	uint8_t   direction;
}STEPPER_status;


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
void set_timer(uint8_t stepper_index);
void stepper_interrupt_call(TIM_HandleTypeDef* htim);
void set_movement(uint8_t stepper_index, uint16_t degres_x10, uint16_t rpm_x10, uint8_t direction);
void start_movement(uint8_t stepper_index);
void start_all_movements(void);
uint8_t is_moving(uint8_t stepper_index);
uint32_t deg_to_steps(uint16_t degres_x10, uint16_t steps_per_rev);

#endif /* SRC_STEPPER_H_ */
