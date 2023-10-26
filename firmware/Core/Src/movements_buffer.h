/*
 * movements_buffer.h
 *
 *  Created on: Oct 19, 2023
 *      Author: marti
 */

#ifndef SRC_MOVEMENTS_BUFFER_H_
#define SRC_MOVEMENTS_BUFFER_H_

#include "stm32f4xx_hal.h"

#define MOVES_BUFFER_SIZE 10

typedef struct
{
	uint16_t  degrees_x10;
	uint16_t  rpm_x10;
	uint8_t direction;
}movement_t;


typedef struct
{
	volatile uint16_t  head_index;
	volatile uint16_t  tail_index;
	volatile uint16_t  counter;
	volatile movement_t   movements[MOVES_BUFFER_SIZE];
}movements_buffer_t;

void init_buffer(volatile movements_buffer_t *buffer);
void add_movement(volatile movements_buffer_t *buffer, movement_t movement);
movement_t get_movement(volatile movements_buffer_t *buffer);
uint8_t is_full(volatile movements_buffer_t *buffer);
uint8_t is_empty(volatile movements_buffer_t *buffer);
void _increment_index(volatile uint16_t *buffer_index);

#endif /* SRC_MOVEMENTS_BUFFER_H_ */
