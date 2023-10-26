/*
 * movements_buffer.c
 *
 *  Created on: Oct 23, 2023
 *      Author: marti
 */

#include "main.h"
#include "movements_buffer.h"


void init_buffer(volatile movements_buffer_t *buffer){
	buffer->head_index = 0;
	buffer->tail_index = 0;
	buffer->counter = 0;
}

void add_movement(volatile movements_buffer_t *buffer, movement_t movement){
	if(!is_full(buffer)){
		buffer->movements[buffer->head_index] = movement;
		_increment_index(&buffer->head_index);
		buffer->counter++;
	}
}

movement_t get_movement(volatile movements_buffer_t *buffer){
	if(is_empty(buffer)){
		Error_Handler();
	}
	movement_t movement = buffer->movements[buffer->tail_index];
	_increment_index(&buffer->tail_index);
	buffer->counter--;
	return movement;
}

uint8_t is_full(volatile movements_buffer_t *buffer){
	return buffer->counter == MOVES_BUFFER_SIZE;
}

uint8_t is_empty(volatile movements_buffer_t *buffer){
	return buffer->counter == 0;
}


void _increment_index(volatile uint16_t *buffer_index){

	(*buffer_index) = ((*buffer_index) + 1) % MOVES_BUFFER_SIZE;
}
