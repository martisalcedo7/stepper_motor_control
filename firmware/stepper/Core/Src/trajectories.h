#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include <stdint.h>

#define PERIOD 0.01f
#define FREQUENCY (1.0f / PERIOD)

#define STEP_SIZE 0.001541507681f //In rad

void smooth_trajectory(float xf, float yf, float current_theta_1, float current_theta_2, uint16_t array_size, float *x_positions, float *y_positions);

float _smooth_trajectory_point(float xf, float xi, float t, float total_time);

uint16_t smooth_trajectory_size(float xf, float yf, float current_theta_1, float current_theta_2, float v_max_cartesian, float a_max_cartesian);

void smooth_trajectory_and_inverse_kinematics(float xf, float yf, float current_theta_1, float current_theta_2, uint16_t array_size, float *theta_1, float *theta_2);

void calculate_next_step_time_increment(volatile float* positions,
		volatile uint16_t positions_size, volatile uint16_t *current_position_index,
		volatile float *accumulated_position_stepper, volatile float *accumulated_time_stepper,
		volatile uint8_t *sign, volatile float *time_increment);

#endif
