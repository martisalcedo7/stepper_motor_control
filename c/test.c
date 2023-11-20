#include "trajectories.h"
#include <stdio.h>
#include <stdlib.h>

int main(void) {

  float xf = -0.05;
  float yf = 0.07;

  const float v_max_cartesian = 0.1;
  const float a_max_cartesian = 0.1;

  float current_theta_1 = 0.1;
  float current_theta_2 = 0.1;

  uint16_t array_size =
      smooth_trajectory_size(xf, yf, current_theta_1, current_theta_2,
                             v_max_cartesian, a_max_cartesian);
  printf("%u\n", array_size);

  float *theta_1 = malloc(array_size * sizeof(float));
  float *theta_2 = malloc(array_size * sizeof(float));

  float *x_positions = malloc(array_size * sizeof(float));
  float *y_positions = malloc(array_size * sizeof(float));

  if (!theta_1 || !theta_2 || !x_positions || !y_positions) {
    return -1;
  }

  smooth_trajectory_angular(xf, yf, current_theta_1, current_theta_2,
                            array_size, theta_1, theta_2);
  smooth_trajectory_cartesian(xf, yf, current_theta_1, current_theta_2,
                              array_size, x_positions, y_positions);

  for (uint16_t idx = 0; idx < array_size; idx++) {
    printf("%.4f, %.4f, %.4f, %.4f\n", theta_1[idx], theta_2[idx],
           x_positions[idx], y_positions[idx]);
  }

  uint16_t current_position_index = 0;
  float accumulated_position_stepper = 0;
  float accumulated_time_stepper = 0;
  float step_time_increment = 0;

  while(step_time_increment >= 0){
    float sign;
    // step_time_increment = calculate_next_step_time(theta_1, array_size, &current_position_index, &accumulated_position_stepper, &sign);
    calculate_next_step_time_increment(theta_1, array_size, &current_position_index, &accumulated_position_stepper, &accumulated_time_stepper, &sign, &step_time_increment);
    printf("%.8f, %.8f\n", step_time_increment, sign);
  }

  free(theta_1);
  free(theta_2);
  free(x_positions);
  free(y_positions);

  return 0;
}
