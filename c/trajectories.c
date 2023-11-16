#include "trajectories.h"
#include "robot.h"
#include <math.h>
#include <stdio.h>

void smooth_trajectory_cartesian(float xf, float yf, float current_theta_1, float current_theta_2, uint16_t array_size, float *x_positions, float *y_positions)
{
    float xi;
    float yi;

    forward_kinematics(current_theta_1, current_theta_2, &xi, &yi);

    float total_time = PERIOD * array_size;
    for (uint16_t idx = 0; idx < array_size; ++idx)
    {
        float t = idx * PERIOD;
        x_positions[idx] = smooth_trajectory_point(xf, xi, t, total_time);
        y_positions[idx] = smooth_trajectory_point(yf, yi, t, total_time);
    }
}

float smooth_trajectory_point(float xf, float xi, float t, float total_time)
{
    float normalized_time = t / total_time;
    float position = (xf - xi) * (10.0f * powf(normalized_time, 3) - 15.0f * powf(normalized_time, 4) + 6.0f * powf(normalized_time, 5)) + xi;
    return position;
}

uint16_t smooth_trajectory_size(float xf, float yf, float current_theta_1, float current_theta_2, float v_max_cartesian, float a_max_cartesian)
{
    float xi;
    float yi;

    forward_kinematics(current_theta_1, current_theta_2, &xi, &yi);

    float initial_point = xi;
    float final_point = xf;

    if (fabsf(xf - xi) < fabsf(yf - yi))
    {
        initial_point = yi;
        final_point = yf;
    }

    float t_vmax = 1.875f * fabsf(final_point - initial_point) / v_max_cartesian;
    float t_amax = 2.40281141413475f * sqrtf(fabsf(final_point - initial_point) / a_max_cartesian);
    float tf = fmaxf(t_vmax, t_amax);

    printf("%.4f\n", tf);

    return (uint16_t)(tf * FREQUENCY);
}

void smooth_trajectory_angular(float xf, float yf, float current_theta_1, float current_theta_2, uint16_t array_size, float *theta_1, float *theta_2)
{
    float xi;
    float yi;

    forward_kinematics(current_theta_1, current_theta_2, &xi, &yi);

    float total_time = PERIOD * array_size;
    for (uint16_t idx = 0; idx < array_size; ++idx)
    {
        float t = idx * PERIOD;
        float x_position = smooth_trajectory_point(xf, xi, t, total_time);
        float y_position = smooth_trajectory_point(yf, yi, t, total_time);

        inverse_kinematics(x_position, y_position, current_theta_1, current_theta_2, &theta_1[idx], &theta_2[idx]);

        current_theta_1 = theta_1[idx];
        current_theta_2 = theta_2[idx];
    }
}




float calculate_next_step_time(const float* positions, uint16_t positions_size, uint16_t *current_position_index, float *accumulated_position_stepper, float *sign) {

    float t_stepper = -1;

    while(*current_position_index < (positions_size - 1)){

        float pos_diff = positions[*current_position_index + 1] - positions[*current_position_index];
        float next_pos = *accumulated_position_stepper + STEP_SIZE * (pos_diff > 0 ? 1 : -1);

        if ((pos_diff > 0 && next_pos < (positions[*current_position_index + 1] - positions[0])) || 
            (pos_diff < 0 && next_pos > (positions[*current_position_index + 1] - positions[0]))) {

                float inc_t_stepper = PERIOD * (next_pos - (positions[*current_position_index] - positions[0])) / pos_diff;
                t_stepper = PERIOD * (*current_position_index) + inc_t_stepper;
                *accumulated_position_stepper = *accumulated_position_stepper + STEP_SIZE * (pos_diff > 0 ? 1 : -1);
                *sign = (pos_diff > 0 ? 1.0 : -1.0);
                break;
        }

        (*current_position_index)++;
    }

    return t_stepper;
}