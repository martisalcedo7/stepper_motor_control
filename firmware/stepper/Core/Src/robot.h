#ifndef ROBOT_H
#define ROBOT_H

#define L1 0.10f
#define L2 0.08f


typedef struct
{
	float l_1;
	float l_2;
	float current_theta_1;
	float current_theta_2;
}ROBOT;


void inverse_kinematics(float x, float y,
                        float current_theta_1, float current_theta_2,
                        float *theta_1, float *theta_2);

void forward_kinematics(float theta_1, float theta_2, float *x, float *y);

#endif
