#include "robot.h"
#include <math.h>

void inverse_kinematics(float x, float y,
                        float current_theta_1, float current_theta_2,
                        float *theta_1, float *theta_2)
{
    float gamma = atan2f(y, x);
    float alpha = acosf((x * x + y * y + L1 * L1 - L2 * L2) / (2.0f * L1 * sqrtf(x * x + y * y)));
    float beta = acosf((L1 * L1 + L2 * L2 - x * x - y * y) / (2.0f * L1 * L2));

    float theta_1_1 = gamma - alpha;
    float theta_2_1 = M_PI - beta;

    float theta_1_2 = gamma + alpha;
    float theta_2_2 = beta - M_PI;

    float dist_1 = (current_theta_1 - theta_1_1) * (current_theta_1 - theta_1_1) +
                   (current_theta_2 - theta_2_1) * (current_theta_2 - theta_2_1);
    float dist_2 = (current_theta_1 - theta_1_2) * (current_theta_1 - theta_1_2) +
                   (current_theta_2 - theta_2_2) * (current_theta_2 - theta_2_2);

    if (dist_1 < dist_2)
    {
        *theta_1 = theta_1_1;
        *theta_2 = theta_2_1;
    }
    else
    {
        *theta_1 = theta_1_2;
        *theta_2 = theta_2_2;
    }
}

void forward_kinematics(float theta_1, float theta_2, float *x, float *y)
{
    *x = L1 * cosf(theta_1) + L2 * cosf(theta_1 + theta_2);
    *y = L1 * sinf(theta_1) + L2 * sinf(theta_1 + theta_2);
}