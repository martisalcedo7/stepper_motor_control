#include "kinematics.h"
#include <stdio.h>

int main(void) {

  robot_t robot;
  robot.l_1 = 1.0;
  robot.l_2 = 1.0;

  point_t target;
  target.x = 0.5;
  target.y = 0.0;

  angles_t angles;
  angles.theta_1 = 0.5;
  angles.theta_2 = 0.0;

  angles_t target_angles =
      static_inverse_kinematics(target, angles, robot, 0.00000001, 1000, 0.1);

  printf("%.6f, %.6f \n", target_angles.theta_1, target_angles.theta_2);

  return 0;
}