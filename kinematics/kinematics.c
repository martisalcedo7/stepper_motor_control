/*
C lib to calculate the kinematics of a two linked robotic arm.

System configuration variables:
 l_1 -> Length of the first link
 l_2 -> Length of the second link
 theta_1 -> Angle between the X axis and the first link (l_1)
 theta_2 -> Angle between the first link (l_1) and the second link (l_2)

The cartesian base is an orthogonal direct base.
- Y axis is vertical pointing up.
- X axis is horizontal pointing right.
- Positive angle is a rotation from X to Y.
*/

#include <math.h>
#include <stdio.h>
#include "kinematics.h"

point_t forward_kinematics(angles_t angles, robot_t robot) {
  point_t coordinates;
  coordinates.x = robot.l_1 * cos(angles.theta_1) +
                  robot.l_2 * cos(angles.theta_1 + angles.theta_2);
  coordinates.y = robot.l_1 * sin(angles.theta_1) +
                  robot.l_2 * sin(angles.theta_1 + angles.theta_2);
  return coordinates;
}

matrix_t calculate_jacobian(angles_t angles, robot_t robot) {
  matrix_t jacobian;
  jacobian.xx = -robot.l_1 * sin(angles.theta_1) -
                robot.l_2 * sin(angles.theta_1 + angles.theta_2);
  jacobian.xy = -robot.l_2 * sin(angles.theta_1 + angles.theta_2);
  jacobian.yx = robot.l_1 * cos(angles.theta_1) +
                robot.l_2 * cos(angles.theta_1 + angles.theta_2);
  jacobian.yy = robot.l_2 * cos(angles.theta_1 + angles.theta_2);
  return jacobian;
}

matrix_t transpose_matrix(matrix_t matrix) {
  matrix_t new_matrix;
  new_matrix.xx = matrix.xx;
  new_matrix.xy = matrix.yx;
  new_matrix.yx = matrix.xy;
  new_matrix.yy = matrix.yy;
  return new_matrix;
}

float euclidean_distance_squared(point_t point_1, point_t point_2) {
  return pow((point_1.x - point_2.x), 2.0) +
         pow((point_1.y - point_2.y), 2.0);
}

angles_t static_inverse_kinematics(point_t target_coordinates,
                                   angles_t initial_angles, robot_t robot,
                                   float acceptable_squared_error,
                                   int max_iterations, float alpha) {

  angles_t angles;
  matrix_t jacobian_transposed;
  angles_t increment_angles;
  point_t coordinates;
  point_t error_coordinates;

  angles = initial_angles;
  coordinates = forward_kinematics(angles, robot);

  error_coordinates.x = target_coordinates.x - coordinates.x;
  error_coordinates.y = target_coordinates.y - coordinates.y;

  int counter = 0;
  while ((euclidean_distance_squared(target_coordinates, coordinates) >
          acceptable_squared_error) &&
         (counter < max_iterations)) {

    jacobian_transposed = transpose_matrix(calculate_jacobian(angles, robot));

    increment_angles.theta_1 =
        ((jacobian_transposed.xx * error_coordinates.x) +
         (jacobian_transposed.xy * error_coordinates.y)) *
        alpha;
    increment_angles.theta_2 =
        ((jacobian_transposed.yx * error_coordinates.x) +
         (jacobian_transposed.yy * error_coordinates.y)) *
        alpha;

    angles.theta_1 = angles.theta_1 + increment_angles.theta_1;
    angles.theta_2 = angles.theta_2 + increment_angles.theta_2;
    coordinates = forward_kinematics(angles, robot);

    error_coordinates.x = target_coordinates.x - coordinates.x;
    error_coordinates.y = target_coordinates.y - coordinates.y;

    counter++;
  }

  printf("%.6f, %.6f, It: %d\n", angles.theta_1, angles.theta_2, counter);
  printf("%.6f, %.6f\n", coordinates.x, coordinates.y);

  return angles;
}

angles_t dynamic_inverse_kinematics(point_t target_coordinates,
                                    angles_t current_angles, robot_t robot,
                                    point_t desired_velocity, float gain_k,
                                    float T_sampling) {

  angles_t angles;
  matrix_t jacobian_transposed;
  angles_t angles_velocity;
  point_t coordinates;
  point_t coordinates_velocity;
  point_t error_coordinates;

  angles = current_angles;
  coordinates = forward_kinematics(angles, robot);

  error_coordinates.x = target_coordinates.x - coordinates.x;
  error_coordinates.y = target_coordinates.y - coordinates.y;

  coordinates_velocity.x = (error_coordinates.x * gain_k) + desired_velocity.x;
  coordinates_velocity.y = (error_coordinates.y * gain_k) + desired_velocity.y;

  jacobian_transposed = transpose_matrix(calculate_jacobian(angles, robot));

  angles_velocity.theta_1 = (jacobian_transposed.xx * coordinates_velocity.x) +
                            (jacobian_transposed.xy * coordinates_velocity.y);
  angles_velocity.theta_2 = (jacobian_transposed.yx * coordinates_velocity.x) +
                            (jacobian_transposed.yy * coordinates_velocity.y);

  angles.theta_1 = angles.theta_1 + (angles_velocity.theta_1 * T_sampling);
  angles.theta_2 = angles.theta_2 + (angles_velocity.theta_2 * T_sampling);

  return angles;
}
