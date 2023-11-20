#ifndef KINEMATICS_H_
#define KINEMATICS_H_

typedef struct {
  float theta_1;
  float theta_2;
} angles_t;

typedef struct {
  float l_1;
  float l_2;
} robot_t;

typedef struct {
  float x;
  float y;
} point_t;

typedef struct {
  float xx;
  float xy;
  float yx;
  float yy;
} matrix_t;

point_t forward_kinematics(angles_t angles, robot_t robot);

matrix_t calculate_jacobian(angles_t angles, robot_t robot);

matrix_t transpose_matrix(matrix_t matrix);

angles_t static_inverse_kinematics(point_t target_coordinates,
                                   angles_t initial_angles, robot_t robot,
                                   float acceptable_squared_error,
                                   int max_iterations, float alpha);

angles_t dynamic_inverse_kinematics(point_t target_coordinates,
                                    angles_t current_angles, robot_t robot,
                                    point_t desired_velocity, float gain_k,
                                    float T_sampling);

float euclidean_distance_squared(point_t point_1, point_t point_2);

#endif // KINEMATICS_H_