import matplotlib.pyplot as plt
import numpy as np
import time

l_1 = 1.0
l_2 = 1.0

TIME = 5

def inverse_kinematics_analytical(x,
                                  y,
                                  current_theta_1=None,
                                  current_theta_2=None):
    gamma = np.arctan2(y, x)
    alpha = np.arccos(
        (x**2 + y**2 + l_1**2 - l_2**2) / (2.0 * l_1 * np.sqrt(x**2 + y**2)))
    beta = np.arccos((l_1**2 + l_2**2 - x**2 - y**2) / (2.0 * l_1 * l_2))

    if (current_theta_1 is None) or (current_theta_2 is None):
        theta_1 = gamma - alpha
        theta_2 = np.pi - beta
        return theta_1, theta_2

    theta_1_1 = gamma - alpha
    theta_2_1 = np.pi - beta

    theta_1_2 = gamma + alpha
    theta_2_2 = beta - np.pi

    if ((current_theta_1 - theta_1_1)**2.0 +
        (current_theta_2 - theta_2_1)**2.0)**0.5 < (
            (current_theta_1 - theta_1_2)**2.0 +
            (current_theta_2 - theta_2_2)**2.0)**0.5:
        theta_1 = theta_1_1
        theta_2 = theta_2_1
    else:
        theta_1 = theta_1_2
        theta_2 = theta_2_2

    return theta_1, theta_2


def smooth_trajectory(xi, xf, v_max, a_max, T_sampling):
    t_vmax = 1.875 * (xf - xi) / v_max
    t_amax = 2.40281141413475 * np.sqrt((xf - xi) / a_max)
    tf = max(t_vmax, t_amax)
    times = np.arange(0, tf, T_sampling)
    positions = (xf - xi) * (10.0 * (times / tf)**3 - 15.0 *
                             (times / tf)**4 + 6.0 * (times / tf)**5) + xi

    return times, positions


def convert_to_step_times(positions, T_sampling, P_sampling):
    stepper_times = []
    stepper_positions = []
    accumulated_position_stepper = 0

    count = 0
    for index in range(len(positions) - 1):
        pos_diff = positions[index + 1] - positions[index]
        while True:
            count += 1
            next_pos = accumulated_position_stepper + P_sampling * (
                1 if pos_diff > 0 else -1)
            if (pos_diff > 0 and next_pos > positions[index + 1]) or \
               (pos_diff < 0 and next_pos < positions[index + 1]):
                break

            inc_t_stepper = T_sampling * (next_pos -
                                          positions[index]) / pos_diff
            t_stepper = T_sampling * index + inc_t_stepper
            stepper_times.append(t_stepper)
            stepper_positions.append(next_pos)
            accumulated_position_stepper = next_pos
    
    print(count/float(TIME))

    return stepper_times, stepper_positions


def main():

    T_sampling = 0.01
    # v_max = 3.0
    # a_max = 5.0
    # xi = 0.0
    # xf = 2.0
    # start_time = time.time()
    # times, positions = smooth_trajectory(xi, xf, v_max, a_max, T_sampling)
    times = np.arange(0, TIME, T_sampling)
    positions = 1 * np.sin(5 * times)

    P_sampling = 0.05

    stepper_times, stepper_positions = convert_to_step_times(
        positions, T_sampling, P_sampling)
    # end_time = time.time()
    # duration = end_time - start_time

    # print(f"{duration} s")

    ### plot

    figure, axes = plt.subplots()
    axes.plot(times, positions, 'x-', label='Trajectory')
    axes.step(stepper_times,
              stepper_positions,
              'o-',
              label='Trajectory_stepper',
              where='post')
    axes.grid()
    axes.legend()
    axes.set_xlabel('Time')
    axes.set_ylabel('Position')
    plt.show()


if __name__ == "__main__":
    main()
