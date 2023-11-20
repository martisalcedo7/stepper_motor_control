import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d


def accumulate_positions(positions):
    # accumulated_positions = [0.0]
    # current_sum = 0.0

    # for idx, pos in enumerate(positions[1:]):
    #     current_sum += abs(positions[idx+1] - positions[idx])
    #     accumulated_positions.append(current_sum)

    accumulated_positions = np.cumsum(np.abs(np.diff(positions)))
    accumulated_positions = np.insert(accumulated_positions, 0, 0.0)
    print('Accu: ', accumulated_positions)
    return accumulated_positions


def convert_to_step_times(positions, T_sampling, P_sampling):

    acc_positions = accumulate_positions(positions)
    print('Accu: ', acc_positions)
    number_of_steps = int(np.floor(abs(acc_positions[-1] / P_sampling)))
    print('Num steps: ', number_of_steps)

    steps = np.arange(0, acc_positions[-1], P_sampling)
    print('Steps: ', steps)
    
    new_times = [0]
    new_steps = [0]
    accumulated_step = 0
    for idx in range(number_of_steps):
        x = 0
        while True:
            if x+1 >= len(acc_positions):
                break
            if acc_positions[x+1] >= ((idx+1) * P_sampling):
                break
            x += 1
        
        new_time = T_sampling * (((idx+1) * P_sampling) - acc_positions[x])/(acc_positions[x+1]-acc_positions[x])

        new_times.append(new_time + T_sampling * x)
        accumulated_step = (P_sampling * np.sign(positions[x+1]-positions[x])) + accumulated_step
        new_steps.append(accumulated_step)

    return steps, new_times, new_steps


def main():

    T_sampling = 0.01
    time = np.arange(0, 1, T_sampling)
    positions = 2.15 * np.sin(8.0 * time)

    P_sampling = 0.1
    steps, new_times, new_steps = convert_to_step_times(positions, T_sampling, P_sampling)
    print(new_times)
    ### plot

    figure, axes = plt.subplots()
    axes.plot(time, positions, label='Trajectory')
    axes.plot(time, accumulate_positions(positions), 'x-')
    for step in steps:
        axes.axhline(y=step, color='r', linestyle='--')
    for new_time in new_times:
        axes.axvline(x=new_time, color='b', linestyle='--')

    axes.step(new_times,
              steps,
              'o-',
              label='Trajectory_stepper', where='post')
    axes.step(new_times,
              new_steps,
              'o-',
              label='Trajectory_stepper', where='post')
    axes.grid()
    axes.legend()
    axes.set_xlabel('Time')
    axes.set_ylabel('Position')
    plt.show()


if __name__ == "__main__":
    main()
