import numpy as np
import matplotlib.pyplot as plt

def test(final_position):
    # Define initial and final positions
    initial_position = 0

    # Define initial and final velocities
    initial_velocity = 0
    final_velocity = 0

    # Define the time duration for the trajectory
    time_duration = 40  # You can adjust this based on your requirements

    # Calculate the coefficients for the cubic polynomial
    a = initial_position
    b = initial_velocity
    c = (3 * (final_position - initial_position) / (time_duration ** 2)) - (2 * initial_velocity / time_duration) - (final_velocity / time_duration)
    d = (-2 * (final_position - initial_position) / (time_duration ** 3)) + (initial_velocity / (time_duration ** 2)) + (final_velocity / (time_duration ** 2))

    # Define the number of steps
    num_steps = 400  # You can adjust this based on how many data points you need

    # Calculate position and velocity at each step
    positions = []
    velocities = []

    for t in np.linspace(0, time_duration, num_steps):
        position = a + b * t + c * (t ** 2) + d * (t ** 3)
        velocity = b + 2 * c * t + 3 * d * (t ** 2)
        
        positions.append(position)
        velocities.append(velocity)

    max_velocity = np.max(np.abs(velocities))
    velocities = 140.0 * (np.abs(velocities) / max_velocity)

    return np.array(positions), np.array(velocities)


if __name__ == "__main__":
    positions, velocities = test(-450)

    plt.plot(np.diff(positions))
    plt.grid()
    plt.show()

    plt.plot(velocities[1:])
    plt.grid()
    plt.show()