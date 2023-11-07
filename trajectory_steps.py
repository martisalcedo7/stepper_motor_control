import numpy as np
import matplotlib.pyplot as plt
import math

steps_per_rev = 4076

period_T = 0.01

position = 45
max_velocity = 30
max_acceleration = 25

velocity = min(np.sqrt(max_acceleration * position), max_velocity)

t_1 = velocity / max_acceleration
t_2 = position / velocity

p_1 = (max_acceleration / 2.0) * t_1**2
p_2 = velocity * (t_2 - t_1) + (max_acceleration / 2.0) * t_1**2

steps = math.floor((position * steps_per_rev) / 360.0)

print(steps)

print(steps * 360.0 / steps_per_rev)

positions_array = np.arange(360.0/steps_per_rev, position, 360.0/steps_per_rev)
print(positions_array)
print(len(positions_array))

v = []

for p in positions_array:
    if p <= p_1:
        v.append(np.sqrt(max_acceleration*p*2.0))
    elif p_1 < p <= p_2:
        v.append(velocity)
    else:
        # v.append(np.sqrt(velocity**2 - 2.0 * max_acceleration * (p - p_2)))
        v.append(np.sqrt(2.0 * max_acceleration * (position-p)))

print(v)
print(len(v))

plt.plot(v, '-x')
plt.grid()
plt.show()