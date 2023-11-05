import numpy as np
import matplotlib.pyplot as plt

period_T = 0.01

position = 6
max_velocity = 2
max_acceleration = 1

velocity = min(np.sqrt(max_acceleration * position), max_velocity)

t_1 = velocity / max_acceleration
t_2 = position / velocity
t_3 = t_1 + t_2

print(t_1)
print(t_2)
print(t_3)

p_1 = (max_acceleration / 2.0) * t_1**2
p_2 = velocity * (t_2 - t_1) + (max_acceleration / 2.0) * t_1**2

time = np.arange(0, t_3+period_T, period_T)

p = []
v = []
for t in time:
    if t <= t_1:
        x = (max_acceleration / 2.0) * t**2
        p.append(x)
        v.append(np.sqrt(max_acceleration*x*2.0))
    elif t_1 < t <= t_2:
        p.append(velocity * (t - t_1) + (max_acceleration / 2.0) * t_1**2)
        v.append(velocity)
    elif t_2 < t:
        x = velocity*(t-t_1)+(max_acceleration/2.0)*t_1**2-(max_acceleration/2.0)*(t-t_2)**2
        p.append(x)
        v.append(np.sqrt(velocity**2 - 2.0 * max_acceleration * (x - p_2)))
    # else:
    #     raise ValueError

print(p[-1])
print(v[-1])

plt.plot(time, p)
plt.plot(time, v)
plt.grid()
plt.show()