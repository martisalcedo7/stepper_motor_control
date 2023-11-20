import numpy as np

l_1 = 1.0
l_2 = 1.0

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


def forward_kinematics(theta_1, theta_2):
    x = l_1 * np.cos(theta_1) + l_2 * np.cos(theta_1 + theta_2)
    y = l_1 * np.sin(theta_1) + l_2 * np.sin(theta_1 + theta_2)

    return x, y