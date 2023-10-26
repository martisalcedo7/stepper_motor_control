import serial
import numpy as np
import trajectory
import time

with serial.Serial() as ser:
    ser.baudrate = 115200
    ser.port = '/dev/ttyACM0'
    ser.open()

    angles = [900, -900, 500, -500, 500, -500, 500, -500]
    velocities = [100]*len(angles)

    # angles, velocities = trajectory.test(900)

    # angles = np.diff(angles)
    # velocities = velocities[1:]

    for angle, velocity in zip(angles, velocities):

        # angle = int(angle)
        # velocity = int(velocity)
        # print(velocity)

        # Convert the integer to bytes (2 bytes in this example, using little-endian byte order)
        direction = 0
        if angle < 0:
            direction = 1
        data = abs(angle) | (velocity << 16) | (direction << 32)
        data = data | (data << 40)
        data_bytes = data.to_bytes(10, byteorder='little')
        # print(data_bytes)
        # Send the bytes over the serial port
        ser.write(data_bytes)
        time.sleep(0.01)
