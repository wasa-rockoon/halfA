import os
import sys
import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation


def main():

    if len(sys.argv) < 2:
        print('Specify port')
        return

    port = sys.argv[1]

    s = serial.Serial(port, 9600)

    fig = plt.figure()
    ax = fig.add_subplot(2, 1, 1, projection='3d')
    bx = fig.add_subplot(2, 1, 2)

    ax.set_title('Attitude and Acceleration')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)
    ax.set_xlabel('x [G]')
    ax.set_ylabel('y [G]')
    ax.set_zlabel('z [G]')

    bx.set_title('Altitude')
    bx.set_xlabel('t [s]')
    bx.set_ylabel('h [m]')


    accl_quiver = ax.quiver(0, 0, 0, 0, 0, 1)
    attX_quiver = ax.quiver(0, 0, 0, 1, 0, 0)
    attY_quiver = ax.quiver(0, 0, 0, 0, 1, 0)
    attZ_quiver = ax.quiver(0, 0, 0, 0, 0, 1)

    # X_quiver = ax.quiver(0,0,0,1,0,0,color='red')
    # Y_quiver = ax.quiver(0,0,0,0,1,0,color='green')
    # Z_quiver = ax.quiver(0,0,0,0,0,1,color='blue')

    alt_line, = bx.plot([], [])


    seconds = 0;
    freq = 0;
    accl = [0, 0, -9.8]
    quat = [1, 0, 0, 0]
    alt = 0
    init_alt = None
    bat = 0

    alt_max = 0
    accl_max = 0

    ts = []
    hs = []

    def update(theta):

        nonlocal seconds
        nonlocal freq
        nonlocal accl
        nonlocal quat
        nonlocal alt
        nonlocal init_alt
        nonlocal bat
        nonlocal ts
        nonlocal hs
        nonlocal alt_max
        nonlocal accl_max

        os.system('clear')

        while s.in_waiting > 0:

            data = s.readline()
            print(data.decode().replace('\n', ' ').replace('\0', ''), end="")

            if len(data) <= 1:
                continue

            label = chr(data[0])
            value = None

            # print(label, data[1:].decode().replace('\0', '')

            try:
                value = float(data[1:].decode().replace('\0', ''))
            except:
                continue

            # print(label, value)
            if label == 'T':
                ps = seconds
                seconds = value / 1000.0
                freq = 1.0 / (seconds - ps)
                hs.append(alt)
                ts.append(seconds)
                # bx.scatter(seconds, alt)
            elif label == 'X':
                accl[0] = value
            elif label == 'Y':
                accl[1] = value
            elif label == 'Z':
                accl[2] = value

            elif label == 'p':
                quat[3] = value
            elif label == 'q':
                quat[0] = value
            elif label == 'r':
                quat[1] = value
            elif label == 's':
                quat[2] = value

            elif label == 'A':
                if init_alt is None:
                    init_alt = value
                alt = value - init_alt
                if alt > alt_max:
                    alt_max = alt

            elif label == 'V':
                bat = value

            

        rot = Rotation.from_quat(quat)
        axisX = rot.apply([-1, 0, 0])
        axisY = rot.apply([0, 1, 0])
        axisZ = rot.apply([0, 0, 1])
        accl_ = rot.apply(accl) / 9.80665
        # accl_ = accl

        print(';')

        rot_euler = rot.as_euler('yxz', degrees=True)
        accl_abs = np.linalg.norm(accl)
        if accl_abs > accl_max:
            accl_max = accl_abs

        print('{:>5.2f} updates per second'.format(freq))

        print('T:        {:>+8.3f} [s]'.format(seconds))
        print('Acceleration:')
        print('  abs:    {:>+8.3f} [m/s2] (Max:{:>8.3f} [m/s2])'.format(accl_abs, accl_max))
        print('  x:      {:>+8.3f} [m/s2]'.format(accl[0]))
        print('  y:      {:>+8.3f} [m/s2]'.format(accl[1]))
        print('  z:      {:>+8.3f} [m/s2]'.format(accl[2]))
        print('Attitude:    ', quat),
        print('  Pitch:  {:>+8.1f}°'.format(rot_euler[0]))
        print('  Roll:   {:>+8.1f}°'.format(rot_euler[1]))
        print('  Yaw:    {:>+8.1f}°'.format(rot_euler[2]))
        print('Altitude: {:>+8.3f} [m] (Max:{:>+8.3f} [m])'.format(alt, alt_max))
        print('Battery:  {:>8.3f} [V]'.format(bat))


        nonlocal attX_quiver
        nonlocal attY_quiver
        nonlocal attZ_quiver
        attX_quiver.remove()
        attY_quiver.remove()
        attZ_quiver.remove()
        attX_quiver = ax.quiver(0,0,0,axisX[0],axisX[1],axisX[2],color='red')
        attY_quiver = ax.quiver(0,0,0,axisY[0],axisY[1],axisY[2],color='green')
        attZ_quiver = ax.quiver(0,0,0,axisZ[0],axisZ[1],axisZ[2],color='blue')

        nonlocal accl_quiver
        accl_quiver.remove()
        accl_quiver = ax.quiver(0,0,0,accl_[0],accl_[1],-accl_[2],color='black')


        nonlocal alt_line
        bx.lines.remove(alt_line)
        alt_line, = bx.plot(ts, hs, c="black")

        return alt_line,

    ani = FuncAnimation(fig, update, interval=20)
    # plt.ion()
    plt.show()

    s.close()


if __name__ == "__main__":
    main()
