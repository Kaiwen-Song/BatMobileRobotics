import re
from matplotlib import pyplot as plt
import sys


def plot_graph(filename):
    f = open(filename,'r')
    fig = plt.figure()
    #left is motor1, right is motor 2
    left_motor_actual = []
    left_motor_ref = []
    right_motor_ref = []
    right_motor_actual = []
    left_motor_diff = []
    right_motor_diff = []
    time = []
    for line in f.readlines():
      reading = line.split()
      time.append(reading[0])
      left_motor_ref.append(reading[1])
      left_motor_actual.append(reading[2])
      right_motor_ref.append(reading[3])
      right_motor_actual.append(reading[4])
      left_motor_diff.append(float(reading[1]) - float(reading[2]))
      right_motor_diff.append(float(reading[3]) - float(reading[4]))
    ax = fig.add_subplot(2,2,1)
    ax.plot(time, left_motor_actual, label = "actual")
    ax.plot(time, left_motor_ref, label = "reference")
    ax.set_title("left motor ")
    ax = fig.add_subplot(2,2,2)
    #ax.plot(time,map(lambda x : -x, left_motor_diff), label = 'left motor')
    ax.plot(time,left_motor_diff, label = 'left motor')
    ax.set_title("left motor differences")
    ax = fig.add_subplot(2,2,3)
    ax.plot(time, right_motor_actual, label = "actual")
    ax.plot(time, right_motor_ref, label = "reference")
    ax.set_title("right motor")
    ax = fig.add_subplot(2,2,4)
    #ax.plot(time,map(lambda x : -x, right_motor_diff), label = 'right motor')
    ax.plot(time,right_motor_diff, label = 'right motor')
    ax.set_title("right motor differences")
    for ax in fig.axes:
        ax.set_xlabel("time")
        ax.set_ylabel("angle")
        ax.legend()
        ax.grid(True)
    plt.show()

path = sys.argv[1]
plot_graph(path)
