#!/usr/bin/python
# -*- coding: utf-8 -*-
from builtins import list

import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import math


def read_swath_record(line):

    [header, pos_x, pos_y, swath_port_x, swath_port_y, swath_stbd_x, swath_stbd_y] = line.strip().split(",")
    return [float(pos_x), float(pos_y), float(swath_port_x), float(swath_port_y), float(swath_stbd_x), float(swath_stbd_y)]

def read_path(line):
    [header, x, y] = line.strip().split(",")
    return [float(x), float(y)]

def shortest_angle(x, y):
    return np.arctan2(np.sin(x - y), np.cos(x - y))


def read_file(path):
    positions = []
    swaths = []
    next_line = []

    with open(path) as fp:
        line = fp.readline()
        while line:
            if line.startswith("SWATH"):
                [pos_x, pos_y, swath_port_x, swath_port_y, swath_stbd_x, swath_stbd_y] = read_swath_record(line)
                positions.append(np.array([pos_x, pos_y]))
                swaths.append(np.array([swath_port_x, swath_port_y, swath_stbd_x, swath_stbd_y]))

            if line.startswith("NEXT_PATH"):
                [x, y] = read_path(line)
                next_line.append(np.array([x, y]))

            line = fp.readline()

        positions = np.asarray(positions)
        swaths = np.asarray(swaths)
        next_line = np.asarray(next_line)

        return [positions, swaths, next_line]


def plot_from_dbg_file(log_file_path):
    [POSITIONS, SWATHS, NEXT_PATH] = read_file(log_file_path)
    print("File reading over \nPositions: " + str(len(POSITIONS)) + "\nSwaths: "
          + str(len(SWATHS)) + "\nNext line points: " + str(len(NEXT_PATH)))

    p_x = POSITIONS[:, 0]
    p_y = POSITIONS[:, 1]
    s_port_x = SWATHS[:, 0]
    s_port_y = SWATHS[:, 1]
    s_stbd_x = SWATHS[:, 2]
    s_stbd_y = SWATHS[:, 3]
    path_x = NEXT_PATH[:, 0]
    path_y = NEXT_PATH[:, 1]

    fig, axVector = plt.subplots()
    q = axVector.quiver(p_x, p_y, 0, 0, units='xy', scale=1,
                        color='black', label="Position")
    p = axVector.quiver(s_port_x, s_port_y, 0, 0, units='xy', scale=1,
                        color='red', alpha=0.6, label="Swath Port")
    g = axVector.quiver(s_stbd_x, s_stbd_y, 0, 0, units='xy', scale=1,
                        color='green', alpha=0.6, label="Swath Starboard")
    n = axVector.quiver(path_x, path_y, 0, 0, units='xy', scale=1,
                        color='blue', label="Next Path")

    plt.grid()
    plt.plot(path_x, path_y)
    axVector.set_aspect('equal')
    axVector.grid()
    axVector.legend()
    axVector.set_title('Trajectory tracking', fontsize=10)

    plt.show()
    # plt.close()

plot_from_dbg_file('PathPlannerOutput/CanGenerateSimplePlan.dbg')
plot_from_dbg_file('PathPlannerOutput/CanGenerateNoisyPlan.dbg')

