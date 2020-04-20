#!/usr/bin/python
# -*- coding: utf-8 -*-
from builtins import list

import matplotlib.pyplot as plt
import numpy as np


def read_swath_record(line):

    [header, pos_x, pos_y, swath_port_x, swath_port_y, swath_stbd_x, swath_stbd_y] = line.strip().split(",")
    return [float(pos_x), float(pos_y), float(swath_port_x), float(swath_port_y), float(swath_stbd_x), float(swath_stbd_y)]

def read_path(line):
    [header, x, y] = line.strip().split(",")
    return [float(x), float(y)]

def shortest_angle(x, y):
    return np.arctan2(np.sin(x - y), np.cos(x - y))


class Logs:
    def __init__(self):
        self.positions = []
        self.swaths = []
        self.next_line = []
        self.raw_path = []
        self.no_intersect_path = []
        self.no_bends_path = []
        self.survey_region = []

    def toNpArrays(self):
        self.positions = np.asarray(self.positions)
        self.swaths = np.asarray(self.swaths)
        self.next_line = np.asarray(self.next_line)
        self.raw_path = np.asarray(self.raw_path)
        self.no_intersect_path = np.asarray(self.no_intersect_path)
        self.no_bends_path = np.asarray(self.no_bends_path)
        self.survey_region = np.asarray(self.survey_region)

    def log_point_entry(self, line, name, array):
        if line.startswith(name):
            [x, y] = read_path(line)
            array.append(np.array([x, y]))

    def read_file(self, path):

        with open(path) as fp:
            line = fp.readline()
            while line:
                if line.startswith("SWATH"):
                    [pos_x, pos_y, swath_port_x, swath_port_y, swath_stbd_x, swath_stbd_y] = read_swath_record(line)
                    self.positions.append(np.array([pos_x, pos_y]))
                    self.swaths.append(np.array([swath_port_x, swath_port_y, swath_stbd_x, swath_stbd_y]))
                else:
                    self.log_point_entry(line, "NEXT_LINE", self.next_line)
                    self.log_point_entry(line, "RAW_PATH", self.raw_path)
                    self.log_point_entry(line, "NO_INTERSECT_PATH", self.no_intersect_path)
                    self.log_point_entry(line, "NO_BENDS_PATH", self.no_bends_path)
                    self.log_point_entry(line, "SURVEY_REGION", self.survey_region)

                line = fp.readline()

            self.toNpArrays()


def plot_from_dbg_file(log_file_path):
    l = Logs()
    l.read_file(log_file_path)
    [POSITIONS, SWATHS, RAW_PATH, NO_INTERSECT, NO_BENDS, NEXT_PATH, SURVEY_OP] = \
        [l.positions, l.swaths, l.raw_path, l.no_intersect_path, l.no_bends_path, l.next_line, l.survey_region]
    print("File reading over \nPositions: " + str(len(POSITIONS)) + "\nSwaths: "
          + str(len(SWATHS)) + "\nRaw path points: " + str(len(RAW_PATH)) + "\nNext line points: " + str(len(NEXT_PATH))
          + "\nSurvey Area: " + str(len(SURVEY_OP)))

    p_x = POSITIONS[:, 0]
    p_y = POSITIONS[:, 1]
    s_port_x = SWATHS[:, 0]
    s_port_y = SWATHS[:, 1]
    s_stbd_x = SWATHS[:, 2]
    s_stbd_y = SWATHS[:, 3]
    raw_path_x = RAW_PATH[:, 0]
    raw_path_y = RAW_PATH[:, 1]
    no_intersect_x = NO_INTERSECT[:, 0]
    no_intersect_y = NO_INTERSECT[:, 1]
    survey_reg_x = SURVEY_OP[:, 0]
    survey_reg_y = SURVEY_OP[:, 1]
    path_x = NEXT_PATH[:, 0]
    path_y = NEXT_PATH[:, 1]

    fig, axVector = plt.subplots()
    axVector.quiver(p_x, p_y, 0, 0, units='xy', scale=1,
                        color='black', label="Position")
    axVector.quiver(s_port_x, s_port_y, 0, 0, units='xy', scale=1,
                        color='red', alpha=0.6, label="Swath Port")
    axVector.quiver(s_stbd_x, s_stbd_y, 0, 0, units='xy', scale=1,
                        color='green', alpha=0.6, label="Swath Starboard")
    axVector.quiver(raw_path_x, raw_path_y, 0, 0, units='xy', scale=1,
                        color='lightcoral',  alpha=0.6, label="Raw Path")
    axVector.quiver(no_intersect_x, no_intersect_y, 0, 0, units='xy', scale=1,
                        color='darkorange',  alpha=0.6, label="No Intersect Path")
    axVector.quiver(path_x, path_y, 0, 0, units='xy', scale=1,
                        color='blue', label="Next Path")

    plt.grid()
    #plt.plot(survey_reg_x, survey_reg_y, color='black', alpha=0.6)
    plt.plot(raw_path_x, raw_path_y, color='coral', alpha=0.6)
    plt.plot(no_intersect_x, no_intersect_y, color='darkorange', alpha=0.6)
    plt.plot(path_x, path_y)
    plt.plot(no_intersect_x, no_intersect_y, color='darkorange', alpha=0.6)

    axVector.set_aspect('equal')
    axVector.grid()
    axVector.legend()
    axVector.set_title('Trajectory tracking', fontsize=10)

    plt.show()
    # plt.close()

plot_from_dbg_file('PathPlannerOutput/CanGenerateSimplePlan.dbg')
plot_from_dbg_file('PathPlannerOutput/CanGenerateNoisyPlan.dbg')

