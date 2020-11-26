# -*- coding: utf-8 -*-
from __future__ import print_function
# A test for the CI/CD to ensure decoder is operating correctly
file_path = './example/171019_031603.csv'
num_lines = sum(1 for line in open(file_path))
with open(file_path) as f:
    # check header
    print("Header check...", end="\r")
    header = f.readline()
    correct_header = " Time(), Date, Local Time, Latitude(deg), Longitude(deg), Altitude(m), Distance horizontal(m), Velocity north(m/s), Velocity east(m/s), Velocity down(m/s), Velocity up(m/s), Velocity forward(m/s), Velocity lateral(m/s), ISO e.f.s. east velocity(m/s), ISO e.f.s. north velocity(m/s), ISO e.f.s. vertical velocity(m/s), ISO i.s. longitudinal velocity(m/s), ISO i.s. lateral velocity(m/s), ISO i.s. vertical velocity(m/s), ISO v.s. longitudinal velocity(m/s), ISO v.s. lateral velocity(m/s), ISO v.s. vertical velocity(m/s), Speed horizontal(m/s), Acceleration Xv(m/s²), Acceleration Yv(m/s²), Acceleration Zv(m/s²), Acceleration forward(m/s²), Acceleration lateral(m/s²), Acceleration down(m/s²), ISO e.f.s. east acceleration(m/s²), ISO e.f.s. north acceleration(m/s²), ISO e.f.s. vertical acceleration(m/s²), ISO i.s. longitudinal acceleration(m/s²), ISO i.s. lateral acceleration(m/s²), ISO i.s. vertical acceleration(m/s²), ISO v.s. longitudinal acceleration(m/s²), ISO v.s. lateral acceleration(m/s²), ISO v.s. vertical acceleration(m/s²), Heading(deg), Pitch(deg), Roll(deg), ISO yaw angle(deg), ISO pitch angle(deg), ISO roll angle(deg), Angular rate Xv(deg/s), Angular rate Yv(deg/s), Angular rate Zv(deg/s), Angular rate forward(deg/s), Angular rate lateral(deg/s), Angular rate down(deg/s), ISO e.f.s. roll velocity(deg/s), ISO e.f.s. pitch velocity(deg/s), ISO e.f.s. yaw velocity(deg/s), ISO i.s. roll velocity(deg/s), ISO i.s. pitch velocity(deg/s), ISO i.s. yaw velocity(deg/s), ISO v.s. roll velocity(deg/s), ISO v.s. pitch velocity(deg/s), ISO v.s. yaw velocity(deg/s), Angular acceleration Xv(deg/s²), Angular acceleration Yv(deg/s²), Angular acceleration Zv(deg/s²), Angular acceleration forward(deg/s²), Angular acceleration lateral(deg/s²), Angular acceleration down(deg/s²), ISO e.f.s. roll acceleration(deg/s²), ISO e.f.s. pitch acceleration(deg/s²), ISO e.f.s. yaw acceleration(deg/s²), ISO i.s. roll acceleration(deg/s²), ISO i.s. pitch acceleration(deg/s²), ISO i.s. yaw acceleration(deg/s²), ISO v.s. roll acceleration(deg/s²), ISO v.s. pitch acceleration(deg/s²), ISO v.s. yaw acceleration(deg/s²),"
    for i, (j, k) in enumerate(zip(header, correct_header)):
        assert (j == k), "Expected: {}, but got: {}".format(k, j)
        print("Header check... {:.2f}%".format(100 * i / len(correct_header)), end="\r")
    print("Header check... OK                                    ")
    # check for values
    print("Row check...", end="\r")
    empty_line_len = 70
    for i, line in enumerate(f):
        print("Row check... {:.2f}%".format(100 * i / num_lines), end="\r")
        assert (len(line) > empty_line_len), "Line {} appears to be empty".format(i)
    print("Row check... OK                                    ")
    