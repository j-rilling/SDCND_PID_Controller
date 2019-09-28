#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 27 23:03:43 2019

@author: earendilavari
"""

import csv
import matplotlib.pyplot as plt

time_ms = []
time_s = []
SP = []
PV = []
CV = []
best_error = []
Kp = []
Ki = []
Kd = []
dKp = []
dKi = []
dKd = []

with open('log_step1.csv') as csvFile:
    readCSV = csv.reader(csvFile, delimiter = ' ')
    for row in readCSV:
        time_ms.append(float(row[0]))
        SP.append(float(row[1]))
        PV.append(float(row[2]))
        CV.append(float(row[3]))
        best_error.append(float(row[4]))
        Kp.append(float(row[5]))
        Ki.append(float(row[6]))
        Kd.append(float(row[7]))
        dKp.append(float(row[8]))
        dKi.append(float(row[9]))
        dKd.append(float(row[10]))
        
for i in range(len(time_ms)):
    time_s.append(time_ms[i]/1000.0)

figure1, fig1_axes = plt.subplots(2,1, figsize = (10,8))
figure1.suptitle("Setpoint (SP), Process value (PV) and Control value (CV)\n Kp = 0.03, Ki = 0.0, Kd = 0.0")
fig1_axes[0].plot(time_s, SP)
fig1_axes[0].plot(time_s, PV)
fig1_axes[0].set_title("SP & PV", fontsize =10)
fig1_axes[0].set_xlabel("Time (ms)")
fig1_axes[0].set_ylabel("Position on lane (m)")
fig1_axes[1].plot(time_s, CV)
fig1_axes[1].set_title("CV", fontsize = 10)
fig1_axes[1].set_xlabel("Time (ms)")
fig1_axes[1].set_ylabel("Angle (rad)")
figure1.savefig('ImgsReport/step1_.png')

"""
figure2, fig2_axes = plt.subplots(3,2)
figure2.suptitle("Controller parameters") 
fig2_axes[0,0].plot(time_s, Kp)
fig2_axes[0,0].set_title("Kp")
fig2_axes[0,1].plot(time_s, dKp)
fig2_axes[0,1].set_title("delta Kp")
fig2_axes[1,0].plot(time_s, Ki)
fig2_axes[1,0].set_title("Ki")
fig2_axes[1,1].plot(time_s, dKi)
fig2_axes[1,1].set_title("delta Ki")
fig2_axes[2,0].plot(time_s, Kd)
fig2_axes[2,0].set_title("Kd")
fig2_axes[2,1].plot(time_s, dKd)
fig2_axes[2,1].set_title("delta Kd")

figure3, fig3_axes = plt.subplots(1,1)
fig3_axes.plot(time_s, best_error)

"""


