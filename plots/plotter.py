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

with open('log_twiddle_3.csv') as csvFile:
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
    

# For position loop

figure1, fig1_axes = plt.subplots(2,1, figsize = (9,9))
figure1.suptitle("Setpoint (SP), Process value (PV) and Control value (CV)\n")
fig1_axes[0].plot(time_s, SP)
fig1_axes[0].plot(time_s, PV)
fig1_axes[0].set_title("SP & PV", fontsize =10)
fig1_axes[0].set_xlabel("Time (ms)")
fig1_axes[0].set_ylabel("Position on lane (m)")
fig1_axes[1].plot(time_s, CV)
fig1_axes[1].set_title("CV", fontsize = 10)
fig1_axes[1].set_xlabel("Time (ms)")
fig1_axes[1].set_ylabel("Steering angle (rad)")
figure1.savefig('ImgsReport/twiddle_3_SP_PV_CV.png')


figure2, fig2_axes = plt.subplots(3,2, figsize = (9,9))
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
figure2.savefig('ImgsReport/twiddle_3_Param.png')

figure3, fig3_axes = plt.subplots(1,1)
figure3.suptitle("Average Quadratic Error")
fig3_axes.plot(time_s[3000:], best_error[3000:])
figure3.savefig('ImgsReport/twiddle_3_Error.png')



# For speed loop
"""
for i in range(3500,len(time_s)): 
    time_s[i] -= time_s[3499]

figure1, fig1_axes = plt.subplots(2,1, figsize = (9,9))
figure1.suptitle("Setpoint (SP), Process value (PV) and Control value (CV)\n")
fig1_axes[0].plot(time_s[3500:], SP[3500:])
fig1_axes[0].plot(time_s[3500:], PV[3500:])
fig1_axes[0].set_title("SP & PV", fontsize =10)
fig1_axes[0].set_xlabel("Time (ms)")
fig1_axes[0].set_ylabel("Speed of car (MPH)")
fig1_axes[1].plot(time_s[3500:], CV[3500:])
fig1_axes[1].set_title("CV", fontsize = 10)
fig1_axes[1].set_xlabel("Time (ms)")
fig1_axes[1].set_ylabel("Throttle")
figure1.savefig('ImgsReport/_twiddle_speed_1_SP_PV_CV.png')


figure2, fig2_axes = plt.subplots(3,2, figsize = (9,9))
figure2.suptitle("Controller parameters") 
fig2_axes[0,0].plot(time_s[3500:], Kp[3500:])
fig2_axes[0,0].set_title("Kp")
fig2_axes[0,1].plot(time_s[3500:], dKp[3500:])
fig2_axes[0,1].set_title("delta Kp")
fig2_axes[1,0].plot(time_s[3500:], Ki[3500:])
fig2_axes[1,0].set_title("Ki")
fig2_axes[1,1].plot(time_s[3500:], dKi[3500:])
fig2_axes[1,1].set_title("delta Ki")
fig2_axes[2,0].plot(time_s[3500:], Kd[3500:])
fig2_axes[2,0].set_title("Kd")
fig2_axes[2,1].plot(time_s[3500:], dKd[3500:])
fig2_axes[2,1].set_title("delta Kd")
figure2.savefig('ImgsReport/_twiddle_speed_1_Param.png')

figure3, fig3_axes = plt.subplots(1,1)
figure3.suptitle("Average Quadratic Error")
fig3_axes.plot(time_s[3500:], best_error[3500:])
figure3.savefig('ImgsReport/_twiddle_speed_1_Error.png')
"""

