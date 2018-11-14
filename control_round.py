#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 4 11:27:10 2018
@author: Potex (github.com/Potex)
Python version: 3.4.2
Control script for Szenergy-model.
"""

# Raspberry Pi script which controls the motor and servo
# also creats log files to save sensor data

import serial
import time
import threading
import csv

t = time.localtime()
timestamp = time.strftime('%Y%m%d%H%M%S', t)
ser=serial.Serial("/dev/ttyACM0", 115200)
ser2=serial.Serial("/dev/ttyUSB0", 115200)
time.sleep(1)

sens_reading = True

log = []

with open('log_sensor'+timestamp+'.csv', 'w') as csvfile:
    Sens_log = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    Sens_log.writerow(['Sensor values form Encoder, Servo and ASSN'])
print('Sensor CSV Done. Start logging')

def sensor_read():
    while sens_reading:
        log1 = ser2.readline()
        data = str(log1)
        data2 = data[2:-5]
        with open('log_sensor'+timestamp+'.csv', 'a') as csvfile:
            Sens_log = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            Sens_log.writerow(data2)

s = threading.Thread(target=sensor_read)
s.start()
time.sleep(4)

nullStr  =  "1.0,1.0a"
forwStr  =  "0.0,0.95a"
rightStr =  "0.0,2.0a"
right2Str =  "0.0,1.88a"
leftStr =  "0.2,0.0a"
forw2Str = "0.0,0.93a"

with open('log_jarmumodell'+timestamp+'.csv', 'w') as csvfile:
    Modell_log = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    Modell_log.writerow(['Systime between performed tasks'])
print('Modell CSV Done. Start logging')

##3219.429703301   Time before serial_write
##3219.430038246   Time after serial_write, before sleep(1)
##  serial_write = 0.334945 ms
##3220.431092828   Time after sleep(1), before serial_write
##  sleep(1)     = 1001.054582 ms
##3220.431360638   Time after serial_write, before sleep(2)
##  serial_write = 0.267810 ms
##3222.433491833   Time after sleep(2), before serial_write
##  sleep(2)     = 2002.125453 ms
##3222.433730789   Time after serial_write, before sleep(11)
##  serial_write = 0.238956 ms
##3233.439386459   Time after sleep(11)
##  sleep(11)    = 11005.65567 ms

##-------------1. Lap ---------------
#log.append(time.perf_counter())
ser.write(nullStr.encode('ascii'))
#log.append(time.perf_counter())
time.sleep(1.5)
#log.append(time.perf_counter())
ser.write(forw2Str.encode('ascii'))
#log.append(time.perf_counter())
time.sleep(3.5)
#log.append(time.perf_counter())
ser.write(rightStr.encode('ascii'))
#log.append(time.perf_counter())
time.sleep(10.2)
#log.append(time.perf_counter())
ser.write(forwStr.encode('ascii'))
#log.append(time.perf_counter())
time.sleep(5.6)
#log.append(time.perf_counter())
ser.write(right2Str.encode('ascii'))
#log.append(time.perf_counter())
time.sleep(10.2)
#log.append(time.perf_counter())
ser.write(forwStr.encode('ascii'))
#log.append(time.perf_counter())
time.sleep(1.65)
#log.append(time.perf_counter())
ser.write(nullStr.encode('ascii'))
#log.append(time.perf_counter())
time.sleep(2)
#log.append(time.perf_counter())
##-------------1. Lap ----END--------
##-------------2. Lap ---------------
'''ser.write(forw2Str.encode('ascii'))
time.sleep(3)
ser.write(rightStr.encode('ascii'))
time.sleep(10.2)
ser.write(forw2Str.encode('ascii'))
time.sleep(5)
ser.write(right2Str.encode('ascii'))
time.sleep(9.4)
ser.write(forw2Str.encode('ascii'))
time.sleep(1.05)
ser.write(nullStr.encode('ascii'))
time.sleep(1)'''
##-------------2. Lap ----END--------
sens_reading = False
time.sleep(1)
##---------Write to log file---------
with open('log_jarmumodell'+timestamp+'.csv', 'a') as csvfile:
    Modell_log = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    Modell_log.writerow(log)
print('Log written.')
##------Write to log file---END------

print('Sensros logging thread stopped.')
ser.close()
ser2.close()
print("Test round comlpeted. Serial connection closed, motor stopped, steering set to middle position. ")
