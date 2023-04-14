# Introduction
This is the control system i used for the mech 223 titan endurance project. it automatically aims at the well and only shoot when it knows that it will get inside.


# Details
## Accounted variable
- Low resolution of rotary encoder
- Even lower resolition of accelerometer
- Current position
- Current velocity
- Acceleration from spring and deceleration from the friciton on the orbiter
- Change in the velocity of lander as the fricition makes the anugular velocity match the velocity such that there is no slipping
- The distance travelled due to the difference in velocity in transition period.
- Latency in the lander launcher
- Uncertainty in all of the parameter above
- Maximum turning speed of the servo before jittering

## Configurable modes
- Stationary launch mode
- Approach direction modes
- Data collection

## Overview of structure
1. Collect sensor data
2. Pass sensor data to kalman filter
3. Check what state it is in and act accordingly. since the most complicated part is when the car is running, so the following steps would be from that branch.
4. Check the trajectory of the ball if it is launched at that moment, if it goes into the hole, launch. else, goto step 5.
5. Move the servos such that it is closer to the ideal entry of 0.8/s straight through the middle. (in the middle as the variance in other components doesn't allow aimming towards the edge.)
6. Repeat

# Miscellaneous
## Visual demonstration and the experimental tool can be found at:
https://www.desmos.com/calculator/omqmqzw1n7

## Data analyzation for the data collected by the pico can be found at:
https://docs.google.com/spreadsheets/d/1cDU1Y42evv7ENGjs-aQJDusUGYOLv96Po3U83Up-XPk/edit#gid=31757851


---

Useful cmd snippets for development.

To delete main from terminal.
```
import os
os.remove("main.py")
```

To get the port of the pico.
```
ls /dev/cu.usbmodem*
```

To enter a pythonenv with rshell and run commands like the following:
```
rshell -p /dev/cu.usbmodem101 cp /pico/data.csv /Users/kaishang/Code/python_dev/course/control/data.csv
rshell -p /dev/cu.usbmodem101 cp /pico/error_dump.txt /Users/kaishang/Code/python_dev/course/control/error_dump.txt
rshell -p /dev/cu.usbmodem101 cp /pico/main.py /Users/kaishang/Code/python_dev/course/control/main.py
rshell -p /dev/cu.usbmodem101 rm /pico/data.csv 
rshell -p /dev/cu.usbmodem101 rm /pico/main.py
rshell -p /dev/cu.usbmodem101 cp /Users/kaishang/Code/python_dev/course/control/main.py /pico/main.py
rshell -p /dev/cu.usbmodem101 cp /Users/kaishang/Code/python_dev/course/control/stationary.py /pico/main.py
rshell -p /dev/cu.usbmodem101 ls /pico
```