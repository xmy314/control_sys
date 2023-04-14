
import os
os.remove("main.py")

To get the output out of the pico.
First get port by:
ls /dev/cu.usbmodem*

Then enter a pythonenv with rshell and run commands like the following:
rshell -p /dev/cu.usbmodem101 cp /pico/data.csv /Users/kaishang/Code/python_dev/course/control/data.csv
rshell -p /dev/cu.usbmodem101 cp /pico/error_dump.txt /Users/kaishang/Code/python_dev/course/control/error_dump.txt
rshell -p /dev/cu.usbmodem101 cp /pico/main.py /Users/kaishang/Code/python_dev/course/control/main.py
rshell -p /dev/cu.usbmodem101 rm /pico/data.csv 
rshell -p /dev/cu.usbmodem101 rm /pico/main.py
rshell -p /dev/cu.usbmodem101 cp /Users/kaishang/Code/python_dev/course/control/main.py /pico/main.py
rshell -p /dev/cu.usbmodem101 cp /Users/kaishang/Code/python_dev/course/control/stationary.py /pico/main.py
rshell -p /dev/cu.usbmodem101 ls /pico

this is the control system i used for the mech 223 titan endurance project. it automatically aims at the well and only shoot when it knows that it will get inside.

I've accounted for the following
- low resolution of rotary encoder
- even lower resolition of accelerometer
- current position
- current velocity
- acceleration from spring and deceleration from the friciton on the orbiter
- change in the velocity of lander as the fricition makes the anugular velocity match the velocity such that there is no slipping
- the distance travelled due to the difference in velocity in transition period.
- latency in the lander launcher
- the uncertainty in all of the parameter above
- maximum turning speed of the servo before messy jittering.
- stationary launch mode
- approach direction modes
- data collection mode

the over all steps of the system are like the following:
1. collect sensor data
2. pass sensor data to kalman filter
3. check what state it is in and act accordingly. since the most complicated part is when the car is running, so the following steps would be from that branch.
4. check the trajectory of the ball if it is launched at that moment, if it goes into the hole, launch. else, goto step 5.
5. move the servos such that it is closer to the ideal entry of 0.8/s straight through the middle. (in the middle as the variance in other components doesn't allow aimming towards the edge.)
6. repeat

visual demonstration and the experimental tool can be found at:
https://www.desmos.com/calculator/omqmqzw1n7

data analyzation for the data collected by the pico can be found at:
https://docs.google.com/spreadsheets/d/1cDU1Y42evv7ENGjs-aQJDusUGYOLv96Po3U83Up-XPk/edit#gid=31757851
