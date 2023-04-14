import machine  # type: ignore
import utime  # type: ignore
import ustruct  # type: ignore
from ulab import numpy as np  # type: ignore
from math import pi
from servo import Servo

# this program is to find the velocity that the ball leaves the ramp at and the corresponding latency

a=3*pi/4
test_servo=Servo(3)
beam_distance=0.1

tt=0
st=0
et=0

def break_beam_a_callback(pin):
    global st
    st=utime.ticks_ms()


def break_beam_b_callback(pin):
    global st,et
    et=utime.ticks_ms()

beam_a_pin = machine.Pin(16, mode=machine.Pin.IN, pull=machine.Pin.PULL_UP)
beam_a_pin.irq(trigger=machine.Pin.IRQ_FALLING, handler=break_beam_a_callback, hard=True)
beam_b_pin = machine.Pin(17, mode=machine.Pin.IN, pull=machine.Pin.PULL_UP)
beam_b_pin.irq(trigger=machine.Pin.IRQ_FALLING, handler=break_beam_b_callback, hard=True)

test_servo.goto(a)
utime.sleep(0.5)
test_servo.goto(0)
tt=utime.ticks_ms()

utime.sleep(1)
test_servo.goto(a)
utime.sleep(0.5)

print(f"ramp latency: {utime.ticks_diff(st,tt)}")
print(f"velocity: {1000*beam_distance/utime.ticks_diff(et,st)}")
