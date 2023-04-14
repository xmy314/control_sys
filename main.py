import machine  # type: ignore
import utime  # type: ignore
import ustruct  # type: ignore
from ulab import numpy as np  # type: ignore
from math import pi, cos, sin, atan2, floor, atan
from servo import Servo
from math import copysign
import sys

# line count distribution give or take:
# 100 constant setup
# 100 sensors
# 50 kalman
# 100 pin setup
# 200 logic


print("runnning main.py")

###############################################################################
# Constants
# - Numbers that are derivative of something else or numbers that are not going to change for all variation in test condition.
# - Idealy, nothing in this section needs to be changed ever again.

# Input Setting
# Condition Constants
EARTH_P = [0, 0]  # !important
HOLE_P = np.array([2.7, 1])  # !important
IS_DEFAULT_DIRECTION = True  # !important if the chimney points to the internal (0,0), True
IS_STATIONARY = False  # !important if the chimney points to the internal (0,0), True

compression_length = 0.045

# Change with ball
angle_lookup = [2*pi/3, 3*pi/4]  # preset by geometry
speed_lookup = [0.7, 1.2]  # !important even though the specifics is impossible to control, the range is important to tell the aiming point
ball_coefficient_friciton = 0.4  # quite literally doesn't matter for the steps that i accounted for
inertia_feel = 8000  # !important only useful to change latency
latency = np.interp(inertia_feel, [1000, 3000, 8000], [0.25, 0.35, 0.45])[0]  # very rough estimation, wood, nylon, steel

# Change when model changes
k = 1000  # after the spring got weaker, this is reasonable
m = 1  # this is the approximiate mass of the car

axle_diameter = 0.01  # this is supposed to be 0.008, but since the string is twirlled on top of it seld, this value is slightly increase to compensate for it
wheel_diameter = 0.121  # this is printed diameter plus that of the rubber band
axle_to_encoder_ratio = 10  # this is the gear ratio next to the rotary encoder

USE_ACCELERAMETER = True  # even though this is on, it is quite literally turned off due to how little trust i give it.
USE_ROTARYENCODER = True  # mvp sensor
RECORD_DATA = False

# Physics Constants
G_ACCELERATION = 9.80665  # won't change unless i actually go to space

# Accelerometer Complement
#  - I2C address (Copied Code)
ADXL343_ADDR = 0x53

#  - Registers (Copied Code)
REG_DEVID = 0x00
REG_POWER_CTL = 0x2D
REG_DATAX0 = 0x32
DEVID = 0xE5

ACC_PER_TICK = G_ACCELERATION/256 * (1 if IS_DEFAULT_DIRECTION else -1)
A_VAR = (40)**2  # this is due to the noisy data of accelerometer.

# Rotary Encoder Complement
COUNTER_TRANSITION = [
    0, 1, -1, 0,
    -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0,
]
DIS_PER_TICK = (pi*wheel_diameter)*axle_to_encoder_ratio/96 * (1 if IS_DEFAULT_DIRECTION else -1)
D_VAR = (DIS_PER_TICK*2)**2  # slightly less trust here to make curves more smooth

# Kalman Filter Complement
Q = np.array([[5.8e-13, 1.175e-9, 1.175e-6], [1.17e-9, 2.35e-6, 0.00235], [1.175e-6, 0.00235, 2.35]])  # process covariance matrix

if USE_ACCELERAMETER and USE_ROTARYENCODER:
    H = np.array([[1, 0, 0], [0, 0, 1]])  # state to obersvation matrix
    R = np.array([[D_VAR, 0], [0, A_VAR]])  # observation covariance matrix
elif USE_ACCELERAMETER:
    H = np.array([[0, 0, 1]])
    R = np.array([[A_VAR]])
elif USE_ROTARYENCODER:
    H = np.array([[1, 0, 0]])
    R = np.array([[D_VAR]])
else:
    print("No Sensor Active")
    exit()

# Other constants
WRAP_RATIO = wheel_diameter/axle_diameter

###############################################################################
# Functions


def reg_write(i2c, addr, reg, data):
    """
    Write bytes to the specified register.
    """

    # Construct message
    msg = bytearray()
    msg.append(data)

    # Write out message to register
    i2c.writeto_mem(addr, reg, msg)


def reg_read(i2c, addr, reg, nbytes=1):
    """
    Read byte(s) from specified register. If nbytes > 1, read from consecutive
    registers.
    """

    # Check to make sure caller is asking for 1 or more bytes
    if nbytes < 1:
        return bytearray()

    # Request data from specified register(s) over I2C
    data = i2c.readfrom_mem(addr, reg, nbytes)

    return data


def ab_change():
    """clockwise rotation positive counter change and is:
        a:  0,0,1,1,0,0,1,1
        b:  0,1,1,0,0,1,1,0
        c:   b+a+b-a-b+a+b-

    Returns:
        float: change in counter
    """
    global counter, rotary_state, temp_rotary_state, d_counter
    temp_rotary_state = (rotary_pin_a.value() << 1)+rotary_pin_b.value()  # type: ignore
    d_counter = COUNTER_TRANSITION[(rotary_state << 2) | temp_rotary_state]
    if d_counter == 0:
        return 0
    rotary_state = temp_rotary_state
    counter = counter + d_counter
    return d_counter


def a_inc(pin):
    if ab_change() != 0:
        rotary_pin_a.irq(trigger=machine.Pin.IRQ_FALLING, handler=a_dec, hard=True)


def a_dec(pin):
    if ab_change() != 0:
        rotary_pin_a.irq(trigger=machine.Pin.IRQ_RISING, handler=a_inc, hard=True)


def b_inc(pin):
    if ab_change() != 0:
        rotary_pin_b.irq(trigger=machine.Pin.IRQ_FALLING, handler=b_dec, hard=True)


def b_dec(pin):
    if ab_change() != 0:
        rotary_pin_b.irq(trigger=machine.Pin.IRQ_RISING, handler=b_inc, hard=True)


def start_record(pin):
    global is_ready
    is_ready = True


def terminate(pin):
    global should_terminate
    should_terminate = True


def dis_x_read() -> float:
    return EARTH_P[0]+counter*DIS_PER_TICK


def acc_x_read() -> float:
    # Read X, Y, and Z values from registers (16 bits each)
    data = reg_read(i2c1, ADXL343_ADDR, REG_DATAX0, 6)

    # Convert 2 bytes (little-endian) into 16-bit integer (signed)
    acc_x = ustruct.unpack_from("<h", data, 0)[0]

    # Convert measurements to [m/s^2]
    acc_x = acc_x * ACC_PER_TICK
    return acc_x


def kalman(x, z, P, F, Q, H, R):
    # this funciton is checked and behaves exactly like the simulation
    x = np.dot(F, x)
    P = np.dot(np.dot(F, P), F.T)+Q

    y = z-np.dot(H, x)
    K = np.dot(np.dot(P, H.T), (np.linalg.inv(np.dot(np.dot(H, P), H.T)+R)))

    x = x+np.dot(K, y)
    I = np.diag([1 for _ in range(3)])
    P = np.dot(I-np.dot(K, H), P)
    return x, P


def safe_config(a: float, b: float, c: float, d: float) -> tuple[float, float]:
    """get the direction and speed such that the ball goes into the center at 0.8m/s.

    Args:
        a (float): dx
        b (float): dy
        c (float): velocity that would get added to ball
        d (float): target entry speed

    Returns:
        _type_: tuple [float,float]
    """
    square_distance = (a**2+b**2)
    i_top = a*b*(d**2)+d*b*c*(square_distance**0.5)
    i_down = (d*a)**2-(c**2)*(square_distance)
    safe_angle = (atan2(i_top, i_down)+pi) % pi

    s_top = max(b*c, 0.1)
    s_down0 = a*sin(safe_angle)-b*cos(safe_angle)
    s_down = copysign(0.000001, s_down0) if (abs(s_down0) < 0.000001) else s_down0
    safe_speed = s_top/s_down

    return (safe_speed, safe_angle)


def slow_config(a: float, b: float, c: float, d: float,sc) -> tuple[float, float]:
    """get the direction and speed such that the ball goes into the center at 0.8m/s.

    Args:
        a (float): dx
        b (float): dy
        c (float): velocity that would get added to ball
        d (float): target entry speed

    Returns:
        _type_: tuple [float,float]
    """

    thresh = atan(b/a)
    slow_angle = (atan(-a/ b)) 
    slow_angle = slow_angle + pi*(floor((sc[1]-thresh)/pi)-floor((slow_angle-thresh)/pi))
    
    safe_speed = b*abs(c)/((a**2+b**2)**0.5)

    return (safe_speed, slow_angle)


def config_speed_for_angle(a: float, b: float, c: float, d: float, angle: float) -> float:
    return b*c/(-b*cos(angle)+a*sin(angle))


def approachable_config(a: float, b: float, c: float, d: float) -> tuple[float, float]:
    c1 = safe_config(a, b, c, d)
    return c1
    # if c1[0] > speed_lookup[0] and c1[0] < speed_lookup[-1]:
    #     # if the good speed is approachable
    #     return c1

    # c2 = slow_config(a, b, c, d, c1)
    # if c2[0] > speed_lookup[-1]:
    #     # if minimum speed required is not achieviable, just try to get to the slow point.
    #     return c2

    # mid_speed = (speed_lookup[0] + speed_lookup[-1])/2
    
    # print(c1,c2)

    # iter_count = 0
    # # in all other cases, just do some iteration.
    # for i in range(10):
    #     iter_count += 1

    #     if c2[1] < c1[1]:
    #         # order the two number by angle, smaller angle is c1.
    #         c2, c1 = c1, c2

    #     if (c2[0]-c1[0]) == 0:
    #         break

    #     interp = (mid_speed-c1[0])/(c2[0]-c1[0])

    #     if interp < 0:
    #         new_angle = c1[1]+max((c2[1]-c1[1])*interp, -pi/12)
    #         new_speed = config_speed_for_angle(a, b, c, d, new_angle)
    #         c2 = (new_speed, new_angle)
    #     elif interp > 1:
    #         new_angle = c1[1]+min((c2[1]-c1[1])*interp, pi/12)
    #         new_speed = config_speed_for_angle(a, b, c, d, new_angle)
    #         c1 = (new_speed, new_angle)
    #     else:
    #         new_angle = c1[1]+(c2[1]-c1[1])*interp
    #         new_speed = config_speed_for_angle(a, b, c, d, new_angle)
    #         if new_speed > mid_speed:
    #             c2 = (new_speed, new_angle)
    #         else:
    #             c1 = (new_speed, new_angle)
    
    # print(iter_count,c1,c2)

    # return c1


def config_predict(a: float, b: float, c: float, d: float, e: float) -> tuple[float, float]:
    """ check where the ball will fall on the lander entry graph is it launched at the moment.

    Args:
        a (float): dx
        b (float): dy
        c (float): velocity that would get added to ball
        d (float): current speed of ball off ramp
        e (float): current angle of ball off ramp

    Returns:
        _type_: tuple [float,float]
    """
    v_x = c+d*cos(e)
    v_y = d*sin(e)
    v_mag = ((v_x**2)+(v_y**2))**(0.5)

    dist = (v_y*a-v_x*b)/v_mag
    return (v_mag, dist)


def aim():
    v_car = (x[1, 0])

    # there is a trick here to assume that well is far enough away (the 2/7 part) for the ball to achieve no slip condition.
    ball_p = [
        x[0, 0] + 1.5*latency*v_car + (v_car)*abs(v_car)*((2/7)**2)/(2*ball_coefficient_friciton*G_ACCELERATION),
        EARTH_P[1]
    ]

    hole_rel_ball = [HOLE_P[0]-ball_p[0], HOLE_P[1]-ball_p[1]]

    safe_speed, safe_angle = approachable_config(hole_rel_ball[0], hole_rel_ball[1], (5/7)*v_car, 0.8)

    max_turn_angle = 1*dt

    if safe_speed > speed_lookup[0]:
        safe_speed = speed_lookup[0]
    elif safe_speed < speed_lookup[1]:
        safe_speed = speed_lookup[1]
    target_speed_angle = np.interp(safe_speed, speed_lookup, angle_lookup)[0]

    if (target_speed_angle-speed_servo.current_angle) >= max_turn_angle:
        target_speed_angle = speed_servo.current_angle+max_turn_angle
    elif (target_speed_angle-speed_servo.current_angle) <= -max_turn_angle:
        target_speed_angle = speed_servo.current_angle-max_turn_angle

    if target_speed_angle < speed_servo.current_angle:
        speed_servo.goto(target_speed_angle)

    if safe_angle < 0:
        safe_angle = 0
    elif safe_angle > pi:
        safe_angle = pi

    target_angle_angle = safe_angle
    if (target_angle_angle-angle_servo.current_angle) >= max_turn_angle:
        target_angle_angle = angle_servo.current_angle+max_turn_angle
    elif (target_angle_angle-angle_servo.current_angle) <= -max_turn_angle:
        target_angle_angle = angle_servo.current_angle-max_turn_angle

    angle_servo.goto(target_angle_angle)

###############################################################################
# Set up all the machinary


# Initialize I2C with pins
if USE_ACCELERAMETER:
    i2c1 = machine.I2C(0,
                       scl=machine.Pin(17),
                       sda=machine.Pin(16),
                       freq=400000)

    data = reg_read(i2c1, ADXL343_ADDR, REG_DEVID)
    data = int.from_bytes(data, "big") | (1 << 3)
    reg_write(i2c1, ADXL343_ADDR, REG_POWER_CTL, data)

# Initialize Rotary Encoder pins
if USE_ROTARYENCODER:
    rotary_pin_a = machine.Pin(0, mode=machine.Pin.IN)  # line closer to ground/ red line
    rotary_pin_b = machine.Pin(1, mode=machine.Pin.IN)  # line further to ground/ orange line

    temp_rotary_state = (rotary_pin_a.value() << 1)+rotary_pin_b.value()  # type: ignore
    d_counter = 0
    rotary_state = temp_rotary_state
    counter = 0

    if rotary_pin_a.value() == 0:
        rotary_pin_a.irq(trigger=machine.Pin.IRQ_RISING, handler=a_inc, hard=True)
    else:
        rotary_pin_a.irq(trigger=machine.Pin.IRQ_FALLING, handler=a_dec, hard=True)

    if rotary_pin_b.value() == 0:
        rotary_pin_b.irq(trigger=machine.Pin.IRQ_RISING, handler=b_inc, hard=True)
    else:
        rotary_pin_b.irq(trigger=machine.Pin.IRQ_FALLING, handler=b_dec, hard=True)

# Initialize Servo Pins
angle_servo = Servo(2)  # from 0 to 180 degrees
if IS_DEFAULT_DIRECTION:
    angle_servo.goto(0)
else:
    angle_servo.goto(180)
speed_servo = Servo(3)  # from 90 to 180 degrees 0 degrees to release
speed_servo.goto(max(angle_lookup[0], angle_lookup[-1]))

# Initialize READY Pin
ready_pin = machine.Pin(4, mode=machine.Pin.IN)  # line closer to ground/ red line

is_ready = False
recorded_frame = 0
if RECORD_DATA:
    fi = open("data.csv", "a")

ready_pin.irq(trigger=machine.Pin.IRQ_FALLING, handler=start_record, hard=True)

# Initialize RESET Pin
stop_pin = machine.Pin(5, mode=machine.Pin.IN, pull=machine.Pin.PULL_UP)  # line closer to ground/ red line

should_terminate = False

stop_pin.irq(trigger=machine.Pin.IRQ_FALLING, handler=terminate, hard=True)

# Initialize LED Pins
g_led = machine.PWM(machine.Pin(13))
g_led.freq(1000)
y_led = machine.PWM(machine.Pin(14))
y_led.freq(1000)
r_led = machine.PWM(machine.Pin(15))
r_led.freq(1000)

g_led.duty_u16(35000)
y_led.duty_u16(10000)
r_led.duty_u16(65000)

utime.sleep(0.25)

g_led.duty_u16(0)
y_led.duty_u16(0)
r_led.duty_u16(0)

utime.sleep(0.25)

###############################################################################
# Main

# Setup basic process
time = utime.ticks_ms()
ready_time = time
is_car_launched = False
car_launch_time = 0
is_ball_launched = False
ball_launch_time = 0

# Setup computes
x = np.array([[EARTH_P[0], 0, 0]]).T
P = np.diag([0.01, 0.01, 0.5])  # state variance


# Run forever
while True:
    # update time step
    n_time = utime.ticks_ms()
    dtick = utime.ticks_diff(n_time, time)
    dt = dtick/1000.0
    time = n_time
    

    # change prediction model for the discontinuity of detaching string
    if abs(x[0, 0]-EARTH_P[0]) < compression_length*WRAP_RATIO:
        F = np.array([[1, dt, 0.5*dt**2], [0, 1, dt], [0, -(k*dt)/(m*WRAP_RATIO**2), 1]])
    else:
        F = np.array([[1, dt, 0.5*dt**2], [0, 1, dt], [0, 0, 1]])

    if USE_ACCELERAMETER and USE_ROTARYENCODER:
        z = np.array([[dis_x_read(), acc_x_read()]]).T
    elif USE_ACCELERAMETER:
        z = np.array([[acc_x_read()]])
    else:
        z = np.array([[dis_x_read()]])

    # the following line takes 1.5ms
    x, P = kalman(x, z, P, F, Q, H, R)  # type:ignore

    # indication and recording
    if x[2, 0] >= -0.5 and x[2, 0] <= 0.5:
        y_led.duty_u16(int((1-abs(x[2, 0]))*10000))
    else:
        y_led.duty_u16(0)

    if is_ready:
        if recorded_frame == 0:
            g_led.duty_u16(65000)

            counter = 0
            x = np.array([[EARTH_P[0], 0, 0]]).T
            P = np.diag([0.01, 0.01, 2])  # state variance

            ready_time = time

            is_car_launched = False
            car_launch_time = 0
            is_ball_launched = False
            ball_launch_time = 0

        if RECORD_DATA:
            sensor_text = "0,0"
            if USE_ACCELERAMETER and USE_ROTARYENCODER:
                sensor_text = f"{z[0, 0]:5.3f},{z[1, 0]:5.3f}"
            elif USE_ACCELERAMETER:
                sensor_text = f"     0,{z[0, 0]:5.3f}"
            elif USE_ROTARYENCODER:
                sensor_text = f"{z[0, 0]:5.3f},     0"

            stage = 0
            if is_car_launched:
                stage += 1
            if is_ball_launched:
                stage += 1

            fi.write(f"{utime.ticks_diff(time, ready_time):6},{dtick:3},{sensor_text},{x[0, 0]:5.3f},{x[1, 0]:5.3f},{x[2, 0]:5.3f},{stage}\n")  # type:ignore

        recorded_frame += 1

        if recorded_frame == 1000:

            g_led.duty_u16(0)
            
            counter = 0
            x = np.array([[EARTH_P[0], 0, 0]]).T
            P = np.diag([0.01, 0.01, 2])  # state variance

            is_ready = False
            recorded_frame = 0

            is_car_launched = False
            car_launch_time = 0
            is_ball_launched = False
            ball_launch_time = 0

    # print("-"*20)
    # print("set: ", "{:11.3f}".format(time/1000.0), "{:6.3f}".format(dt), is_ready, is_car_launched, is_ball_launched)
    # print("sim: ", "{:5.3f}".format(x[0,0]), "{:5.3f}".format(x[1,0]), "{:5.3f}".format(x[2,0]))

    # Decision making
    if should_terminate:
        if RECORD_DATA:
            fi.close()  # type:ignore
        sys.exit(0)

    if not is_ready:
        aim()

    if (is_ready and not is_car_launched):
        aim()
        movement = abs(x[0, 0]-EARTH_P[0])
        if (movement >= abs(0.5*DIS_PER_TICK) or IS_STATIONARY and utime.ticks_diff(time, ready_time) <5000 and utime.ticks_diff(time, ready_time) >= 2000):
            is_car_launched = True
            car_launch_time = time

            if not IS_STATIONARY:
                x = np.array([[EARTH_P[0], (1.5 if IS_DEFAULT_DIRECTION else -1.5), k*(compression_length-movement/WRAP_RATIO)/(m*WRAP_RATIO) * (1 if IS_DEFAULT_DIRECTION else -1)]]).T
                P = np.diag([0.01, 0.1, 2])  # state variance

    if (is_car_launched and not is_ball_launched):

        # the first region finds the best ramp configuration
        v_car = (x[1, 0])

        # there is a trick here to assume that well is far enough away (the 2/7 part)
        ball_p = [
            x[0, 0] + latency*v_car + (v_car)*abs(v_car)*((2/7)**2)/(2*ball_coefficient_friciton*G_ACCELERATION),
            EARTH_P[1]
        ]

        hole_rel_ball = [HOLE_P[0]-ball_p[0], HOLE_P[1]-ball_p[1]]

        current_speed = np.interp(speed_servo.current_angle, angle_lookup, speed_lookup)[0]

        # the second region uses current ramp and find what would happen to the ball
        current_v_mag, current_dist = config_predict(hole_rel_ball[0], hole_rel_ball[1], (5/7)*v_car, current_speed, angle_servo.current_angle)

        if (current_v_mag > 0.5 and current_v_mag < 3 and  # the only reason for the low speed requirement is to make sure that the ball won't be diverted by uneveness.
                abs(current_dist) < 0.3):

            speed_servo.goto(0)
            is_ball_launched = True
            ball_launch_time = time

            r_led.duty_u16(65000)

        else:
            aim()

    if is_ball_launched:
        if utime.ticks_diff(time, ball_launch_time) > 1000:
            speed_servo.goto(3*pi/4)
            r_led.duty_u16(0)
