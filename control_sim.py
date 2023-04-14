import numpy as np
from random import random
from math import cos, sin, log, sqrt, pi
from matplotlib import pyplot

# apply zlema to simulated data
# result: not great, velocity data lags significantly

def generate_data(k, m, x_0, wrap_ratio):

    dt = 0.001

    t = [0]
    p = [0]
    v = [0]
    a = [0]

    for i in range(round(1/dt)):
        if x_0-p[-1]/wrap_ratio > 0:
            f = (k*(x_0-p[-1]/wrap_ratio))/wrap_ratio
        else:
            f = 0

        t.append(t[-1]+dt)
        a.append(f/m)
        p.append(p[-1]+v[-1]*dt+0.5*a[-1]*dt**2)
        v.append(v[-1]+a[-1]*dt)

    return t, p, v, a


k = 1243
m = 1.5
x_0 = 0.138
wrap_ratio = 10
meter_per_tick = (pi*0.08)*10/96

t_ref, p_ref, v_ref, a_ref = generate_data(k, m, x_0, wrap_ratio)


lag = 128
decay_factor = 1/lag

lag_q = lag >> 2

# first dimension left right
# second dimension up
p_hole_x = 3.5
p_hole_y = 1.9
py = 0
v_shoot = 1
a_shoot = 0.5*3.1415926

t_ref = t_ref
p_ref = p_ref
v_ref = v_ref
st = 0  # shoot time

# these initial values should be measurable given the test condition.
p_raw = [0]*(lag+1)
p_smoothed = 0

v_raw = [0]*(lag+1)
v_smoothed = 0

loop_dex = 0

shot = False

log = []
for i in range(1, len(t_ref)):
    # replace this region with actually getting the velocity and acceleration.

    # region external
    p_raw[loop_dex] = round(p_ref[i]/meter_per_tick)*meter_per_tick
    dt = t_ref[i]-t_ref[i-1]
    # endregion

    # region data filtering
    old_p = p_smoothed
    emadata_temp = 2.5*p_raw[loop_dex]-p_raw[(loop_dex+1+lag_q*2) % (lag+1)]-0.5*p_raw[(loop_dex+1) % (lag+1)]
    p_smoothed = (1-decay_factor)*old_p+decay_factor*emadata_temp

    v_raw[loop_dex] = (p_smoothed-old_p)/dt
    old_v = v_smoothed
    emadata_temp = 2*v_raw[loop_dex] - v_raw[(loop_dex+1) % (lag+1)]
    v_smoothed = (1-decay_factor)*old_v+decay_factor*emadata_temp
    loop_dex = (loop_dex+1) % (lag+1)
    log.append([p_smoothed, v_smoothed])
    # endregion

    # region decision making
    p_car = p_smoothed
    v_car = v_smoothed

    v_x = 5/7*v_car+v_shoot*cos(a_shoot)
    v_y = v_shoot*sin(a_shoot)

    v_mag = (v_x**2+v_y**2)**0.5

    v_unit_x = v_x/v_mag
    v_unit_y = v_y/v_mag

    # project the position of the hole and car onto the vector perpendicular to the ball exit velocity.
    # ideally, the difference is the radius.
    projected_hole = -v_unit_y*p_hole_x+v_unit_x*p_hole_y
    projected_pos = -v_unit_y*p_car+v_unit_x*py
    projected_vel = -v_unit_y*v_car
    distance_to_well = abs(projected_hole-projected_pos)
    count_down = distance_to_well/abs(projected_vel+0.00000000001)
    # endregion

    if not shot:
        dvdvs = (v_car*cos(a_shoot)+v_shoot)/v_mag
        dvda = (-v_shoot*v_car*sin(a_shoot))/v_mag
        if v_mag > 0.85:
            if dvdvs > 0:
                v_shoot -= 0.5/0.3*dt

            if dvda > 0 and a_shoot:
                a_shoot -= pi/0.3*dt
            else:
                a_shoot += pi/0.3*dt

        elif v_mag < 0.75:
            if dvdvs < 0:
                v_shoot -= 0.5/0.3*dt

            if dvda > 0:
                a_shoot += pi/0.3*dt
            else:
                a_shoot -= pi/0.3*dt

        if abs(projected_hole-projected_pos) < 0.38 and abs(projected_hole-projected_pos) > 0.32:
            print(p_ref[i], p_car)
            print(v_ref[i], v_car)
            print(v_shoot)
            print(a_shoot)
            st = t_ref[i]
            shot = True

log = np.array(log)

p_error = [log[i, 0]-p_ref[i] for i in range(len(log))]
v_error = [log[i, 1]-v_ref[i] for i in range(len(log))]

p_e = sqrt(sum([(p_error[i]**2)*(t_ref[i]-t_ref[i-1]) for i in range(1, len(t_ref)-1)]))
v_e = sqrt(sum([(v_error[i]**2)*(t_ref[i]-t_ref[i-1]) for i in range(1, len(t_ref)-1)]))

print("pe2", p_e)
print("ve2", v_e)

pyplot.plot(t_ref[1:], log[:, 0], label='p estimate')
pyplot.plot(t_ref[1:], log[:, 1], label='v estimate')
pyplot.plot(t_ref, p_ref, label='p reference')
pyplot.plot(t_ref, v_ref, label='v reference')
# pyplot.plot([st]*2, [-1, 1], label='launch trigger')
# pyplot.plot([st+0.05]*2, [-1, 1], label='launch end')
pyplot.legend()
pyplot.xlim([0, 1])
pyplot.ylim([-1, 5])
pyplot.show()
