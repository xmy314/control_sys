import numpy as np # type: ignore
from random import random
from math import cos, sin, sqrt, pi
from matplotlib import pyplot  # type: ignore

# apply kalman filter on raotary encoder and accelerameter
# have to use raspberry pi pico and micropython

# result: great

def generate_data(k, m, x_0, wrap_ratio):

    dt = 0.005

    t = [0]
    p = [0]
    v = [0]
    a = [0]

    for i in range(round(10/dt)):
        if x_0-p[-1]/wrap_ratio > 0:
            f = (k*(x_0-p[-1]/wrap_ratio))/wrap_ratio
        else:
            f = 0

        t.append(t[-1]+dt)  # type: ignore
        a.append(f/m)
        p.append(p[-1]+v[-1]*dt+0.5*a[-1]*dt**2)  # type: ignore
        v.append(v[-1]+a[-1]*dt)  # type: ignore

    return t, p, v, a


def kalman(x, z, P, F,  Q, H, R):
    # this funciton is checked and behaves exactly like the simulation
    x = np.dot(F, x)
    P = np.dot(np.dot(F, P), F.T)+Q

    y = z-np.dot(H, x)
    K = np.dot(np.dot(P, H.T), (np.linalg.inv(np.dot(np.dot(H, P), H.T)+R)))

    x = x+np.dot(K, y)
    P = np.dot(identity(3)-np.dot(K, H), P)
    return x, P


def identity(n):
    return np.diag([1 for _ in range(n)])


k = 1243
m = 1.5
x_0 = 0.138
wrap_ratio = 10
meter_per_tick = (pi*0.08)*20/96
# the 0.02 is a guess and need to be inputted for different material.
# this is not rolling friciton which is ignored completely
friciton_coefficient = 0.2

a_std = 0.1

t_ref, p_ref, v_ref, a_ref = generate_data(k, m, x_0, wrap_ratio)

x = np.array([[0, 0, 0*k*x_0/(m*wrap_ratio)]]).T  # type: ignore
P = np.diag([0.01, 0.01, 0.5])  # this is the process accuracy.
Q = np.array([[5.8e-13, 1.175e-9, 1.175e-6], [1.17e-9, 2.35e-6, 0.00235], [1.175e-6, 0.00235, 2.35]])


# first dimension left right
# second dimension up
p_hole_x = 3.5
p_hole_y = 1.9
py = 0
v_shoot = 1.6
a_shoot = 0.5*3.1415926

t_ref = t_ref
p_ref = p_ref
v_ref = v_ref
st = 0  # shoot time
shot = False


log0 = []
log1 = []

for i in range(1, len(t_ref)):
    dt = t_ref[i]-t_ref[i-1]

    # both
    # H = np.array([[1, 0, 0], [0, 0, 1]])
    # R = np.array([[((meter_per_tick)**2), 0], [0, a_std*a_std]]) # type: ignore
    # z = np.array([[(round(p_ref[i]/meter_per_tick))*meter_per_tick, a_ref[i]+a_std*np.random.randn()]]).T

    # position
    H = np.array([[1, 0, 0]])
    R = np.array([[((meter_per_tick)**2)/12]])
    z = np.array([[(round(p_ref[i]/meter_per_tick))*meter_per_tick]]).T

    # acc
    # H = np.array([[0, 0, 1]])
    # R = np.array([[ a_std*a_std]])
    # z = np.array([[ a_ref[i]+a_std*np.random.randn()]]).T

    if x[0] < x_0*wrap_ratio:
        F = np.array([[1, dt, 0.5*dt**2], [0, 1, dt], [0, -(k*dt)/(m*wrap_ratio**2), 1]])  # type: ignore
    else:
        F = np.array([[1, dt, 0], [0, 1, 0], [0, 0, 0]])

    x, P = kalman(x, z, P, F,  Q, H, R)  # type:ignore

    # region decision making
    v_car = x[1, 0]
    p_car = x[0, 0] + (v_car**2)/(2*friciton_coefficient*9.81)

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
        # there should be six derivatives
        # the position/speed with respect to angle/speed/time
        # then adjust launch accordingly

        dvdvs = (v_car*cos(a_shoot)+v_shoot)/v_mag
        dvda = (-v_shoot*v_car*sin(a_shoot))/v_mag
        if v_mag > 0.82:
            if dvdvs > 0:
                v_shoot -= 0.5/0.3*dt

            if dvda > 0 and a_shoot:
                a_shoot -= pi/0.3*dt
            else:
                a_shoot += pi/0.3*dt

        elif v_mag < 0.78:
            if dvdvs < 0:
                v_shoot -= 0.5/0.3*dt

            if dvda > 0:
                a_shoot += pi/0.3*dt
            else:
                a_shoot -= pi/0.3*dt

        if abs(projected_hole-projected_pos) < 0.38 and abs(projected_hole-projected_pos) > 0.32:
            print("k=", [p_ref[i], v_ref[i], v_shoot, a_shoot])
            st = t_ref[i]
            shot = True

    log0.append(x.copy()[:, 0])
    log1.append([P[0, 0], P[1, 1], P[2, 2]])


log0 = np.array(log0)
log1 = np.array(log1)

p_error = [log0[i, 0]-p_ref[i+1] for i in range(len(t_ref)-1)]
v_error = [log0[i, 1]-v_ref[i+1] for i in range(len(t_ref)-1)]

p_e = sqrt(sum([(p_error[i]**2)*(t_ref[i]-t_ref[i-1]) for i in range(1, len(t_ref)-1)]))
v_e = sqrt(sum([(v_error[i]**2)*(t_ref[i]-t_ref[i-1]) for i in range(1, len(t_ref)-1)]))

print("pe2", p_e)
print("ve2", v_e)

# pyplot.plot(t_ref[1:],log0[:,0]+np.sqrt(log1[:,0]))
# pyplot.plot(t_ref[1:],log0[:,0]-np.sqrt(log1[:,0]))
# pyplot.plot(t_ref[1:],log0[:,1]+np.sqrt(log1[:,1]))
# pyplot.plot(t_ref[1:],log0[:,1]-np.sqrt(log1[:,1]))
# pyplot.plot(t_ref[1:],log0[:,2]+np.sqrt(log1[:,2]))
# pyplot.plot(t_ref[1:],log0[:,2]-np.sqrt(log1[:,2]))
pyplot.plot(t_ref[1:], log0[:, 0], label='Position Estimate')
pyplot.plot(t_ref[1:], log0[:, 1], label='Velocity Estimate')
# pyplot.plot(t_ref[1:], log0[:, 2], label='a estimate')
pyplot.plot(t_ref, p_ref, label='Position Reference')
pyplot.plot(t_ref, v_ref, label='Velocity Reference')
# pyplot.plot(t_ref, a_ref, label='a reference')
pyplot.plot([st]*2, [-1, 1], label='launch trigger')
pyplot.plot([st+0.02]*2, [-1, 1], label='launch end')
pyplot.legend()
pyplot.xlim([0, 1])
pyplot.ylim([-1, 5])

pyplot.show()
