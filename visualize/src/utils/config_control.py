import numpy as np

# Vehicle config
wheelbase = 0.8  # wheel base: front to rear axle [m]
wheeldist = 0.6  # wheel dist: left to right wheel [m]
v_w = 0.70  # vehicle width [m]
r_b = 0.145  # rear to back [m]
r_f = wheelbase +  r_b  # rear to front [m]
t_r = 0.1425  # tire radius [m]
t_w = 0.10  # tire width [m]

c_f = 155494.663  # [N / rad]
c_r = 155494.663  # [N / rad]
m_f = 570  # [kg]
m_r = 570  # [kg]
l_f = 1.165  # [m]
l_r = 1.165  # [m]
Iz = 1436.24  # [kg m2]

# Controller Config
ts = 1/60  # [s]
max_iteration = 150
eps = 0.01

matrix_q = [1.0, 0.0, 1.0, 0.0]
matrix_r = [1.0]

state_size = 4

max_acceleration = 5.0  # [m / s^2]
max_steer_angle = np.deg2rad(30)  # [rad]
max_speed = 30 / 3.6  # [km/h]
