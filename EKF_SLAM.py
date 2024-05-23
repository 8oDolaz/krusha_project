'''
More indepth documentation and explanation could be found here:
https://drive.google.com/file/d/0By_SW19c1BfhSVFzNHc0SjduNzg/view?resourcekey=0-41olC9ht9xE3wQe2zHZ45A
11.4 -- Robot localization

Even more examples could be found in this GitHub repo:
https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/tree/master?tab=readme-ov-file
'''

import numpy as np
import sympy as sp
from sympy import symbols, Matrix
from filterpy.kalman import ExtendedKalmanFilter as EKF


class EKF_SLAM(EKF):
    def __init__(self, dt,
                 std_velocity_lin, std_velocity_angl,
                 std_dev_range, std_dev_bearing,
                 wheelbase=200, wheelradius=100,
                 initial_pos=None):
        '''
        dt : int
            delta of time

        wheelbase and wheelradius : int
            specified in mm, change accorging to your robot


        dim_x : int
            Number of state variables for the Kalman filter.
            Default is 3, since we're tracking x, y, theta

        dim_z : int
            Number of measurement inputs. For example, if the sensor
            provides you with position in (x,y), dim_z would be 2.

        dim_u : int
            Number of variables for control input.
            We specify linear and rotational speed
        '''

        dim_x, dim_z, dum_u = 3, 2, 2
        EKF.__init__(self, dim_x, dim_z, dum_u)

        self.wheelradius = wheelradius
        self.wheelbase = wheelbase

        self.dt = dt
        self.std_velocity_lin = std_velocity_lin
        self.std_velocity_angl = std_velocity_angl

        x, y, v, w, theta, dt = symbols(
            'x, y, v, w, theta, t'
        )
        radius = v / w

        self.motion_model = Matrix([
            [x -radius*sp.sin(theta) + radius*sp.sin(theta + w*dt)],
            [y +radius*sp.cos(theta) - radius*sp.cos(theta + w*dt)],
            [theta + w*dt]
        ])

        self.motion_jacobian_pos = self.motion_model.jacobian(Matrix([x, y, theta]))
        self.motion_jacobian_cont = self.motion_model.jacobian(Matrix([v, w]))

        self.subs = {
            x: 0, y: 0, v: 0, w: 0, theta: 0,
            dt: self.dt
        }
        self.x = initial_pos if initial_pos is not None else np.array([[0], [0], [0]])
        self.v, self.w, self.theta = v, w, theta

        self.covariance = self.P = np.diag([.1, .1, .1])


    def predicted_position(self, control_signal):
        '''
        control_signal : vector of [ v, w ].T
            Control signal containing linear volocity and rotation speed
        '''

        self.x = self.expected_position(self.x, control_signal, self.dt)

        self.subs[self.theta] = self.x[2, 0]
        self.subs[self.v] = control_signal[0, 0]
        self.subs[self.w] = control_signal[1, 0]

        F = np.array(
            self.motion_jacobian_pos.evalf(subs=self.subs)).astype(np.float64)
        V = np.array(
            self.motion_jacobian_cont.evalf(subs=self.subs)).astype(np.float64)

        M = np.array([
            [self.std_velocity_lin ** 2, 0],
            [0, self.std_velocity_angl ** 2],
        ])

        self.covariance = F @ self.covariance @ F.T + V @ M @ V.T


    def expected_position(self, current_pos, control_signal, dt):
        '''
        Function computes position of robot based on control signal

        current_pos : vector of [ x, y, theta ].T
            Current position of a robot
        '''

        heading_angle = current_pos[2, 0]
        velocity_lin, velocity_angl = control_signal[0, 0], control_signal[1, 0]

        if abs(velocity_angl) > 0:
            angle_change = velocity_angl * dt
            radius = velocity_lin / velocity_angl

            dx = np.array([
                [-radius * sp.cos(heading_angle) + radius * sp.sin(heading_angle + angle_change)],
                [ radius * sp.cos(heading_angle) - radius * sp.sin(heading_angle + angle_change)],
                [velocity_angl * dt],
            ])
        else:
            dx = np.array([
                [velocity_lin * dt * sp.cos(heading_angle)],
                [velocity_lin * dt * sp.sin(heading_angle)],
                [0],
            ])

        return current_pos + dx


    def z_landmark(self, landmark):
        '''
        Compute observation of landmark

        lanmark : vector [ range, bearing ]
        '''

        distance = np.sqrt((landmark[0] - self.x[0])**2 + (landmark[1] - self.x[1])**2)

        angle = sp.atan2(landmark[1] - self.x[1, 0], landmark[0] - self.x[0, 0])
        angle -= self.x[3, 0]

        z = np.array([
            [distance + np.random.randn() * self.std_dev_range],
            [angle + np.random.randn() * self.std_dev_bearing]
        ])

        return z

    
    def ekf_update():
        ...


    # Probably this should be done in main loop of progam?
    def run_localization():
        ...


    @staticmethod
    def residual(a, b, /):
        '''
        Normalizes residual (a - b) between measurements of [range, bearing].
        Bearing normalized between [-pi, pi)

        a, b : vector [ range, bearing ]
        '''

        diff = a - b
        diff[1] %= 2*np.pi

        if diff[1] > np.pi:
            diff[1] -= 2*np.pi

        return diff
