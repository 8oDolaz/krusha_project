import math
from math import cos, sin

import numpy as np
import matplotlib.pyplot as plt
from filterpy.stats import plot_covariance_ellipse
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class ImuSensor():
    def __init__(self, objectName):
        self.objectName = objectName
        self.velocity_x_prev = 0
        self.velocity_y_prev = 0
        self.velocity_z_prev = 0


    # Accelerometer simulation
    def get_accel(self):
        linear_velocity_axis = sim.getObjectVelocity(self.objectName)[0]
        accel_x = (linear_velocity_axis[0] - self.velocity_x_prev) / dt
        accel_y = (linear_velocity_axis[1] - self.velocity_y_prev) / dt
        accel_z = (linear_velocity_axis[2] - self.velocity_z_prev) / dt
        self.velocity_x_prev = linear_velocity_axis[0]
        self.velocity_y_prev = linear_velocity_axis[1]
        self.velocity_z_prev = linear_velocity_axis[2]
        return accel_x, accel_y, accel_z


    # Gyroscope simulation
    def get_yaw_velocity(self):
        angular_velocity_axis = sim.getObjectVelocity(self.objectName)[1]
        return angular_velocity_axis[2]


    # Compas simulation
    def get_yaw(self):
        angles = sim.getObjectOrientation(self.objectName)
        return angles[2]


class UltraSonic():
    def __init__(self, x, y, alpha, objectName):
        self.x = x
        self.y = y
        self.alpha = alpha
        self.objectName = objectName

    def measure_distance(self):
        res, dist, point, obj, n = sim.checkProximitySensor(self.objectName, sim.handle_all)
        if res:
            return dist


class Encoder():
    def __init__(self, objectName):
        self.objectName = objectName

    def get_velocity(self):
        velocity_x, velocity_y, velocity_z = sim.getObjectVelocity(self.objectName)[0]
        yaw = sim.getObjectOrientation(self.objectName)[2]
        velocity_local = velocity_x * math.cos(yaw) + velocity_y * math.sin(yaw)
        return velocity_local


class WheelControl():
    def __init__(self, leftBehindWheelName, rightBehindWheelName, leftFrontWheelName, rightFrontWheelName):
        self.leftBehindWheel = leftBehindWheelName
        self.rightBehindWheel = rightBehindWheelName
        self.leftFrontWheel = leftFrontWheelName
        self.rightFrontWheel = rightFrontWheelName

    def __drive(self, left_back, right_back, left_front, right_front):
        sim.setJointTargetVelocity(self.leftBehindWheel, left_back)
        sim.setJointTargetVelocity(self.rightBehindWheel, right_back)
        sim.setJointTargetVelocity(self.leftFrontWheel, left_front)
        sim.setJointTargetVelocity(self.rightFrontWheel, right_front)

    def move_forward(self, left_back=1, right_back=1, left_front=1, right_front=1):
        self.__drive(left_back, right_back, left_front, right_front)

    def move_stop(self, left_back=0, right_back=0, left_front=0, right_front=0):
        self.__drive(left_back, right_back, left_front, right_front)

    def rotateLeft(self, left_back=1, right_back=-1, left_front=1, right_front=-1):
        self.__drive(left_back, right_back, left_front, right_front)

    def rotateRight(self, left_back=-1, right_back=1, left_front=-1, right_front=1):
        self.__drive(left_back, right_back, left_front, right_front)


def sysCall_init():
    global leftBehindWheel, rightBehindWheel, leftFrontWheel, rightFrontWheel, leftSensor, rightSensor, frontSensor, dt, main_vehicle, imu_sensor, encoder, wheel_control, ax,flag_move_stop, flag_rotate_right, flag_rotate_left, mu, flag_yawing_end

    # Vehicle
    main_vehicle = sim.getObject('./Cuboid')
    imu_sensor = ImuSensor(main_vehicle)

    # Drivers
    wheel_control = WheelControl(sim.getObject('./LeftBehindWheel'), sim.getObject('./RightBehindWheel'), sim.getObject('./LeftFrontWheel'), sim.getObject('./RightFrontWheel'))
    # matplotlib.use('TkAgg')
    # UltraSonic Sensor
    encoder = Encoder(main_vehicle)
    leftSensor = UltraSonic(0.175, 0.1, np.deg2rad(90), sim.getObject('./LeftSensor'))
    rightSensor = UltraSonic(0.175, -0.1, np.deg2rad(-90), sim.getObject('./RightSensor'))
    frontSensor = UltraSonic(0.175, 0, np.deg2rad(0), sim.getObject('./FrontSensor'))


def ekf_slam(xEst, PEst, u, z):
    # Predict
    G, Fx = jacob_motion(xEst, u)
    xEst[0:STATE_SIZE] = motion_model(xEst[0:STATE_SIZE], u)
    PEst = G.T @ PEst @ G + Fx.T @ Cx @ Fx
    initP = np.zeros((2, 2))

    # Update
    for iz in range(len(z[:, 0])):  # for each observation
        min_id = search_correspond_landmark_id(xEst, PEst, z[iz, 0:2])

        nLM = calc_n_lm(xEst)
        if min_id == nLM:
            print("New LM")
            # Extend state and covariance matrix
            xAug = np.vstack((xEst, calc_landmark_position(xEst, z[iz, :])))
            PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), LM_SIZE)))),
                              np.hstack((np.zeros((LM_SIZE, len(xEst))), initP))))
            xEst = xAug
            PEst = PAug
        lm = get_landmark_position_from_state(xEst, min_id)
        y, S, H = calc_innovation(lm, xEst, PEst, z[iz, 0:2], min_id)

        K = (PEst @ H.T) @ np.linalg.inv(S)
        xEst = xEst + (K @ y)
        PEst = (np.eye(len(xEst)) - (K @ H)) @ PEst

    xEst[2] = pi_2_pi(xEst[2])

    return xEst, PEst


def observation(xTrue, xd, u):
    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = np.zeros((0, 3))
    for i in range(len(points)):
        dx = points[i].x - xTrue[0, 0]
        dy = points[i].y - xTrue[1, 0]
        d = math.hypot(dx, dy)
        angle = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])
        dn = d
        angle_n = angle
        zi = np.array([dn, angle_n, i])
        z = np.vstack((z, zi))
    # add noise to input
    ud = np.array([[
        u[0, 0] + np.random.randn() * R_sim[0, 0] ** 0.5,
        u[1, 0] + np.random.randn() * R_sim[1, 1] ** 0.5]]).T

    xd = motion_model(xd, ud)
    return xTrue, z, xd, ud


def motion_model(x, u):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])

    x = (F @ x) + (B @ u)
    return x


def calc_n_lm(x):
    n = int((len(x) - STATE_SIZE) / LM_SIZE)
    return n


def jacob_motion(x, u):
    Fx = np.hstack((np.eye(STATE_SIZE), np.zeros(
        (STATE_SIZE, LM_SIZE * calc_n_lm(x)))))

    jF = np.array([[0.0, 0.0, -DT * u[0, 0] * math.sin(x[2, 0])],
                   [0.0, 0.0, DT * u[0, 0] * math.cos(x[2, 0])],
                   [0.0, 0.0, 0.0]], dtype=float)

    G = np.eye(len(x)) + Fx.T @ jF @ Fx

    return G, Fx,


def calc_landmark_position(x, z):
    zp = np.zeros((2, 1))

    zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
    zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])

    return zp


def get_landmark_position_from_state(x, ind):
    lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

    return lm


def search_correspond_landmark_id(xAug, PAug, zi):
    """
    Landmark association with Mahalanobis distance
    """

    nLM = calc_n_lm(xAug)

    min_dist = []
    for i in range(nLM):
        lm = get_landmark_position_from_state(xAug, i)
        y, S, H = calc_innovation(lm, xAug, PAug, zi, i)
        min_dist.append(y.T @ np.linalg.inv(S) @ y)

    min_dist.append(M_DIST_TH)  # new landmark
    min_id = min_dist.index(min(min_dist))
    return min_id


def calc_innovation(lm, xEst, PEst, z, LMid):
    delta = lm - xEst[0:2]
    q = (delta.T @ delta)[0, 0]
    z_angle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
    zp = np.array([[math.sqrt(q), pi_2_pi(z_angle)]])
    y = (z - zp).T
    y[1] = pi_2_pi(y[1])
    H = jacob_h(q, delta, xEst, LMid + 1)
    S = H @ PEst @ H.T + Q_sim
    return y, S, H


def jacob_h(q, delta, x, i):
    sq = math.sqrt(q)
    G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])

    G = G / q
    nLM = calc_n_lm(x)
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))

    F = np.vstack((F1, F2))
    H = G @ F

    return H


def pi_2_pi(angle):
    return angle_mod(angle)


def angle_mod(x, zero_2_2pi=False, degree=False):
    if isinstance(x, float):
        is_float = True
    else:
        is_float = False

    x = np.asarray(x).flatten()
    if degree:
        x = np.deg2rad(x)

    if zero_2_2pi:
        mod_angle = x % (2 * np.pi)
    else:
        mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

    if degree:
        mod_angle = np.rad2deg(mod_angle)

    if is_float:
        return mod_angle.item()
    else:
        return mod_angle


class Point():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.r = math.sqrt(x ** 2 + y ** 2)
        self.phi = math.atan2(y, x)


def get_coordinates_from_measure(sensor, robot_x, robot_y):
    global points
    r = sensor.measure_distance()
    yaw = imu_sensor.get_yaw()
    if r:
        rotateMatrix = np.array([[cos(yaw), -sin(yaw)],
                                 [sin(yaw), cos(yaw)]])

        coordinates = rotateMatrix @ (np.array([[r * cos(sensor.alpha), r * sin(sensor.alpha)]]).T + np.array([[sensor.x, sensor.y]]).T) + np.array([[robot_x, robot_y]]).T

        points.append(Point(coordinates[0, 0], coordinates[1, 0]))


client = RemoteAPIClient()
sim = client.getObject('sim')
sim.setStepping(True)
sim.startSimulation()

sysCall_init()

points = []

DT = 0.050  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range
M_DIST_TH = 5  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]

show_animation = True

xEst = np.zeros((STATE_SIZE, 1))
xTrue = np.zeros((STATE_SIZE, 1))
PEst = np.zeros(STATE_SIZE)
xDR = np.zeros((STATE_SIZE, 1))  # Dead reckoning

# history
hxEst = xEst
hxTrue = xTrue
hxDR = xTrue
hxEst = xEst
hxTrue = xTrue
hxDR = xTrue

# EKF state covariance
Cx = np.diag([
    0.01,
    0.01,
    np.deg2rad(30)
]) ** 2

#  Simulation parameter
Q_sim = np.diag([
    0.01,
    np.deg2rad(1)
]) ** 2

R_sim = np.diag([0.05, np.deg2rad(1)]) ** 2

while (t := sim.getSimulationTime()) < 150:
    wheel_control.move_forward()

    # GET MEASUREMENTS
    get_coordinates_from_measure(leftSensor, xTrue[0, 0], xTrue[1, 0])
    get_coordinates_from_measure(rightSensor, xTrue[0, 0], xTrue[1, 0])
    get_coordinates_from_measure(frontSensor, xTrue[0, 0], xTrue[1, 0])
    # GET SIGNAL
    u = np.array([[encoder.get_velocity(), imu_sensor.get_yaw_velocity()]]).T
    xTrue, z, xDR, ud = observation(xTrue, xDR, u)
    xEst, PEst = ekf_slam(xEst, PEst, ud, z)
    x_state = xEst[0:STATE_SIZE]

    # store data history
    hxEst = np.hstack((hxEst, x_state))
    hxDR = np.hstack((hxDR, xDR))
    hxTrue = np.hstack((hxTrue, xTrue))

    if show_animation:  # pragma: no cover
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        plt.plot(xEst[0], xEst[1], ".r")
        for p in points:
            plt.plot(p.x,
                     p.y, "xb")

        # plot landmark
        for i in range(calc_n_lm(xEst)):
            lm_index = STATE_SIZE + i * LM_SIZE

            plt.plot(xEst[lm_index],
                     xEst[lm_index + 1], "xg")

            plot_covariance_ellipse(
                (xEst[lm_index], xEst[lm_index + 1]),
                PEst[lm_index:lm_index + 2, lm_index:lm_index + 2],
                std=6, facecolor='g', alpha=0.8
            )

        plt.plot(hxTrue[0, :],
                 hxTrue[1, :], "-b")
        plt.plot(hxDR[0, :],
                 hxDR[1, :], "-k")
        plt.plot(hxEst[0, :],
                 hxEst[1, :], "-r")

        plot_covariance_ellipse(
            (xEst[0, 0], xEst[1, 0]),
            PEst[0:2, 0:2],
            std=6, facecolor='g', alpha=0.8
        )

        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.01)
    points = []
    sim.step()

sim.stopSimulation()
