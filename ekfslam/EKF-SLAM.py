import math
import numpy as np

DT = 0.1  # time tick [s]
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # Landmark state size [x,y]


# x = (x, y, theta) - position of robot.
# u - control signal
# Math motion model


def motion_model(x, u):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])

    x = (F @ x) + (B @ u)
    return x


# calc_n_lm get X - the state vector(x, y, yaw, x1, y1, x2, y2, ...)
# returns count of landmarks
def calc_n_lm(x):
    n = int((len(x) - STATE_SIZE) / LM_SIZE)
    return n

# Its calculating matrix of Jacob. It get x - current state pos of robot (x, y, yaw), and get u.
def jacob_motion(x, u):
    # Fx - matrix linking the current state of the robot to the full state.
    Fx = np.hstack((np.eye(STATE_SIZE), np.zeros(
        (STATE_SIZE, LM_SIZE * calc_n_lm(x)))))

    jF = np.array([[0.0, 0.0, -DT * u[0, 0] * math.sin(x[2, 0])],
                   [0.0, 0.0, DT * u[0, 0] * math.cos(x[2, 0])],
                   [0.0, 0.0, 0.0]], dtype=float)
    # G - the state transition Jacob matrix. Used for update covariance state in EKF
    G = np.eye(len(x)) + Fx.T @ jF @ Fx

    return G, Fx