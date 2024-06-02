import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class Cart:
    def __init__(self, sim, x, y, angle, k_s=0.3, k_r=3):
        self.right_front_wheel = sim.getObject('./RightFrontWheel')
        self.right_rear_wheel = sim.getObject('./RightRearWheel')

        self.left_front_wheel = sim.getObject('./LeftFrontWheel')
        self.left_rear_wheel = sim.getObject('./LeftRearWheel')

        self.object = sim.getObject('./Cuboid')

        # position of cart center
        self.x = x
        self.y = y
        # angle of a cart relative to x axis
        self.angle = angle

        self.k_s = k_s
        self.k_r = k_r


    def compute_control_signal(self):
        distance_to_goal = self.distance_to_goal()
        angle_to_goal = self.angle_to_goal()

        linear_speed = self.k_s * distance_to_goal * math.cos(angle_to_goal)
        
        angular_speed = self.k_s * math.cos(angle_to_goal) * math.sin(angle_to_goal)
        angular_speed += self.k_r * angle_to_goal

        speed_r = linear_speed + angular_speed
        speed_l = linear_speed - angular_speed

        return speed_r, speed_l


    def drive(self, speed_r, speed_l, sim):
        sim.setJointTargetVelocity(self.right_front_wheel, speed_r)
        sim.setJointTargetVelocity(self.right_rear_wheel, speed_r)

        sim.setJointTargetVelocity(self.left_front_wheel, speed_l)
        sim.setJointTargetVelocity(self.left_rear_wheel, speed_l)


    def set_goal(self, x, y):
        self.x_goal = x
        self.y_goal = y


    def distance_to_goal(self):
        distance_x = self.x_goal - self.x
        distance_y = self.y_goal - self.y

        return (distance_x**2 + distance_y**2) ** (1/2)


    def angle_to_goal(self):
        azimuth = math.atan2(self.y_goal - self.y, self.x_goal - self.x)
        azimuth -= self.angle

        azimuth = self.__normalize_angle(azimuth)

        return azimuth


    @staticmethod
    def __normalize_angle(angle):
        if angle <= -math.pi:
            return angle + 2 * math.pi
        elif angle >= math.pi:
            return angle - 2 * math.pi
        return angle


client = RemoteAPIClient()
sim = client.getObject('sim')
sim.setStepping(True)
sim.startSimulation()

print('Simulation started')

simulation_time = 150

cart = Cart(
    sim, 
    0, 0, 0,
    k_s=3, k_r=3,
)
cart.set_goal(-1, -1)

print('Cart created and goal set') 

while cart.distance_to_goal() > 0.05:
    control_signal = cart.compute_control_signal()

    cart.drive(*control_signal, sim)

    cart.x, cart.y = sim.getObjectPosition(cart.object)[:-1]
    cart.angle = sim.getObjectOrientation(cart.object)[-1]

    sim.step()
