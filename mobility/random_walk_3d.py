import numpy as np
import random
from utils import config
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RandomWalk3D:
    def __init__(self, drone):
        self.my_drone = drone
        self.move_counter = 1
        self.position_update_interval = 1 * 1e5  # 0.1s
        self.travel_duration = 4*1e6  # the travelling time in the new direction

        self.b1 = 50  # safety boundary of x-axis
        self.b2 = 50  # safety boundary of y-axis
        self.b3 = 3  # safety boundary of z-axis

        self.min_x = 0
        self.max_x = config.MAP_LENGTH

        self.min_y = 0
        self.max_y = config.MAP_WIDTH

        self.min_z = 0
        self.max_z = config.MAP_HEIGHT

        self.my_drone.simulator.env.process(self.mobility_update(self.my_drone))
        self.trajectory = []
        # self.my_drone.simulator.env.process(self.show_trajectory())

    def mobility_update(self, drone):
        while True:
            env = drone.simulator.env
            drone_id = drone.identifier
            cur_position = drone.coords
            cur_velocity = drone.velocity
            cur_direction = drone.direction
            cur_pitch = drone.pitch

            # update the position of next time step
            if config.STATIC_CASE == 0:
                next_position_x = cur_position[0] + cur_velocity[0] * self.position_update_interval / 1e6
                next_position_y = cur_position[1] + cur_velocity[1] * self.position_update_interval / 1e6
                next_position_z = cur_position[2] + cur_velocity[2] * self.position_update_interval / 1e6
            else:
                next_position_x = cur_position[0]
                next_position_y = cur_position[1]
                next_position_z = cur_position[2]

            cur_speed = ((cur_velocity[0] ** 2) + (cur_velocity[1] ** 2) + (cur_velocity[2] ** 2)) ** 0.5

            if env.now % self.travel_duration == 0:  # update velocity and direction
                self.move_counter += 1

                random.seed(drone_id + 1 + self.move_counter)
                next_direction = random.uniform(0, 2 * np.pi)

                random.seed(drone_id + 1000 + self.move_counter)
                next_pitch = random.uniform(-0.05, 0.05)

                next_velocity_x = cur_speed * np.cos(next_direction) * np.cos(next_pitch)
                next_velocity_y = cur_speed * np.sin(next_direction) * np.cos(next_pitch)
                next_velocity_z = cur_speed * np.sin(next_pitch)

                if type(next_position_x) is np.ndarray:
                    next_position_x = next_position_x[0]
                    next_position_y = next_position_y[0]
                    next_position_z = next_position_z[0]

                next_position = [next_position_x, next_position_y, next_position_z]

                if drone_id == 1:
                    self.trajectory.append(next_position)

                if type(next_velocity_x) is np.ndarray:
                    next_velocity_x = next_velocity_x[0]
                    next_velocity_y = next_velocity_y[0]
                    next_velocity_z = next_velocity_z[0]

                next_velocity = [next_velocity_x, next_velocity_y, next_velocity_z]
                next_speed = ((next_velocity_x ** 2) + (next_velocity_y ** 2) + (next_velocity_z ** 2)) ** 0.5
            else:
                next_position = [next_position_x, next_position_y, next_position_z]

                # velocity, direction and pitch should stay the same
                next_direction = cur_direction
                next_pitch = cur_pitch
                next_velocity = cur_velocity
                next_speed = ((cur_velocity[0] ** 2) + (cur_velocity[1] ** 2) + (cur_velocity[2] ** 2)) ** 0.5

            # wall rebound
            next_position, next_velocity, next_direction, next_pitch = self.boundary_test(next_position, next_velocity,
                                                                                          next_direction, next_pitch)

            drone.coords = next_position
            drone.direction = next_direction
            drone.pitch = next_pitch
            drone.velocity = next_velocity

            yield env.timeout(self.position_update_interval)
            energy_consumption = (self.position_update_interval / 1e6) * drone.energy_model.power_consumption(drone.speed)
            drone.residual_energy -= energy_consumption

    def show_trajectory(self):
        x = []
        y = []
        z = []
        yield self.my_drone.simulator.env.timeout(config.SIM_TIME-1)
        if self.my_drone.identifier == 1:
            for i in range(len(self.trajectory)):
                x.append(self.trajectory[i][0])
                y.append(self.trajectory[i][1])
                z.append(self.trajectory[i][2])

            plt.figure()
            ax = plt.axes(projection='3d')
            ax.set_xlim(self.min_x, self.max_x)
            ax.set_ylim(self.min_y, self.max_y)
            ax.set_zlim(self.min_z, self.max_z)

            x = np.array(x)
            y = np.array(y)
            z = np.array(z)

            ax.plot(x, y, z)
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            plt.show()

    # rebound scheme
    def boundary_test(self, next_position, next_velocity, next_direction, next_pitch):
        if next_position[0] < self.min_x + self.b1 or next_position[0] > self.max_x - self.b1:
            next_velocity[0] = -next_velocity[0]
        if next_position[1] < self.min_y + self.b2 or next_position[1] > self.max_y - self.b2:
            next_velocity[1] = -next_velocity[1]
        if next_position[2] < self.min_z + self.b3 or next_position[2] > self.max_z - self.b3:
            next_velocity[2] = -next_velocity[2]

        next_position[0] = np.clip(next_position[0], self.min_x + self.b1, self.max_x - self.b1)
        next_position[1] = np.clip(next_position[1], self.min_y + self.b2, self.max_y - self.b2)
        next_position[2] = np.clip(next_position[2], self.min_z + self.b3, self.max_z - self.b3)

        return next_position, next_velocity, next_direction, next_pitch
