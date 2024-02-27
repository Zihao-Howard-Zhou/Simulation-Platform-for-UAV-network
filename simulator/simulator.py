from phy.channel import Channel
from entities.drone import Drone
from simulator.metrics import Metrics
from mobility import start_coords
from utils import config
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Simulator:
    """
    Description:

    """

    def __init__(self,
                 seed,
                 env,
                 channel_states,
                 n_drones,
                 routing_protocol=config.ROUTING_PROTOCOL,
                 total_simulation_time=config.SIM_TIME):

        self.env = env
        self.routing_protocol = routing_protocol

        self.total_simulation_time = total_simulation_time  # total simulation time (ns)

        self.n_drones = n_drones  # total number of drones in the simulation
        self.channel_states = channel_states
        self.channel = Channel(self.env)

        self.metrics = Metrics(self)  # use to record the network performance

        start_position = start_coords.get_random_start_point_3d(seed)

        fig = plt.figure()
        ax = Axes3D(fig)
        for xx in range(len(start_position)):
            pose = start_position[xx]
            ax.scatter3D(pose[0], pose[1], pose[2])

        plt.show()

        self.drones = []
        for i in range(n_drones):
            print('UAV: ', i, ' initial location is at: ', start_position[i])
            drone = Drone(env=env, node_id=i, coords=start_position[i], speed=70,
                          certain_channel=self.channel.create_store_for_receiver(), simulator=self)
            self.drones.append(drone)

        self.env.process(self.show_performance())
        self.env.process(self.show_time())

    def show_time(self):
        while True:
            print('At time: ', self.env.now / 1e6, ' s.')
            yield self.env.timeout(0.5*1e6)  # the simulation process is displayed every 0.5s

    def show_performance(self):
        yield self.env.timeout(self.total_simulation_time - 1)
        self.metrics.print_metrics()
