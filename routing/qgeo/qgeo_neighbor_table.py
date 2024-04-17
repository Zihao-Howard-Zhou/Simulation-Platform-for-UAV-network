import numpy as np
from collections import defaultdict
from utils.util_function import euclidean_distance

class QGeoNeighborTable:
    """
    Neighbor table of QGeo (Q-Learning based Geographic routing protocol)

    Main function: 1) determine the available (optional) neighbor, 2) determine if local maximum (void area) is occurred

    References:
        [1] Jung W S, Yim J, Ko Y B., "QGeo: Q-learning-based geographic ad hoc routing protocol for unmanned robotic
            networks," IEEE Communications Letters, 21(10): 2258-2261, 2017.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/4/17
    Updated at: 2024/4/17
    """

    def __init__(self, env, my_drone):
        self.env = env
        self.my_drone = my_drone
        self.neighbor_table = defaultdict(list)
        self.q_table = 0.5 * np.ones((my_drone.simulator.n_drones, my_drone.simulator.n_drones))
        self.entry_life_time = 1 * 1e6  # unit: us (1s)

    # determine if the neighbor table is empty
    def is_empty(self):
        return not bool(self.neighbor_table)

    # get the updated time of certain item
    def get_updated_time(self, drone_id):
        if drone_id not in self.neighbor_table.keys():
            raise RuntimeError('This item is not in the neighbor table')
        else:
            return self.neighbor_table[drone_id][-1]

    def add_neighbor(self, hello_packet, cur_time):
        """
        Update the neighbor table according to the hello packet
        :param hello_packet: the received hello packet
        :param cur_time: the moment when the packet is received
        :return: none
        """

        drone_id = hello_packet.src_drone.identifier
        position = hello_packet.cur_position
        velocity = hello_packet.cur_velocity

        self.neighbor_table[drone_id] = [position, velocity, cur_time]

    # delete the specified item
    def remove_neighbor(self, drone_id):
        del self.neighbor_table[drone_id]

    # determine whether a certain drone is one's neighbor
    def is_neighbor(self, drone_id):
        return drone_id in self.neighbor_table.keys()

    # get the position of a neighbor node
    def get_neighbor_position(self, drone_id):
        if self.is_neighbor(drone_id):
            return self.neighbor_table[drone_id][0]  # return the position list
        else:
            raise RuntimeError('This drone is not my neighbor!')

    def get_neighbor_velocity(self, drone_id):
        if self.is_neighbor(drone_id):
            return self.neighbor_table[drone_id][1]  # return the velocity list
        else:
            raise RuntimeError('This drone is not my neighbor!')

    # determine if void area is occurred
    def judge_void_area(self, dst_drone):
        beacon_distance = euclidean_distance(self.my_drone.coords, dst_drone.coords)
        have_void_area = 1
        for key in self.neighbor_table.keys():
            next_hop_position = self.get_neighbor_position(key)
            temp_distance = euclidean_distance(next_hop_position, dst_drone.coords)
            if temp_distance < beacon_distance:
                have_void_area = 0

        return have_void_area

    # remove the expired item
    def purge(self):
        if not bool(self.neighbor_table):
            # it means that the neighbor table is empty
            return

        for key in list(self.neighbor_table):
            updated_time = self.get_updated_time(key)
            if updated_time + self.entry_life_time < self.env.now:  # expired
                self.remove_neighbor(key)

    # clear neighbor table
    def clear(self):
        self.neighbor_table.clear()

    # get the minimum Q-value of my neighbors
    def get_max_q_value(self, dst_drone_id):
        max_q = 0
        for neighbor in self.neighbor_table.keys():
            max_q_temp = self.q_table[neighbor][dst_drone_id]
            if max_q_temp > max_q:
                max_q = max_q_temp

        return max_q

    def best_neighbor(self, my_drone, dst_drone):
        """
        Choose the best next hop according to the neighbor table
        :param my_drone: the drone that installed the GPSR
        :param dst_drone: the destination of the data packet
        :return: none
        """

        dst_id = dst_drone.identifier
        best_q_value = 0
        best_id = my_drone.identifier

        for row in range(self.my_drone.simulator.n_drones):
            if self.is_neighbor(row):
                next_hop_q_value = self.q_table[row][dst_id]
                if next_hop_q_value > best_q_value:
                    best_q_value = next_hop_q_value
                    best_id = row

        return best_id
