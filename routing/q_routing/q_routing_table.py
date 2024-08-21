import math
import random
import numpy as np
from collections import defaultdict


class QRoutingTable:
    def __init__(self, env, my_drone):
        self.env = env
        self.my_drone = my_drone
        self.neighbor_table = defaultdict(list)
        self.q_table = 30000 * np.ones((my_drone.simulator.n_drones, my_drone.simulator.n_drones))  # initialization
        self.entry_life_time = 2.5 * 1e6  # unit: us
        self.random_sd = self.my_drone.identifier * 1000

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

        self.neighbor_table[drone_id] = [position, cur_time]

    # delete the specified item
    def remove_neighbor(self, drone_id):
        del self.neighbor_table[drone_id]

    # determine whether a certain drone is one's neighbor
    def is_neighbor(self, drone_id):
        if drone_id in self.neighbor_table.keys():
            if self.get_updated_time(drone_id) + self.entry_life_time > self.env.now:  # valid neighbor
                return True
        else:
            return False

    # remove the expired item
    def purge(self):
        if self.is_empty():
            # it means that the neighbor table is empty
            return

        for key in list(self.neighbor_table):
            updated_time = self.get_updated_time(key)
            if updated_time + self.entry_life_time <= self.env.now:  # expired
                self.remove_neighbor(key)

    # clear neighbor table
    def clear(self):
        self.neighbor_table.clear()

    # get the minimum Q-value of my neighbors
    def get_min_q_value(self, dst_drone_id):
        self.purge()

        min_q = 1e10  # initial value
        for neighbor in self.neighbor_table.keys():
            min_q_temp = self.q_table[neighbor][dst_drone_id]
            if min_q_temp <= min_q:
                min_q = min_q_temp

        return min_q

    def best_neighbor(self, my_drone, dst_drone):
        """
        Choose the best next hop according to the Q-table
        :param my_drone: the drone that installed the GPSR
        :param dst_drone: the destination of the data packet
        :return: none
        """

        self.purge()

        dst_id = dst_drone.identifier

        random.seed(self.random_sd)

        if random.random() < 0.9 * math.pow(0.5, self.env.now / 1e6):
            best_id = random.choice(list(self.neighbor_table.keys()))
            self.random_sd += 1
        else:
            best_q_value = 1e10
            best_id = my_drone.identifier

            candidate_of_min_q_list = []

            for neighbor in self.neighbor_table.keys():
                if neighbor != self.my_drone.identifier:  # cannot forward the packet to myself
                    next_hop_q_value = self.q_table[neighbor][dst_id]
                    if next_hop_q_value <= best_q_value:
                        best_q_value = next_hop_q_value

            for neighbor in self.neighbor_table.keys():
                if neighbor != self.my_drone.identifier:
                    if self.q_table[neighbor][dst_id] == best_q_value:
                        candidate_of_min_q_list.append(neighbor)

            if len(candidate_of_min_q_list) != 0:
                random.seed(self.random_sd)
                best_id = random.choice(candidate_of_min_q_list)
                self.random_sd += 1

        return best_id
