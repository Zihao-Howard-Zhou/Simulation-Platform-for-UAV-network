import math
from collections import defaultdict


class ParrotNeighborTable:
    """
    Neighbor table of PARRoT (v1.0)

    the only usage of neighbor table is to calculate the cohesion

    Attributes:
        env:
        my_drone: the drone that installed the PARRoT protocol
        neighbor_table: a dictionary, used to record the neighbor drone and its updated time
        entry_life_time: each entry of the neighbor table has its lifetime, those expired items will be removed
        delta_t: used to calculate changes in neighbors at different times
        cohesion: served as a measure for the neighbor set coherence

    References:
        [1] B. Sliwa, et al.,"PARRoT: Predictive Ad-hoc Routing Fueled by Reinforcement Learning and Trajectory Knowledge,"
            in IEEE 93rd Vehicular Technology Conference (VTC2021-Spring), pp. 1-7, 2021.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/3/26
    Updated at: 2024/3/26
    """

    def __init__(self, env, my_drone):
        self.env = env
        self.my_drone = my_drone
        self.neighbor_table = defaultdict(list)
        self.entry_life_time = 1*1e6  # unit: us (1s)
        self.delta_t = 2.5*1e6  #
        self.cohesion = 1.0
        self.env.process(self.calculate_cohesion())

    # determine if the neighbor table is empty
    def is_empty(self):
        return not bool(self.neighbor_table)

    # get the updated time of certain item
    def get_updated_time(self, drone_id):
        if drone_id not in self.neighbor_table.keys():
            raise RuntimeError('This item is not in the neighbor table')
        else:
            return self.neighbor_table[drone_id][0]

    def add_neighbor(self, chirp_packet, cur_time):
        """
        Update the neighbor table according to the chirp packet
        :param chirp_packet: the received hello packet
        :param cur_time: the moment when the packet is received
        :return: None
        """

        drone_id = chirp_packet.src_drone.identifier
        self.neighbor_table[drone_id] = [cur_time]

    # delete the specified item
    def remove_neighbor(self, drone_id):
        del self.neighbor_table[drone_id]

    # remove the expired item
    def purge(self):
        if self.is_empty():
            # it means that the neighbor table is empty
            return

        for key in list(self.neighbor_table):
            updated_time = self.get_updated_time(key)
            if updated_time + self.entry_life_time < self.env.now:  # expired
                self.remove_neighbor(key)

    def calculate_cohesion(self):
        # calculate changes in neighbor set every "delta_t"
        while True:
            neighbor_set_last_time = set(self.neighbor_table.keys())
            if len(neighbor_set_last_time) == 0:
                neighbor_set_last_time = {self.my_drone.identifier}  # initial case

            yield self.env.timeout(self.delta_t)

            neighbor_set_now = set(self.neighbor_table.keys())

            symmetric_diff = neighbor_set_now.symmetric_difference(neighbor_set_last_time)
            molecular = len(symmetric_diff)

            union_set = neighbor_set_now.union(neighbor_set_last_time)
            denominator = len(union_set)

            self.cohesion = math.sqrt(1 - molecular / denominator)
            print('UAV: ', self.my_drone.identifier, ' coherence is: ', self.cohesion)
