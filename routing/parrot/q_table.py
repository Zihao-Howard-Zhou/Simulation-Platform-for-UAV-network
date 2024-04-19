import logging
import math
import numpy as np
from utils import config
from phy.large_scale_fading import maximum_communication_range


class Qtable:
    """
    Q-table of PARRoT (v1.0)

    type of the Q-table: two-dimensional dictionary
    the structure of the Q-table is:
    |                  |     action a1    |     action a2    |  ...  |     action an    |
    |------------------|------------------|------------------|-------|------------------|
    |  destination d1  | [Q(d1, a1), SEQ] | [Q(d1, a2), SEQ] |  ...  | [Q(d1, an), SEQ] |
    |  destination d2  | [Q(d2, a1), SEQ] | [Q(d2, a2), SEQ] |  ...  | [Q(d2, an), SEQ] |
    |       ...        |
    |  destination dn  | [Q(dn, a1), SEQ] | [Q(dn, a2), SEQ] |  ...  | [Q(dn, an), SEQ] |
    (n is the number of drones in network)

    Attributes:
        env:
        my_drone: the drone that installed the PARRoT
        learning_rate: learning rate, to control the degree of Q-value update
        gamma0: constant value for ensuring loop-free routing
        tau: prediction horizon

    References:
        [1] B. Sliwa, et al.,"PARRoT: Predictive Ad-hoc Routing Fueled by Reinforcement Learning and Trajectory Knowledge,"
            in IEEE 93rd Vehicular Technology Conference (VTC2021-Spring), pp. 1-7, 2021.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/3/25
    Updated at: 2024/3/26
    """

    def __init__(self, env, my_drone):
        self.env = env
        self.my_drone = my_drone
        self.learning_rate = 0.5
        self.gamma0 = 0.8
        self.tau = 2.5
        self.max_comm_range = maximum_communication_range()

        self.q_table = np.zeros((config.NUMBER_OF_DRONES, config.NUMBER_OF_DRONES), dtype=object)
        # initialize the q-table
        for i in range(config.NUMBER_OF_DRONES):
            for j in range(config.NUMBER_OF_DRONES):
                self.q_table[i][j] = [0, 0]

    def update_table(self, chirp_packet, previous_drone_id, cur_time):
        """
        Update the Q-table according to the chirp packet when the packet is fresh
        :param chirp_packet: the received chirp packet
        :param previous_drone_id: the drone that transmits this chirp packet to me
        :param cur_time: the moment when the packet is received
        :return: None
        """

        packet_seq_num = chirp_packet.packet_id  # get the sequence number of the packet
        destination = chirp_packet.src_drone.identifier
        action = previous_drone_id

        reward = chirp_packet.reward
        cohesion = chirp_packet.cohesion

        link_expiry_time = link_lifetime_predictor(self.my_drone, self.my_drone.simulator.drones[action], self.max_comm_range)

        if link_expiry_time < self.tau:
            let = math.sqrt(link_expiry_time / self.tau)
        else:
            let = 1.0

        gamma = self.gamma0 * let * cohesion
        self.q_table[destination, action][1] = packet_seq_num  # update sequence number

        # update Q-value
        prev_q = self.q_table[destination, action][0]
        self.q_table[destination, action][0] = prev_q + self.learning_rate * (gamma * reward - prev_q)

        logging.info('Q table of UAV: %s is: %s', self.my_drone.identifier, self.q_table)
        # print('-----------------------')
        # print('Q table of UAV: ', self.my_drone.identifier)
        # print(self.q_table)

    def take_action(self, my_drone, dst_drone):
        dst_drone_id = dst_drone.identifier

        q_list = [self.q_table[dst_drone_id, _][0] for _ in range(config.NUMBER_OF_DRONES)]

        best_q_value = max(q_list)
        if best_q_value == 0:
            best_id = my_drone.identifier
        else:
            best_id = q_list.index(max(q_list))

        return best_id


def link_lifetime_predictor(drone1, drone2, max_comm_range):
    coords1 = drone1.coords
    coords2 = drone2.coords
    velocity1 = drone1.velocity
    velocity2 = drone2.velocity

    x1 = (velocity1[0] - velocity2[0]) ** 2
    x2 = (velocity1[1] - velocity2[1]) ** 2
    x3 = (velocity1[2] - velocity2[2]) ** 2

    y1 = 2 * (velocity1[0] - velocity2[0]) * (coords1[0] - coords2[0])
    y2 = 2 * (velocity1[1] - velocity2[1]) * (coords1[1] - coords2[1])
    y3 = 2 * (velocity1[2] - velocity2[2]) * (coords1[2] - coords2[2])

    z1 = (coords1[0] - coords2[0]) ** 2
    z2 = (coords1[1] - coords2[1]) ** 2
    z3 = (coords1[2] - coords2[2]) ** 2

    A = x1 + x2 + x3

    B = y1 + y2 + y3
    C = (z1 + z2 + z3) - max_comm_range ** 2

    delta_t_1 = (-B + math.sqrt(B ** 2 - 4 * A * C)) / (2 * A)
    delta_t_2 = (-B - math.sqrt(B ** 2 - 4 * A * C)) / (2 * A)

    delta_t = max(delta_t_1, delta_t_2)

    return delta_t
