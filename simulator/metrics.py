import numpy as np
from collections import defaultdict


class Metrics:
    """
    Tools for statistics of network performance

    1. Packet Delivery Ratio (PDR): is the ratio of number of packets received at the destinations to the number
       of packets sent from the sources.
    2. Average end-to-end (E2E) delay: is the time a packet takes to route from a source to its destination through
       the network. It is the time the data packet reaches the destination minus the time the data packet was generated
       in the source node.
    3. Routing Load: is calculated as the ratio between the numbers of control Packets transmitted
       to the number of packets actually received. NRL can reflect the average number of control packets required to
       successfully transmit a data packet and reflect the efficiency of the routing protocol.

    References:
        [1] Rani. N, Sharma. P, Sharma. P., "Performance Comparison of Various Routing Protocols in Different Mobility
            Models," in arXiv preprint arXiv:1209.5507, 2012.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2024/4/15
    """

    def __init__(self, simulator):
        self.simulator = simulator

        self.control_packet_num = 0

        self.datapacket_generated = set()  # all data packets generated
        self.datapacket_arrived = set()  # all data packets that arrives the destination
        self.datapacket_generated_num = 0

        self.delivery_time = []
        self.deliver_time_dict = defaultdict()

        self.collision_num = 0

    def print_metrics(self):
        # calculate the average end-to-end delay
        for key in self.deliver_time_dict.keys():
            self.delivery_time.append(self.deliver_time_dict[key])

        e2e_delay = np.mean(self.delivery_time) / 1e3  # unit: ms

        pdr = len(self.datapacket_arrived) / self.datapacket_generated_num * 100

        rl = self.control_packet_num / len(self.datapacket_arrived)

        print('Total send: ', self.datapacket_generated_num)
        print('PDR is: ', pdr, ' %')
        print('Average end-to-end delay is: ', e2e_delay, ' ms')
        print('Routing load is: ', rl)
        print('Collisiion num is: ', self.collision_num)
