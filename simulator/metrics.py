import numpy as np
from collections import defaultdict

class Metrics:
    def __init__(self, simulator):
        self.simulator = simulator

        self.datapacket_generated = set()  # all data packets generated
        self.datapacket_arrived = set()  # all data packets that arrives the destination
        self.datapacket_generated_num = 0

        self.delivery_time = []
        self.deliver_time_dict = defaultdict()

    def print_metrics(self):
        number_packets_arrived = len(self.delivery_time)
        number_packets_unarrived = self.datapacket_generated_num - number_packets_arrived

        # calculate the average end-to-end delay
        for key in self.deliver_time_dict.keys():
            self.delivery_time.append(self.deliver_time_dict[key])

        # delivery_time_unarrived = [100 for _ in range(number_packets_unarrived)]
        e2e_delay = np.mean(self.delivery_time) / 1e3  # unit: ms

        pdr = len(self.datapacket_arrived) / self.datapacket_generated_num * 100

        print('Total send: ', self.datapacket_generated_num)
        print('PDR is: ', pdr, ' %')
        print('Average end-to-end delay is: ', e2e_delay, ' ms')
