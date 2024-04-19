import logging
import simpy
import simpy.core


class Channel:
    """
    Wireless channel of the physical layer

    sender transmits packets through the wireless channel
    the propagation delay of the channel is negligible since electromagnetic waves travel at the speed of light
    packet loss due to signal attenuation or interference will be added in the future work

    Attributes:
        env: simulation environment created by simpy
        capacity: maximum number of packets that a single store can hold
        pipes: list, contain store for each receiver

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2024/4/18
    """

    def __init__(self, env, capacity=simpy.core.Infinity):
        self.env = env
        self.capacity = capacity
        self.pipes = []

    def broadcast_put(self, value):
        """
        Broadcast support
        :param value: packet that needs to broadcast
        :return:
        """

        if not self.pipes:
            logging.error('Pipes does not have any stores')

        # the sender "puts" packets to all stores in pipes separately
        events = [store.put(value) for store in self.pipes]
        return self.env.all_of(events)

    def unicast_put(self, value, dst_id):
        """
        Unicast support
        :param value: packet that needs to unicast
        :param dst_id: next hop id for transmitting this packet
        :return:
        """

        if not self.pipes[dst_id]:
            logging.error('There is no store for dst_id')

        self.pipes[dst_id].put(value)

    def multicast_put(self, value, dst_id_list):
        """
        Multicast support
        :param value: packet that needs to multicast
        :param dst_id_list: next hop list
        :return:
        """

        for dst_id in dst_id_list:
            if not self.pipes[dst_id]:
                logging.error('There is no store for dst_id')
            else:
                self.pipes[dst_id].put(value)

    def create_store_for_receiver(self):
        # each receiver needs a store
        pipe = simpy.Store(self.env)
        self.pipes.append(pipe)
        return pipe
