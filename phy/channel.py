import logging
from collections import defaultdict


class Channel:
    """
    Wireless channel of the physical layer

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2024/4/23
    """

    def __init__(self, env):
        self.env = env
        self.pipes = defaultdict(list)

    def broadcast_put(self, value):
        """
        Broadcast support
        :param value: packet that needs to broadcast
        :return: none
        """

        if not self.pipes:
            logging.error('No inboxes available!')

        # the sender "puts" packets to all inboxes in pipes separately
        for inbox in self.pipes.values():
            inbox.append(value)

    def unicast_put(self, value, dst_id):
        """
        Unicast support
        :param value: packet that needs to unicast
        :param dst_id: next hop id for transmitting this packet
        :return: none
        """

        if dst_id not in self.pipes.keys():
            logging.error('There is no inbox for dst_id')

        self.pipes[dst_id].append(value)

    def multicast_put(self, value, dst_id_list):
        """
        Multicast support
        :param value: packet that needs to multicast
        :param dst_id_list: next hop list
        :return: none
        """

        for dst_id in dst_id_list:
            if not self.pipes[dst_id]:
                logging.error('There is no inbox for dst_id')
            else:
                self.pipes[dst_id].append(value)

    def create_inbox_for_receiver(self, identifier):
        # each receiver needs a list as its inbox
        pipe = []
        self.pipes[identifier] = pipe
        return pipe
