from utils import config

class Packet:
    """
    Basic properties of the packet

    all other packets need to inherit this class

    Attributes:
        packet_id: identifier of the packet, used to uniquely represent a packet
        creation_time: the generation time of the packet
        deadline:
        __ttl: current "Time to live (TTL)"
        __max_ttl: maximum value of TTL
        time_delivery: the time at which the packet arrives at its destination

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2024/1/16
    """

    def __init__(self,
                 packet_id,
                 packet_length,
                 creation_time,
                 simulator):

        self.packet_id = packet_id
        self.packet_length = packet_length
        self.creation_time = creation_time
        # self.deadline = creation_time + config.PACKET_LIFETIME
        self.simulator = simulator
        self.__ttl = 0
        # self.__max_ttl = config.MAX_TTL

        self.number_retransmission_attempt = {}

        for drone in self.simulator.drones:
            self.number_retransmission_attempt[drone.identifier] = 0

        self.time_delivery = None

    def increase_ttl(self):
        self.__ttl += 1

    def get_current_ttl(self):
        return self.__ttl

    # def is_expired(self, current_time):
    #     return current_time > self.deadline


class DataPacket(Packet):
    """
    Basic properties of the data packet

    Attributes:
        src_drone: source drone that originates the data packet
        dst_drone: destination drone of this data packet
        creation_time: the generation time of the packet
        data_packet_id: identifier of the packet, used to uniquely represent a packet

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2024/1/16
    """

    def __init__(self,
                 src_drone,
                 dst_drone,
                 creation_time,
                 data_packet_id,
                 data_packet_length,
                 simulator):
        super().__init__(data_packet_id, data_packet_length, creation_time, simulator)

        self.src_drone = src_drone
        self.dst_drone = dst_drone


class AckPacket(Packet):
    def __init__(self,
                 src_drone,
                 dst_drone,
                 ack_packet_id,
                 ack_packet_length,
                 ack_packet,
                 simulator,
                 creation_time=None):
        super().__init__(ack_packet_id, ack_packet_length, creation_time, simulator)

        self.src_drone = src_drone
        self.dst_drone = dst_drone

        self.ack_packet = ack_packet
