from utils import config


class Packet:
    """
    Basic properties of the packet

    all other packets need to inherit this class

    Attributes:
        packet_id: identifier of the packet, used to uniquely represent a packet
        creation_time: the generation time of the packet
        deadline: maximum segment lifetime of packet, in second
        __ttl: current "Time to live (TTL)"
        number_retransmission_attempt: record the number of retransmissions of packet on different drones
        waiting_start_time: the time at which tha packet is added to the "transmitting queue" of drone
        backoff_start_time: the time at which the packet starts the backoff stage
        transmitting_start_time: the time at which the packet can be transmitted to the channel after backoff
        time_delivery: the time at which the packet arrives at its destination
        time_transmitted_at_last_hop: the transmitting time at last drone
        transmission_mode: unicast or multicast or broadcast?

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2024/8/29
    """

    def __init__(self,
                 packet_id,
                 packet_length,
                 creation_time,
                 simulator):

        self.packet_id = packet_id
        self.packet_length = packet_length
        self.creation_time = creation_time
        self.deadline = config.PACKET_LIFETIME
        self.simulator = simulator
        self.__ttl = 0

        self.number_retransmission_attempt = {}

        for drone in self.simulator.drones:
            self.number_retransmission_attempt[drone.identifier] = 0  # initialization

        # for calculating the queuing delay
        self.waiting_start_time = None
        self.backoff_start_time = None
        self.transmitting_start_time = None

        self.time_delivery = None  # for end-to-end delay
        self.time_transmitted_at_last_hop = 0
        self.transmission_mode = None

        self.intermediate_drones = []

    def increase_ttl(self):
        self.__ttl += 1

    def get_current_ttl(self):
        return self.__ttl


class DataPacket(Packet):
    """
    Basic properties of the data packet

    Attributes:
        src_drone: source drone that originates the data packet
        dst_drone: destination drone of this data packet
        routing_path: record to whole routing path in centralized routing protocol
        next_hop_id: identifier of the next hop drone

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2024/5/4
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

        self.routing_path = None  # for centralized routing protocols
        self.next_hop_id = None  # next hop for this data packet


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
