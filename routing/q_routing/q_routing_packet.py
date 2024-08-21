from entities.packet import Packet


class QRoutingHelloPacket(Packet):
    def __init__(self,
                 src_drone,
                 creation_time,
                 id_hello_packet,
                 hello_packet_length,
                 simulator):
        super().__init__(id_hello_packet, hello_packet_length, creation_time, simulator)

        self.src_drone = src_drone
        self.cur_position = src_drone.coords


class QRoutingAckPacket(Packet):
    def __init__(self,
                 src_drone,
                 dst_drone,
                 ack_packet_id,
                 ack_packet_length,
                 ack_packet,
                 transmitting_start_time,
                 queuing_delay,
                 min_q,
                 simulator,
                 creation_time=None
                 ):
        super().__init__(ack_packet_id, ack_packet_length, creation_time, simulator)

        self.src_drone = src_drone
        self.dst_drone = dst_drone

        self.ack_packet = ack_packet
        self.transmitting_start_time = transmitting_start_time
        self.queuing_delay = queuing_delay
        self.min_q = min_q
