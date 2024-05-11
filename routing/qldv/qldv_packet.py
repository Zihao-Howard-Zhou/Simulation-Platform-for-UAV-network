from entities.packet import Packet


class QldvHelloPacket(Packet):
    """
    The format of "update_material" of drone "i":
    [(dst_id 1, max_q for "dst_id 1", action for "max_q"),
     (dst_id 2, max_q for "dst_id 2", action for "max_q"),
     ...
     (dst_id n, max_q for "dst_id n", action for "max_q")]

    When node "j" receiving hello packet, "update_material" field will be extracted for updating Q-value.
    Take dst 1 as example, if the "action for max_q" is not equal to "j" (it means that this path is not a loop),
    then we have: Q_j(1, i) = (1 - a) * Q_j(1, i) + a * [r_ij + y * (1 - f) * max_q]
    otherwise, Q_j(1, j) will not be updated.
    """

    def __init__(self,
                 src_drone,
                 creation_time,
                 id_hello_packet,
                 hello_packet_length,
                 update_material,
                 simulator):
        super().__init__(id_hello_packet, hello_packet_length, creation_time, simulator)

        self.src_drone = src_drone
        self.cur_position = src_drone.coords
        self.cur_velocity = src_drone.velocity
        self.update_material = update_material


class QldvErrorPacket(Packet):
    """
    The format of "error_update_material":

    """

    def __init__(self,
                 src_drone,
                 creation_time,
                 id_hello_packet,
                 hello_packet_length,
                 error_update_material,
                 simulator):
        super().__init__(id_hello_packet, hello_packet_length, creation_time, simulator)

        self.src_drone = src_drone
        self.error_update_material = error_update_material
