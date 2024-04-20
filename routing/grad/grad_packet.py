from entities.packet import Packet


class GradMessage(Packet):
    def __init__(self,
                 src_drone,
                 dst_drone,
                 creation_time,
                 id_message,
                 message_length,
                 message_type,
                 accrued_cost,
                 remaining_value,
                 simulator):
        super().__init__(id_message, message_length, creation_time, simulator)

        self.msg_type = message_type
        self.originator = src_drone  # it should be noted that relaying node cannot change this field!
        self.seq_num = id_message
        self.target = dst_drone
        self.accrued_cost = accrued_cost
        self.remaining_value = remaining_value

        self.attached_data_packet = None  # when msg_type is "M_DATA"
