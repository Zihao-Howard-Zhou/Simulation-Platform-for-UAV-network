from entities.packet import Packet


class ChirpPacket(Packet):
    """
    Chirp packet of PARRoT

    Attributes:
        src_drone: the source drone that generates this chirp packet
        creation_time: the time when this packet was generated
        id_chirp_packet: to identify the chirp packet uniquely (served as sequence number)
        chirp_packet_length: 40 Byte in [1]
        current_position:
        predicted_position:
        reward: when the node (A) originates the chirp packet, the reward to itself is set to 1.0, when a node further
                forwards the chirp packet, the reward is set to the maximum Q-value to the destination (A). It should be
                noted that each chirp packet is corresponding with a certain destination node
        cohesion: each node regularly calculates the changes in the number of its neighbors, when it initiates or
                  forward a chirp packet, it fills in its latest calculated cohesion
        simulator: the simulation platform that contains everything

    References:
        [1] B. Sliwa, et al.,"PARRoT: Predictive Ad-hoc Routing Fueled by Reinforcement Learning and Trajectory Knowledge,"
            in IEEE 93rd Vehicular Technology Conference (VTC2021-Spring), pp. 1-7, 2021.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/3/25
    Updated at: 2024/3/25
    """

    def __init__(self,
                 src_drone,
                 creation_time,
                 id_chirp_packet,
                 current_position,
                 predicted_position,
                 reward,
                 cohesion,
                 simulator,
                 chirp_packet_length=320):
        super().__init__(id_chirp_packet, chirp_packet_length, creation_time, simulator)

        self.src_drone = src_drone
        self.current_position = current_position
        self.predicted_position = predicted_position
        self.reward = reward
        self.cohesion = cohesion
