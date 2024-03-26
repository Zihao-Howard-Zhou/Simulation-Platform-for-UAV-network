import random
import logging
from entities.packet import DataPacket, AckPacket
from routing.parrot.parrot_packet import ChirpPacket
from routing.parrot.q_table import Qtable
from routing.parrot.parrot_neighbor_table import ParrotNeighborTable
from utils import config

# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=logging.INFO
                    )

GL_ID_CHIRP_PACKET = 5000
GL_ID_ACK_PACKET = 10000


class Parrot:
    """
    Main procedure of PARRoT (v1.0)

    Attributes:
        simulator: the simulation platform that contains everything
        my_drone: the drone that installed the PARRoT


    References:
        [1] B. Sliwa, et al.,"PARRoT: Predictive Ad-hoc Routing Fueled by Reinforcement Learning and Trajectory Knowledge,"
            in IEEE 93rd Vehicular Technology Conference (VTC2021-Spring), pp. 1-7, 2021.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/3/25
    Updated at: 2024/3/26
    """

    def __init__(self, simulator, my_drone):
        self.simulator = simulator
        self.my_drone = my_drone
        self.chirp_interval = 0.5 * 1e6  # broadcast chirp packet every 0.5s
        self.qtable = Qtable(self.simulator.env, my_drone)
        self.neighbor_table = ParrotNeighborTable(self.simulator.env, my_drone)
        self.simulator.env.process(self.broadcast_chirp_packet_periodically())

    def broadcast_chirp_packet(self, my_drone):
        global GL_ID_CHIRP_PACKET

        GL_ID_CHIRP_PACKET += 1

        # 要先做轨迹预测+获取凝聚力值
        cohesion = self.neighbor_table.cohesion

        # if a certain drone originates a chirp packet, then in the views of other drones, it will be regarded as
        # destination, so the reward item in chirp_packet of the initiator is set to 1.0 in this stage
        chirp_packet = ChirpPacket(src_drone=my_drone, creation_time=self.simulator.env.now,
                                   id_chirp_packet=GL_ID_CHIRP_PACKET,
                                   current_position=self.my_drone.coords, predicted_position=0,
                                   reward=1.0, cohesion=cohesion,
                                   simulator=self.simulator)

        logging.info('At time: %s, UAV: %s has chirp packet to broadcast',
                     self.simulator.env.now, self.my_drone.identifier)

        yield self.simulator.env.process(my_drone.packet_coming(chirp_packet, 1))

    def broadcast_chirp_packet_periodically(self):
        while True:
            self.simulator.env.process(self.broadcast_chirp_packet(self.my_drone))
            jitter = random.randint(1000, 2000)  # delay jitter
            yield self.simulator.env.timeout(self.chirp_interval+jitter)

    def next_hop_selection(self, packet):
        """
        Select the next hop according to the routing protocol
        :param packet: the data packet that needs to be sent
        :return: id of next hop drone
        """

        dst_drone = packet.dst_drone
        best_next_hop_id = self.qtable.take_action(self.my_drone, dst_drone)

        return best_next_hop_id

    def packet_reception(self, packet, src_drone_id):
        """
        Packet reception at network layer

        since different routing protocols have their own corresponding packets, it is necessary to add this packet
        reception function in the network layer
        :param packet: the received packet
        :param src_drone_id: previous hop
        :return: None
        """

        global GL_ID_ACK_PACKET

        current_time = self.simulator.env.now

        if isinstance(packet, ChirpPacket):
            self.neighbor_table.add_neighbor(packet, current_time)  # update neighbor table

            packet_seq_num = packet.packet_id  # get the sequence number of the packet
            destination = packet.src_drone  # drone that originates the chirp packet

            latest_seq_num = max([self.qtable.q_table[destination.identifier, _][1] for _ in range(config.NUMBER_OF_DRONES)])
            if latest_seq_num >= packet_seq_num or self.my_drone.identifier == src_drone_id:
                pass
            else:
                self.qtable.update_table(packet, src_drone_id, current_time)

                reward = max([self.qtable.q_table[destination.identifier, _][0] for _ in range(self.simulator.n_drones)])
                cohesion = self.neighbor_table.cohesion

                logging.info('At time: %s, UAV: %s receives the CHIRP packet from UAV: %s, the reward is: %s, and the'
                             'cohesion is: %s', current_time, self.my_drone.identifier, src_drone_id, reward, cohesion)

                # continue to flood the chirp packet
                chirp_packet = ChirpPacket(src_drone=self.my_drone, creation_time=self.simulator.env.now,
                                           id_chirp_packet=packet_seq_num, current_position=self.my_drone.coords,
                                           predicted_position=0, reward=reward,
                                           cohesion=cohesion, simulator=self.simulator)

                self.my_drone.fifo_queue.put([chirp_packet, 1])

        elif isinstance(packet, DataPacket):
            if packet.dst_drone.identifier == self.my_drone.identifier:
                self.simulator.metrics.deliver_time_dict[packet.packet_id] = self.simulator.env.now - packet.creation_time
                self.simulator.metrics.datapacket_arrived.add(packet.packet_id)
                logging.info('Packet: %s is received by destination UAV: %s', packet.packet_id, self.my_drone.identifier)
            else:
                self.my_drone.fifo_queue.put([packet, 0])

            GL_ID_ACK_PACKET += 1
            src_drone = self.simulator.drones[src_drone_id]  # previous drone
            ack_packet = AckPacket(src_drone=self.my_drone, dst_drone=src_drone, ack_packet_id=GL_ID_ACK_PACKET,
                                   ack_packet_length=config.ACK_PACKET_LENGTH,
                                   ack_packet=packet, simulator=self.simulator)

            yield self.simulator.env.timeout(config.SIFS_DURATION)  # switch from receiving to transmitting

            # unicast the ack packet immediately without contention for the channel
            yield self.simulator.env.process(self.my_drone.mac_protocol.phy.unicast(ack_packet, src_drone_id))

        elif isinstance(packet, AckPacket):
            key2 = str(self.my_drone.identifier) + '_' + str(self.my_drone.mac_protocol.wait_ack_process_count)

            if self.my_drone.mac_protocol.wait_ack_process_finish[key2] == 0:
                logging.info('At time: %s, the wait_ack process (id: %s) of UAV: %s is interrupted by UAV: %s',
                             self.simulator.env.now, key2, self.my_drone.identifier, src_drone_id)
                self.my_drone.mac_protocol.wait_ack_process_dict[key2].interrupt()
