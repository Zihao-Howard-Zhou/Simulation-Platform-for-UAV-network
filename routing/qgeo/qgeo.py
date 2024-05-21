import copy
import random
import logging
from entities.packet import DataPacket
from routing.qgeo.qgeo_packet import QGeoHelloPacket, QGeoAckPacket
from topology.virtual_force.vf_packet import VfPacket
from routing.qgeo.qgeo_neighbor_table import QGeoNeighborTable
from utils.util_function import euclidean_distance
from phy.large_scale_fading import maximum_communication_range
from utils import config

# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=config.LOGGING_LEVEL
                    )


class QGeo:
    """
    QGeo (Q-Learning based Geographic routing protocol) v1.0

    It should be noted that this version has been modified based on the original paper for the following reason:
    1. On reward function setting in the original paper, channel access overhead with re-transmission was considered,
       however, we argue that in actual situations, ACK packet are essential to obtain accurate mac delay. Although
       we can get rough mac delay according to the link error rate (LER), LER is often difficult to obtain, which is
       related to SINR.

    References:
        [1] Jung W S, Yim J, Ko Y B., "QGeo: Q-learning-based geographic ad hoc routing protocol for unmanned robotic
            networks," IEEE Communications Letters, 21(10): 2258-2261, 2017.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/4/17
    Updated at: 2024/4/17
    """

    def __init__(self, simulator, my_drone):
        self.simulator = simulator
        self.my_drone = my_drone
        self.hello_interval = 0.5 * 1e6  # broadcast hello packet every 0.5s
        self.learning_rate = 0.5
        self.neighbor_table = QGeoNeighborTable(self.simulator.env, my_drone)
        self.simulator.env.process(self.broadcast_hello_packet_periodically())
        self.simulator.env.process(self.check_waiting_list())

    def broadcast_hello_packet(self, my_drone):
        config.GL_ID_HELLO_PACKET += 1
        hello_pkd = QGeoHelloPacket(src_drone=my_drone,
                                    creation_time=self.simulator.env.now,
                                    id_hello_packet=config.GL_ID_HELLO_PACKET,
                                    hello_packet_length=config.HELLO_PACKET_LENGTH,
                                    simulator=self.simulator)
        hello_pkd.transmission_mode = 1

        logging.info('At time: %s, UAV: %s has hello packet to broadcast',
                     self.simulator.env.now, self.my_drone.identifier)

        self.simulator.metrics.control_packet_num += 1
        self.my_drone.transmitting_queue.put(hello_pkd)

    def broadcast_hello_packet_periodically(self):
        while True:
            self.broadcast_hello_packet(self.my_drone)
            jitter = random.randint(1000, 2000)  # delay jitter
            yield self.simulator.env.timeout(self.hello_interval + jitter)

    def next_hop_selection(self, packet):
        """
        Select the next hop according to the routing protocol
        :param packet: the data packet that needs to be sent
        :return: next hop drone
        """
        enquire = False
        has_route = True

        # update neighbor table
        self.neighbor_table.purge()

        dst_drone = packet.dst_drone

        # choose best next hop according to the neighbor table
        best_next_hop_id = self.neighbor_table.best_neighbor(self.my_drone, dst_drone)

        if best_next_hop_id is self.my_drone.identifier:
            has_route = False  # no available next hop
        else:
            packet.next_hop_id = best_next_hop_id  # it has an available next hop drone

        return has_route, packet, enquire

    def packet_reception(self, packet, src_drone_id):
        """
        Packet reception at network layer

        since different routing protocols have their own corresponding packets, it is necessary to add this packet
        reception function in the network layer
        :param packet: the received packet
        :param src_drone_id: previous hop
        :return: none
        """

        current_time = self.simulator.env.now
        if isinstance(packet, QGeoHelloPacket):
            self.neighbor_table.add_neighbor(packet, current_time)  # update the neighbor table
            # self.neighbor_table.print_neighbor(self.my_drone)

        elif isinstance(packet, DataPacket):
            packet_copy = copy.copy(packet)
            if packet_copy.dst_drone.identifier == self.my_drone.identifier:
                self.simulator.metrics.deliver_time_dict[packet_copy.packet_id] \
                    = self.simulator.env.now - packet_copy.creation_time
                self.simulator.metrics.datapacket_arrived.add(packet_copy.packet_id)
                logging.info('Packet: %s is received by destination UAV: %s',
                             packet_copy.packet_id, self.my_drone.identifier)
            else:
                self.my_drone.transmitting_queue.put(packet_copy)

            config.GL_ID_ACK_PACKET += 1
            src_drone = self.simulator.drones[src_drone_id]  # previous drone
            max_q = self.neighbor_table.get_max_q_value(packet_copy.dst_drone.identifier)
            void_area = self.neighbor_table.judge_void_area(packet_copy.dst_drone)  # determine if next hop is local minimum
            ack_packet = QGeoAckPacket(src_drone=self.my_drone,
                                       dst_drone=src_drone,
                                       ack_packet_id=config.GL_ID_ACK_PACKET,
                                       ack_packet_length=config.ACK_PACKET_LENGTH,
                                       max_q=max_q,
                                       void_area=void_area,
                                       ack_packet=packet,
                                       simulator=self.simulator)

            yield self.simulator.env.timeout(config.SIFS_DURATION)  # switch from receiving to transmitting

            # unicast the ack packet immediately without contention for the channel
            if not self.my_drone.sleep:
                self.my_drone.mac_protocol.phy.unicast(ack_packet, src_drone_id)
                yield self.simulator.env.timeout(ack_packet.packet_length / config.BIT_RATE * 1e6)
            else:
                pass

        elif isinstance(packet, QGeoAckPacket):
            # update Q-table
            self.update_q_table(packet, src_drone_id)

            key2 = str(self.my_drone.identifier) + '_' + str(self.my_drone.mac_protocol.wait_ack_process_count)

            if self.my_drone.mac_protocol.wait_ack_process_finish[key2] == 0:
                if not self.my_drone.mac_protocol.wait_ack_process_dict[key2].triggered:
                    logging.info('At time: %s, the wait_ack process (id: %s) of UAV: %s is interrupted by UAV: %s',
                                 self.simulator.env.now, key2, self.my_drone.identifier, src_drone_id)
                    self.my_drone.mac_protocol.wait_ack_process_dict[key2].interrupt()

        elif isinstance(packet, VfPacket):
            logging.info('At time %s, UAV: %s receives the vf hello msg from UAV: %s, pkd id is: %s',
                         self.simulator.env.now, self.my_drone.identifier, src_drone_id, packet.packet_id)

            # update the neighbor table
            self.my_drone.motion_controller.neighbor_table.add_neighbor(packet, current_time)

            if packet.msg_type == 'hello':
                config.GL_ID_VF_PACKET += 1
                ack_packet = VfPacket(src_drone=self.my_drone,
                                      creation_time=self.simulator.env.now,
                                      id_hello_packet=config.GL_ID_VF_PACKET,
                                      hello_packet_length=config.HELLO_PACKET_LENGTH,
                                      simulator=self.simulator)
                ack_packet.msg_type = 'ack'

                self.my_drone.transmitting_queue.put(ack_packet)
            else:
                pass

    def update_q_table(self, packet, next_hop_id):
        # next hello interval
        future_time = (self.hello_interval * (self.simulator.env.now / self.hello_interval + 1)) / 1e6

        cur_pos_myself = list(self.my_drone.coords)
        cur_pos_next_hop = list(self.neighbor_table.get_neighbor_position(next_hop_id))

        cur_vel_myself = list(self.my_drone.velocity)
        cur_vel_next_hop = list(self.neighbor_table.get_neighbor_velocity(next_hop_id))

        update_time_next_hop = self.neighbor_table.get_updated_time(next_hop_id) / 1e6
        time_interval = [future_time - update_time_next_hop] * 3  # three dimensions

        position_shift_next_hop = [v * t for v, t in zip(cur_vel_next_hop, time_interval)]
        position_shift_myself = [v * t for v, t in zip(cur_vel_myself, time_interval)]

        future_pos_myself = cur_pos_myself + position_shift_myself
        future_pos_next_hop = cur_pos_next_hop + position_shift_next_hop

        future_distance = euclidean_distance(future_pos_myself, future_pos_next_hop)

        # calculate discounted factor
        if future_distance < maximum_communication_range():
            gamma = 0.6
        else:
            gamma = 0.4

        next_hop_drone = self.simulator.drones[next_hop_id]
        data_packet_acked = packet.ack_packet
        dst_drone = data_packet_acked.dst_drone
        myself_to_dst = euclidean_distance(dst_drone.coords, self.my_drone.coords)
        next_hop_to_dst = euclidean_distance(dst_drone.coords, next_hop_drone.coords)
        dist_difference = myself_to_dst - next_hop_to_dst

        mac_delay = (self.simulator.env.now - data_packet_acked.time_transmitted_at_last_hop) / 1e6
        max_q = packet.max_q
        void_area = packet.void_area

        # normalization factor
        f = maximum_communication_range() / mac_delay

        # calculate reward function
        if next_hop_drone is dst_drone:
            reward = 1
        elif void_area == 1:
            reward = -1
        else:
            reward = (dist_difference / mac_delay) / f

        self.neighbor_table.q_table[next_hop_id][dst_drone.identifier] = \
            (1 - self.learning_rate) * self.neighbor_table.q_table[next_hop_id][dst_drone.identifier] + \
            self.learning_rate * (reward + gamma * max_q)

    def check_waiting_list(self):
        while True:
            if not self.my_drone.sleep:
                yield self.simulator.env.timeout(0.6 * 1e6)
                for waiting_pkd in self.my_drone.waiting_list:
                    if self.simulator.env.now < waiting_pkd.creation_time + waiting_pkd.deadline:
                        self.my_drone.waiting_list.remove(waiting_pkd)
                    else:
                        dst_drone = waiting_pkd.dst_drone
                        best_next_hop_id = self.neighbor_table.best_neighbor(self.my_drone, dst_drone)
                        if best_next_hop_id != self.my_drone.identifier:
                            self.my_drone.transmitting_queue.put(waiting_pkd)
                            self.my_drone.waiting_list.remove(waiting_pkd)
                        else:
                            pass
            else:
                break
