import copy
import random
import logging
from entities.packet import DataPacket
from routing.q_routing.q_routing_packet import QRoutingHelloPacket, QRoutingAckPacket
from routing.q_routing.q_routing_table import QRoutingTable
from utils import config

# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=config.LOGGING_LEVEL
                    )


class QRouting:
    """
    Main procedure of Q-routing

    The core idea of this protocol is using Q-learning to approximate the end-to-end delay of packet transmission.
    The update of Q-values was implemented by ACK packet. It should be noted that the implementation in this project is
    sightly different from the original paper regrading to the calculation of transmission delay "s", because in highly
    dynamic network, the reply of ACK packet will fail due to many different factors.

    Calculation example:
    For a drone "x", it has a data packet bound for destination "d", after checking its Q-table, it selects its neighbor
    drone "y" with minimum Q-value Q_x(d, y) as the next hop. Then drone "x" transmits this data packet to "y". When "y"
    receives the data packet from "x", it will reply an ACK packet to "x", which carries serval information listed below:
    1. t = min Q_y(d, z): represents drone y's estimation for the time remaining in the trip.
    2. q: queuing delay in drone "x"

    When drone "x" receives the ACK packet from "y", it first calculates the real transmission delay "s" (Note that one
    cannot simply divide the data packet length by the bit rate, as re-transmission may also be included). Then drone
    "x" updates the Q-value of "y" using the following formula:

    Q_x(d, y) <-- (1 - a) * Q_x(d, y) + a * (q + s + t)

    In a nutshell, the protocol proposed in this paper can be considered as a kind of online learning protocol, where
    each drone can interact with its neighbors and updates its policy online. No replay buffer and each drone should
    maintain its own Q-table. However, if the topology of the network changes rapidly, the learning speed will be a
    problem. Besides, the update mechanism by ACK may be unsatisfactory in the situation of bad channel condition.

    Attributes:
        simulator: the simulation platform that contains everything
        my_drone: the drone that installed the GPSR
        hello_interval: interval of sending hello packet
        learning_rate: used to guide the degree to which the Q-value is updated
        table: including neighbor table and Q-table

    References:
        [1] J. Boyan and M. Littman, "Packet Routing in Dynamically Changing Networks: A Reinforcement Learning
            Approach," Advances in Neural Information Processing Systems (NIPS), no. 6, 1993.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/8/20
    Updated at: 2024/8/29

    """

    def __init__(self, simulator, my_drone):
        self.simulator = simulator
        self.my_drone = my_drone
        self.hello_interval = 0.5 * 1e6  # broadcast hello packet every 0.5s
        self.learning_rate = 0.5
        self.table = QRoutingTable(self.simulator.env, my_drone)
        self.simulator.env.process(self.broadcast_hello_packet_periodically())
        self.simulator.env.process(self.check_waiting_list())

    def broadcast_hello_packet(self, my_drone):
        config.GL_ID_HELLO_PACKET += 1
        hello_pkd = QRoutingHelloPacket(src_drone=my_drone,
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
        self.table.purge()

        dst_drone = packet.dst_drone

        # choose best next hop according to the neighbor table
        packet.intermediate_drones.append(self.my_drone.identifier)

        best_next_hop_id = self.table.best_neighbor(self.my_drone, dst_drone)

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
        if isinstance(packet, QRoutingHelloPacket):
            self.table.add_neighbor(packet, current_time)  # update the neighbor table

        elif isinstance(packet, DataPacket):
            packet_copy = copy.copy(packet)

            packet_copy.previous_drone = self.simulator.drones[src_drone_id]
            logging.info('~~~Packet: %s is received by UAV: %s at: %s',
                         packet_copy.packet_id, self.my_drone.identifier, self.simulator.env.now)

            if packet_copy.dst_drone.identifier == self.my_drone.identifier:
                latency = self.simulator.env.now - packet_copy.creation_time  # in us
                self.simulator.metrics.deliver_time_dict[packet_copy.packet_id] = latency
                self.simulator.metrics.throughput_dict[packet_copy.packet_id] = config.DATA_PACKET_LENGTH / (latency / 1e6)
                self.simulator.metrics.hop_cnt_dict[packet_copy.packet_id] = packet_copy.get_current_ttl()
                self.simulator.metrics.datapacket_arrived.add(packet_copy.packet_id)
                logging.info('Packet: %s is received by destination UAV: %s',
                             packet_copy.packet_id, self.my_drone.identifier)

                # waiting time includes queuing delay and access delay
                waiting_time = packet_copy.transmitting_start_time - packet_copy.waiting_start_time

                config.GL_ID_ACK_PACKET += 1
                src_drone = self.simulator.drones[src_drone_id]  # previous drone
                min_q = self.table.get_min_q_value(packet_copy.dst_drone.identifier)

                ack_packet = QRoutingAckPacket(src_drone=self.my_drone,
                                               dst_drone=src_drone,
                                               ack_packet_id=config.GL_ID_ACK_PACKET,
                                               ack_packet_length=config.ACK_PACKET_LENGTH,
                                               ack_packet=packet,
                                               transmitting_start_time=packet_copy.transmitting_start_time,
                                               queuing_delay=waiting_time,
                                               min_q=min_q,
                                               simulator=self.simulator)

                yield self.simulator.env.timeout(config.SIFS_DURATION)  # switch from receiving to transmitting

                # unicast the ack packet immediately without contention for the channel
                if not self.my_drone.sleep:
                    ack_packet.increase_ttl()
                    self.my_drone.mac_protocol.phy.unicast(ack_packet, src_drone_id)
                    yield self.simulator.env.timeout(ack_packet.packet_length / config.BIT_RATE * 1e6)
                    self.simulator.drones[src_drone_id].receive()
                else:
                    pass
            else:
                if self.my_drone.transmitting_queue.qsize() < self.my_drone.max_queue_size:
                    self.my_drone.transmitting_queue.put(packet_copy)
                    packet_copy.waiting_start_time = self.simulator.env.now  # this packet starts to wait in the queue

                    # waiting time includes queuing delay and access delay
                    waiting_time = packet_copy.transmitting_start_time - packet_copy.waiting_start_time

                    config.GL_ID_ACK_PACKET += 1
                    src_drone = self.simulator.drones[src_drone_id]  # previous drone
                    min_q = self.table.get_min_q_value(packet_copy.dst_drone.identifier)

                    ack_packet = QRoutingAckPacket(src_drone=self.my_drone,
                                                   dst_drone=src_drone,
                                                   ack_packet_id=config.GL_ID_ACK_PACKET,
                                                   ack_packet_length=config.ACK_PACKET_LENGTH,
                                                   ack_packet=packet,
                                                   transmitting_start_time=packet_copy.transmitting_start_time,
                                                   queuing_delay=waiting_time,
                                                   min_q=min_q,
                                                   simulator=self.simulator)

                    yield self.simulator.env.timeout(config.SIFS_DURATION)  # switch from receiving to transmitting

                    # unicast the ack packet immediately without contention for the channel
                    if not self.my_drone.sleep:
                        ack_packet.increase_ttl()
                        self.my_drone.mac_protocol.phy.unicast(ack_packet, src_drone_id)
                        yield self.simulator.env.timeout(ack_packet.packet_length / config.BIT_RATE * 1e6)
                        self.simulator.drones[src_drone_id].receive()
                    else:
                        pass
                else:
                    pass

        elif isinstance(packet, QRoutingAckPacket):
            data_packet_acked = packet.ack_packet
            # update Q-table
            self.update_q_table(packet, src_drone_id)

            self.my_drone.remove_from_queue(data_packet_acked)

            key2 = 'wait_ack' + str(self.my_drone.identifier) + '_' + str(data_packet_acked.packet_id)

            if self.my_drone.mac_protocol.wait_ack_process_finish[key2] == 0:
                if not self.my_drone.mac_protocol.wait_ack_process_dict[key2].triggered:
                    logging.info('At time: %s, the wait_ack process (id: %s) of UAV: %s is interrupted by UAV: %s',
                                 self.simulator.env.now, key2, self.my_drone.identifier, src_drone_id)

                    self.my_drone.mac_protocol.wait_ack_process_finish[key2] = 1  # mark it as "finished"
                    self.my_drone.mac_protocol.wait_ack_process_dict[key2].interrupt()

    def update_q_table(self, packet, next_hop_id):
        data_packet_acked = packet.ack_packet
        dst_drone = data_packet_acked.dst_drone

        waiting_time = packet.queuing_delay

        transmitting_start_time = data_packet_acked.transmitting_start_time
        transmission_delay = self.simulator.env.now - transmitting_start_time  # in us

        self.simulator.metrics.mac_delay.append((self.simulator.env.now - data_packet_acked.backoff_start_time) / 1e3)

        logging.info('Data packet id: %s, real transmission delay is: %s',
                     data_packet_acked.packet_id, transmission_delay)

        min_q = packet.min_q

        # calculate reward function
        if next_hop_id == dst_drone.identifier:
            f = 1
        else:
            f = 0

        self.table.q_table[next_hop_id][dst_drone.identifier] = \
            (1 - self.learning_rate) * self.table.q_table[next_hop_id][dst_drone.identifier] + \
            self.learning_rate * (waiting_time + transmission_delay + (1 - f) * min_q)

        logging.info('The Q-table in UAV: %s is: %s',
                     self.my_drone.identifier, self.table.q_table)

    def check_waiting_list(self):
        while True:
            if not self.my_drone.sleep:
                yield self.simulator.env.timeout(0.6 * 1e6)
                for waiting_pkd in self.my_drone.waiting_list:
                    if self.simulator.env.now < waiting_pkd.creation_time + waiting_pkd.deadline:
                        self.my_drone.waiting_list.remove(waiting_pkd)
                    else:
                        dst_drone = waiting_pkd.dst_drone
                        best_next_hop_id = self.table.best_neighbor(self.my_drone, dst_drone)
                        if best_next_hop_id != self.my_drone.identifier:
                            self.my_drone.transmitting_queue.put(waiting_pkd)
                            self.my_drone.waiting_list.remove(waiting_pkd)
                        else:
                            pass
            else:
                break
