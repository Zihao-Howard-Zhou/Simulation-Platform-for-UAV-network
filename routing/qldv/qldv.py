import copy
import random
import logging
from entities.packet import DataPacket, AckPacket
from routing.qldv.qldv_table import QldvTable
from routing.qldv.qldv_packet import QldvHelloPacket, QldvErrorPacket
from utils import config

# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=config.LOGGING_LEVEL
                    )

GL_ID_HELLO_PACKET = 5000
GL_ID_ACK_PACKET = 10000
GL_ID_ERROR_PACKET = 20000


class Qldv:
    """

    """

    def __init__(self, simulator, my_drone):
        self.simulator = simulator
        self.my_drone = my_drone
        self.hello_interval = 0.1 * 1e6  # broadcast hello packet every 0.1s
        self.table = QldvTable(self.simulator.env, my_drone)  # includes neighbor table and Q-table
        self.processed_error_pkd = []
        self.simulator.env.process(self.broadcast_hello_packet_periodically())
        self.simulator.env.process(self.purge_periodically())
        self.simulator.env.process(self.check_waiting_list())

    def broadcast_hello_packet(self, my_drone):
        global GL_ID_HELLO_PACKET

        GL_ID_HELLO_PACKET += 1
        update_material = self.table.get_update_material()
        hello_pkd = QldvHelloPacket(src_drone=my_drone,
                                    creation_time=self.simulator.env.now,
                                    id_hello_packet=GL_ID_HELLO_PACKET,
                                    hello_packet_length=config.HELLO_PACKET_LENGTH,
                                    update_material=update_material,
                                    simulator=self.simulator)
        hello_pkd.transmission_mode = 1  # broadcast

        logging.info('At time: %s, UAV: %s has hello packet to broadcast',
                     self.simulator.env.now, self.my_drone.identifier)

        self.simulator.metrics.control_packet_num += 1
        self.my_drone.transmitting_queue.put(hello_pkd)

    def broadcast_hello_packet_periodically(self):
        while True:
            self.broadcast_hello_packet(self.my_drone)
            jitter = random.randint(1000, 2000)  # delay jitter
            yield self.simulator.env.timeout(self.hello_interval+jitter)

    def purge_periodically(self):
        global GL_ID_ERROR_PACKET
        while True:
            flag, error_update_material = self.table.purge()
            if flag == 1:
                GL_ID_ERROR_PACKET += 1
                error_pkd = QldvErrorPacket(src_drone=self.my_drone,
                                            creation_time=self.simulator.env.now,
                                            id_hello_packet=GL_ID_HELLO_PACKET,
                                            hello_packet_length=config.HELLO_PACKET_LENGTH,
                                            error_update_material=error_update_material,
                                            simulator=self.simulator)
                error_pkd.transmission_mode = 1

                logging.info('At time: %s, UAV: %s has error packet to broadcast',
                             self.simulator.env.now, self.my_drone.identifier)

                self.simulator.metrics.control_packet_num += 1
                self.my_drone.transmitting_queue.put(error_pkd)
            else:
                pass

            yield self.simulator.env.timeout(0.4 * 1e6)

    def next_hop_selection(self, packet):
        """
        Select the next hop according to the routing protocol
        :param packet: the data packet that needs to be sent
        :return: next hop drone
        """
        enquire = False  # "True" when reactive protocol is adopted
        has_route = True

        dst_drone = packet.dst_drone

        # choose best next hop according to the neighbor table
        best_next_hop_id = self.table.best_neighbor(dst_drone)

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

        global GL_ID_ACK_PACKET

        current_time = self.simulator.env.now
        if isinstance(packet, QldvHelloPacket):
            self.table.regular_update(packet, current_time)

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

            GL_ID_ACK_PACKET += 1
            src_drone = self.simulator.drones[src_drone_id]  # previous drone
            ack_packet = AckPacket(src_drone=self.my_drone,
                                   dst_drone=src_drone,
                                   ack_packet_id=GL_ID_ACK_PACKET,
                                   ack_packet_length=config.ACK_PACKET_LENGTH,
                                   ack_packet=packet_copy,
                                   simulator=self.simulator)

            yield self.simulator.env.timeout(config.SIFS_DURATION)  # switch from receiving to transmitting

            # unicast the ack packet immediately without contention for the channel
            if not self.my_drone.sleep:
                self.my_drone.mac_protocol.phy.unicast(ack_packet, src_drone_id)
                yield self.simulator.env.timeout(ack_packet.packet_length / config.BIT_RATE * 1e6)
            else:
                pass

        elif isinstance(packet, AckPacket):
            key2 = str(self.my_drone.identifier) + '_' + str(self.my_drone.mac_protocol.wait_ack_process_count)

            if self.my_drone.mac_protocol.wait_ack_process_finish[key2] == 0:
                if not self.my_drone.mac_protocol.wait_ack_process_dict[key2].triggered:
                    logging.info('At time: %s, the wait_ack process (id: %s) of UAV: %s is interrupted by UAV: %s',
                                 self.simulator.env.now, key2, self.my_drone.identifier, src_drone_id)
                    self.my_drone.mac_protocol.wait_ack_process_dict[key2].interrupt()

        elif isinstance(packet, QldvErrorPacket):
            error_id = packet.packet_id
            if error_id not in self.processed_error_pkd:
                self.processed_error_pkd.append(error_id)
                next_error_update_material = self.table.urgent_update(packet)

                error_pkd = QldvErrorPacket(src_drone=self.my_drone,
                                            creation_time=self.simulator.env.now,
                                            id_hello_packet=error_id,
                                            hello_packet_length=config.HELLO_PACKET_LENGTH,
                                            error_update_material=next_error_update_material,
                                            simulator=self.simulator)
                error_pkd.transmission_mode = 1

                logging.info('At time: %s, UAV: %s has error packet to broadcast',
                             self.simulator.env.now, self.my_drone.identifier)

                self.simulator.metrics.control_packet_num += 1
                self.my_drone.transmitting_queue.put(error_pkd)

    def check_waiting_list(self):
        while True:
            if not self.my_drone.sleep:
                yield self.simulator.env.timeout(0.6 * 1e6)
                for waiting_pkd in self.my_drone.waiting_list:
                    if self.simulator.env.now < waiting_pkd.creation_time + waiting_pkd.deadline:
                        self.my_drone.waiting_list.remove(waiting_pkd)
                    else:
                        dst_drone = waiting_pkd.dst_drone
                        best_next_hop_id = self.table.best_neighbor(dst_drone)
                        if best_next_hop_id != self.my_drone.identifier:
                            self.my_drone.transmitting_queue.put(waiting_pkd)
                            self.my_drone.waiting_list.remove(waiting_pkd)
                        else:
                            pass
            else:
                break
