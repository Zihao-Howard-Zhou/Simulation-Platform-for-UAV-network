import random
import logging
from entities.packet import DataPacket, AckPacket
from routing.dsdv.dsdv_packet import DsdvHelloPacket
from routing.dsdv.dsdv_routing_table import DsdvRoutingTable
from utils import config

# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=config.LOGGING_LEVEL
                    )

GL_ID_HELLO_PACKET = 5000
GL_ID_ACK_PACKET = 10000


class Dsdv:
    """
    Main procedure of DSDV (v1.0)

    Attributes:
        simulator: the simulation platform that contains everything
        my_drone: the drone that installed the GPSR
        hello_interval: interval of sending hello packet
        routing_table: routing table of DSDV

    References:
        [1] Perkins, C. E., and Bhagwat, P.,"Highly dynamic destination-sequenced distance-vector routing (DSDV) for
            mobile computer," ACM SIGCOMM computer communication review, vol. 24, no. 4, pp. 234-244, 1994.
        [2] He. G, "Destination-sequenced distance vector (DSDV) protocol," Networking Laboratory, Helsinki University
            of Technology, 135, pp. 1-9, 2002.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/4/14
    Updated at: 2024/5/4
    """

    def __init__(self, simulator, my_drone):
        self.simulator = simulator
        self.my_drone = my_drone
        self.hello_interval = 0.5 * 1e6  # broadcast hello packet every 0.5s
        self.routing_table = DsdvRoutingTable(self.simulator.env, my_drone)
        self.simulator.env.process(self.broadcast_hello_packet_periodically())
        self.simulator.env.process(self.detect_broken_link_periodically(my_drone))

    def detect_broken_link_periodically(self, my_drone):
        """
        If a node finds that it has not received a hello packet from a neighbor for more than a period of time, it can
        be considered that the link is broken and an update packet needs to be broadcast immediately
        :param my_drone: the node that installs the protocol
        :return: none
        """

        global GL_ID_HELLO_PACKET

        while True:
            yield self.simulator.env.timeout(0.5 * 1e6)  # detect the broken link every 0.5s
            flag = self.routing_table.purge()

            if flag == 1:
                GL_ID_HELLO_PACKET += 1
                hello_pkd = DsdvHelloPacket(src_drone=my_drone,
                                            creation_time=self.simulator.env.now,
                                            id_hello_packet=GL_ID_HELLO_PACKET,
                                            hello_packet_length=config.HELLO_PACKET_LENGTH,
                                            routing_table=self.routing_table.routing_table,
                                            simulator=self.simulator)
                hello_pkd.transmission_mode = 1  # broadcast

                logging.info('At time: %s, UAV: %s broadcast a hello packet to announce broken links',
                             self.simulator.env.now, self.my_drone.identifier)

                self.simulator.metrics.control_packet_num += 1
                self.my_drone.transmitting_queue.put(hello_pkd)

    def broadcast_hello_packet(self, my_drone):
        global GL_ID_HELLO_PACKET

        GL_ID_HELLO_PACKET += 1

        self.routing_table.routing_table[self.my_drone.identifier][2] += 2  # important!
        hello_pkd = DsdvHelloPacket(src_drone=my_drone,
                                    creation_time=self.simulator.env.now,
                                    id_hello_packet=GL_ID_HELLO_PACKET,
                                    hello_packet_length=config.HELLO_PACKET_LENGTH,
                                    routing_table=self.routing_table.routing_table,
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

    def next_hop_selection(self, packet):
        """
        Select the next hop according to the routing table
        :param packet: the data packet that needs to be sent
        :return: next hop drone
        """

        has_route = True
        enquire = False  # "True" when reactive protocol is adopted

        dst_drone = packet.dst_drone

        best_next_hop_id = self.routing_table.has_entry(dst_drone.identifier)
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
        if isinstance(packet, DsdvHelloPacket):
            self.routing_table.update_item(packet, current_time)
            # self.routing_table.print_neighbor(self.my_drone)

        elif isinstance(packet, DataPacket):
            if packet.dst_drone.identifier == self.my_drone.identifier:
                self.simulator.metrics.deliver_time_dict[packet.packet_id] = self.simulator.env.now - packet.creation_time
                self.simulator.metrics.datapacket_arrived.add(packet.packet_id)
                logging.info('Packet: %s is received by destination UAV: %s', packet.packet_id, self.my_drone.identifier)
            else:
                self.my_drone.transmitting_queue.put(packet)

            GL_ID_ACK_PACKET += 1
            src_drone = self.simulator.drones[src_drone_id]  # previous drone
            ack_packet = AckPacket(src_drone=self.my_drone,
                                   dst_drone=src_drone,
                                   ack_packet_id=GL_ID_ACK_PACKET,
                                   ack_packet_length=config.ACK_PACKET_LENGTH,
                                   ack_packet=packet,
                                   simulator=self.simulator)

            yield self.simulator.env.timeout(config.SIFS_DURATION)  # switch from receiving to transmitting

            # unicast the ack packet immediately without contention for the channel
            if not self.my_drone.sleep:
                ack_packet.increase_ttl()
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
