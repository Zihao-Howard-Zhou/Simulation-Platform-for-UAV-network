import random
import logging
from entities.packet import DataPacket, AckPacket
from routing.gpsr.gpsr_neighbor_table import GpsrNeighborTable
from routing.gpsr.gpsr_packet import GpsrHelloPacket
from utils import config

# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=logging.INFO
                    )

GL_ID_HELLO_PACKET = 5000
GL_ID_ACK_PACKET = 10000


class Gpsr:
    """
    Main procedure of GPSR (v1.0)

    Attributes:
        simulator: the simulation platform that contains everything
        my_drone: the drone that installed the GPSR
        hello_interval: interval of sending hello packet
        neighbor_table: neighbor table of GPSR

    References:
        [1] Karp B and Kung H T.,"GPSR: Greedy Perimeter Stateless Routing for Wireless Networks," in Proceedings of the
            6-th annual international conference on Mobile computing and networking, pp. 243-254, 2000.
        [2] Fu J, Cui B, Wang N, et al., "A Distributed Position-based Routing Algorithm in 3-D Wireless Industrial
            Internet of Things," IEEE Transactions on Industrial Informatics, vol. 15, no. 10, pp. 5664-5673, 2019.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2024/3/19
    """

    def __init__(self, simulator, my_drone):
        self.simulator = simulator
        self.my_drone = my_drone
        self.hello_interval = 0.5*1e6  # broadcast hello packet every 0.5s
        self.neighbor_table = GpsrNeighborTable(self.simulator.env, my_drone)
        self.simulator.env.process(self.broadcast_hello_packet_periodically())

    def broadcast_hello_packet(self, my_drone):
        global GL_ID_HELLO_PACKET

        GL_ID_HELLO_PACKET += 1
        hello_pkd = GpsrHelloPacket(src_drone=my_drone, creation_time=self.simulator.env.now,
                                    id_hello_packet=GL_ID_HELLO_PACKET,
                                    hello_packet_length=config.HELLO_PACKET_LENGTH,
                                    simulator=self.simulator)

        logging.info('At time: %s, UAV: %s has hello packet to broadcast',
                     self.simulator.env.now, self.my_drone.identifier)

        yield self.simulator.env.process(my_drone.packet_coming(hello_pkd, 1))

    def broadcast_hello_packet_periodically(self):
        while True:
            self.simulator.env.process(self.broadcast_hello_packet(self.my_drone))
            jitter = random.randint(1000, 2000)  # delay jitter
            yield self.simulator.env.timeout(self.hello_interval+jitter)

    def next_hop_selection(self, packet):
        """
        Select the next hop according to the routing protocol
        :param packet: the data packet that needs to be sent
        :return: next hop drone
        """

        # update neighbor table
        self.neighbor_table.purge()

        dst_drone = packet.dst_drone
        if self.neighbor_table.is_neighbor(dst_drone):  # if the destination is my one-hop neighbor
            return dst_drone.identifier  # the destination if the next hop
        else:
            best_next_hop_id = self.neighbor_table.best_neighbor(self.my_drone, dst_drone)

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
        if isinstance(packet, GpsrHelloPacket):
            self.neighbor_table.add_neighbor(packet, current_time)  # update the neighbor table
            # self.neighbor_table.print_neighbor(self.my_drone)

        elif isinstance(packet, DataPacket):
            if packet.dst_drone.identifier == self.my_drone.identifier:
                self.simulator.metrics.deliver_time_dict[packet.packet_id] = self.simulator.env.now - packet.creation_time
                self.simulator.metrics.datapacket_arrived.add(packet.packet_id)
                logging.info('Packet: %s is received by destination UAV: %s', packet.packet_id, self.my_drone.identifier)
            else:
                self.my_drone.fifo_queue.put(packet)

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
