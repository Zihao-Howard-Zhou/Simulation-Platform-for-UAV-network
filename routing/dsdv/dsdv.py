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


    References:
        [1] Perkins, C. E., and Bhagwat, P.,"Highly dynamic destination-sequenced distance-vector routing (DSDV) for
            mobile computer," ACM SIGCOMM computer communication review, vol. 24, no. 4, pp. 234-244, 1994.
        [2] He. G, "Destination-sequenced distance vector (DSDV) protocol," Networking Laboratory, Helsinki University
            of Technology, 135, pp. 1-9, 2002.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/4/14
    Updated at: 2024/4/14
    """

    def __init__(self, simulator, my_drone):
        self.simulator = simulator
        self.my_drone = my_drone
        self.hello_interval = 0.5 * 1e6  # broadcast hello packet every 0.5s
        self.routing_table = DsdvRoutingTable(self.simulator.env, my_drone)
        self.simulator.env.process(self.broadcast_hello_packet_periodically())

    def broadcast_hello_packet(self, my_drone):
        global GL_ID_HELLO_PACKET

        GL_ID_HELLO_PACKET += 1  #
        hello_pkd = DsdvHelloPacket(src_drone=my_drone, creation_time=self.simulator.env.now,
                                    id_hello_packet=GL_ID_HELLO_PACKET,
                                    hello_packet_length=config.HELLO_PACKET_LENGTH,
                                    routing_table=self.routing_table.routing_table,
                                    simulator=self.simulator)
        hello_pkd.transmission_mode = 1  # broadcast

        logging.info('At time: %s, UAV: %s has hello packet to broadcast',
                     self.simulator.env.now, self.my_drone.identifier)

        yield self.simulator.env.process(my_drone.packet_coming(hello_pkd))

    def broadcast_hello_packet_periodically(self):
        while True:
            self.simulator.env.process(self.broadcast_hello_packet(self.my_drone))
            jitter = random.randint(1000, 2000)  # delay jitter
            yield self.simulator.env.timeout(self.hello_interval+jitter)

    def next_hop_selection(self, packet):
        """
        Select the next hop according to the routing table
        :param packet: the data packet that needs to be sent
        :return: next hop drone
        """

        # update routing table
        self.routing_table.purge()

        dst_drone = packet.dst_drone

        best_next_hop_id = self.routing_table.has_entry(dst_drone.identifier)

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
        if isinstance(packet, DsdvHelloPacket):
            self.routing_table.update_item(packet, current_time)
            self.routing_table.print_neighbor(self.my_drone)

            yield self.simulator.env.timeout(1)
