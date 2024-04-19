import logging
import math
import numpy as np
from entities.packet import DataPacket, AckPacket
from utils import config
from utils.util_function import euclidean_distance
from phy.large_scale_fading import maximum_communication_range


# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=config.LOGGING_LEVEL
                    )

GL_ID_DATA_PACKET = 5000
GL_ID_ACK_PACKET = 10000


class Opar:
    """
    Main procedure of OPAR (v3.0)

    Attributes:
        simulator: the simulation platform that contains everything
        my_drone: the drone that installed the routing protocol
        cost: cost matrix, used to record the cost of all links
        best_obj: the minimum objective function value under all iterations
        best_path: optimal routing path corresponding to "best_obj"
        w1: weight of the first term in objective function
        w2: weight of the second term in objective function
        max_comm_range: maximum communication range corresponding to the snr threshold

    References:
        [1] M. Gharib, F. Afghah and E. Bentley, "OPAR: Optimized Predictive and Adaptive Routing for Cooperative UAV
            Networks," in IEEE Conference on Computer Communications Workshops, PP. 1-6, 2021.
        [2] M. Gharib, F. Afghah and E. Bentley, "LB-OPAR: Load Balanced Optimized Predictive and Adaptive Routing for
            Cooperative UAV Networks," Ad hoc Networks, vol. 132, pp. 102878, 2022.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/3/19
    Updated at: 2024/4/11
    """

    def __init__(self, simulator, my_drone):
        self.simulator = simulator
        self.my_drone = my_drone
        self.cost = None
        self.best_obj = 0
        self.best_path = None

        self.w1 = 0.5
        self.w2 = 0.5

        self.max_comm_range = maximum_communication_range()

    def calculate_cost_matrix(self):
        cost = np.zeros((self.simulator.n_drones, self.simulator.n_drones))
        cost.fill(np.inf)

        for i in range(self.simulator.n_drones):
            for j in range((i+1), self.simulator.n_drones):
                drone1 = self.simulator.drones[i]
                drone2 = self.simulator.drones[j]

                if (i != j) and (euclidean_distance(drone1.coords, drone2.coords) < self.max_comm_range):
                    cost[i, j] = 1
                    cost[j, i] = 1

        return cost

    def dijkstra(self, cost, src_id, dst_id, minimum_link_lifetime):
        """
        Dijkstra's algorithm to find the shortest path
        :param cost: cost matrix
        :param src_id: source node id
        :param dst_id: destination node id
        :param minimum_link_lifetime: used to determine which edges cannot be considered in this iteration
        :return: routing path that has the minimum total cost
        """

        distance_list = [np.inf for _ in range(self.simulator.n_drones)]
        distance_list[src_id] = 0

        prev_list = [-1 for _ in range(self.simulator.n_drones)]
        prev_list[src_id] = -2

        visited_list = [False for _ in range(self.simulator.n_drones)]

        for i in range(self.simulator.n_drones):
            unvisited_list = [(index, value) for index, value in enumerate(distance_list) if not visited_list[index]]
            min_distance_node, _ = min(unvisited_list, key=lambda x: x[1])

            visited_list[min_distance_node] = True

            for j in range(self.simulator.n_drones):
                drone1 = self.simulator.drones[min_distance_node]
                drone2 = self.simulator.drones[j]

                if (visited_list[j] is False) and (cost[min_distance_node, j] != np.inf):
                    delta_temp = link_lifetime_predictor(drone1, drone2, self.max_comm_range)

                    if delta_temp <= minimum_link_lifetime:
                        cost[min_distance_node, j] = np.inf
                        cost[j, min_distance_node] = np.inf

                    alt = distance_list[min_distance_node] + cost[min_distance_node, j]
                    if alt < distance_list[j]:
                        distance_list[j] = alt
                        prev_list[j] = min_distance_node

        # path construction
        current_node = dst_id
        path = [dst_id]

        while current_node != -2:
            current_node = prev_list[current_node]

            if current_node != -1:
                path.insert(0, current_node)
            else:
                path = []
                break

        return path

    def next_hop_selection(self, packet):
        if packet.src_drone is self.my_drone:  # if it is the source, optimization should be executed
            self.cost = self.calculate_cost_matrix()
            temp_cost = self.cost
            src_drone = self.my_drone  # packet.src_drone
            dst_drone = packet.dst_drone  # get the destination of the data packet

            path = self.dijkstra(temp_cost, src_drone.identifier, dst_drone.identifier, 0)

            if len(path) != 0:
                path.pop(0)

                total_cost = 0
                t = 0
                minimum_link_lifetime = 1e11

                for link in range(len(path)-1):
                    drone1 = self.simulator.drones[path[link]]
                    drone2 = self.simulator.drones[path[link+1]]
                    link_cost = self.cost[path[link], path[link+1]]
                    total_cost += link_cost

                    delta_t = link_lifetime_predictor(drone1, drone2, self.max_comm_range)

                    if 1/delta_t > t:
                        t = delta_t

                    if delta_t < minimum_link_lifetime:
                        minimum_link_lifetime = delta_t

                # calculate the objective function
                obj = self.w1 * total_cost + self.w2 * t
                self.best_obj = obj
                self.best_path = path
            else:
                minimum_link_lifetime = None
                self.best_path = [src_drone.identifier, src_drone.identifier]

            while len(path) != 0:
                path = self.dijkstra(temp_cost, src_drone.identifier, dst_drone.identifier, minimum_link_lifetime)

                if len(path) != 0:
                    path.pop(0)

                    total_cost = 0
                    t = 0
                    minimum_link_lifetime = 1e11

                    for link in range(len(path) - 1):
                        drone1 = self.simulator.drones[path[link]]
                        drone2 = self.simulator.drones[path[link + 1]]
                        link_cost = self.cost[path[link], path[link + 1]]
                        total_cost += link_cost

                        delta_t = link_lifetime_predictor(drone1, drone2, self.max_comm_range)

                        if 1 / delta_t > t:
                            t = delta_t

                        if delta_t < minimum_link_lifetime:
                            minimum_link_lifetime = delta_t

                    # calculate the objective function
                    obj = self.w1 * total_cost + self.w2 * t

                    if obj < self.best_obj:
                        self.best_obj = obj
                        self.best_path = path

            self.best_path.pop(0)  # remove myself
            packet.routing_path = self.best_path
            best_next_hop_id = self.best_path[0]

        else:  # for relay nodes, no additional calculations are required
            routing_path = packet.routing_path

            if len(routing_path) > 1:
                routing_path.pop(0)
                packet.routing_path = routing_path
                best_next_hop_id = routing_path[0]
            else:
                # if it is passed to itself, it'll try to find the path again the next time the packet is sent
                best_next_hop_id = self.my_drone.identifier

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

        if isinstance(packet, DataPacket):
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
            if not self.my_drone.sleep:
                yield self.simulator.env.process(self.my_drone.mac_protocol.phy.unicast(ack_packet, src_drone_id))
            else:
                pass

        elif isinstance(packet, AckPacket):
            key2 = str(self.my_drone.identifier) + '_' + str(self.my_drone.mac_protocol.wait_ack_process_count)

            if self.my_drone.mac_protocol.wait_ack_process_finish[key2] == 0:
                logging.info('At time: %s, the wait_ack process (id: %s) of UAV: %s is interrupted by UAV: %s',
                             self.simulator.env.now, key2, self.my_drone.identifier, src_drone_id)
                self.my_drone.mac_protocol.wait_ack_process_dict[key2].interrupt()


def link_lifetime_predictor(drone1, drone2, max_comm_range):
    coords1 = drone1.coords
    coords2 = drone2.coords
    velocity1 = drone1.velocity
    velocity2 = drone2.velocity

    x1 = (velocity1[0] - velocity2[0]) ** 2
    x2 = (velocity1[1] - velocity2[1]) ** 2
    x3 = (velocity1[2] - velocity2[2]) ** 2

    y1 = 2*(velocity1[0] - velocity2[0])*(coords1[0] - coords2[0])
    y2 = 2*(velocity1[1] - velocity2[1])*(coords1[1] - coords2[1])
    y3 = 2*(velocity1[2] - velocity2[2])*(coords1[2] - coords2[2])

    z1 = (coords1[0] - coords2[0]) ** 2
    z2 = (coords1[1] - coords2[1]) ** 2
    z3 = (coords1[2] - coords2[2]) ** 2

    A = x1 + x2 + x3

    B = y1 + y2 + y3
    C = (z1 + z2 + z3) - max_comm_range ** 2

    delta_t_1 = (-B + math.sqrt(B ** 2 - 4 * A * C)) / (2 * A)
    delta_t_2 = (-B - math.sqrt(B ** 2 - 4 * A * C)) / (2 * A)

    delta_t = max(delta_t_1, delta_t_2)

    return delta_t
