import simpy
import logging
import numpy as np
import random
import math
import queue
from entities.packet import DataPacket
from routing.dsdv.dsdv import Dsdv
from routing.gpsr.gpsr import Gpsr
from routing.grad.grad import Grad
from routing.opar.opar import Opar
from routing.parrot.parrot import Parrot
from routing.qgeo.qgeo import QGeo
from mac.csma_ca import CsmaCa
from mac.pure_aloha import PureAloha
from mobility.gauss_markov_3d import GaussMarkov3D
from mobility.random_walk_3d import RandomWalk3D
from mobility.random_waypoint_3d import RandomWaypoint3D
from energy.energy_model import EnergyModel
from utils import config
from phy.large_scale_fading import sinr_calculator

# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=config.LOGGING_LEVEL
                    )

GLOBAL_DATA_PACKET_ID = 0


class Drone:
    """
    Drone implementation

    Drones in the simulation are served as routers. Each drone can be selected as a potential source node, destination and
    relaying node. Each drone needs to install the corresponding routing module, MAC module, mobility module and energy
    module, etc. At the same time, each drone also has its own queue and can only send one packet at a time, so
    subsequent data packets need queuing for queue resources, which is used to reflect the queue delay in the drone network

    Attributes:
        simulator: the simulation platform that contains everything
        env: simulation environment created by simpy
        identifier: used to uniquely represent a drone
        coords: the 3-D position of the drone
        direction: current direction of the drone
        pitch: current pitch of the drone
        speed: current speed of the drone
        velocity: velocity components in three directions
        direction_mean: mean direction
        pitch_mean: mean pitch
        velocity_mean: mean velocity
        inbox: a "Store" in simpy, used to receive the packets from other drones (calculate SINR)
        buffer: used to describe the queuing delay of sending packet
        transmitting_queue: when the next hop node receives the packet, it should first temporarily store the packet in
                    "transmitting_queue" instead of immediately yield "packet_coming" process. It can prevent the buffer
                    resource of the previous hop node from being occupied all the time
        waiting_list: for reactive routing protocol, if there is no available next hop, it will put the data packet into
                      "waiting_list". Once the routing information bound for a destination is obtained, drone will get
                      the data packets related to this destination, and put them into "transmitting_queue"
        mac_protocol: installed mac protocol (CSMA/CA, ALOHA, etc.)
        mac_process_dict: a dictionary, used to store the mac_process that is launched each time
        mac_process_finish: a dictionary, used to indicate the completion of the process
        mac_process_count: used to distinguish between different "mac_send" processes
        routing_protocol: routing protocol installed (GPSR, DSDV, etc.)
        mobility_model: mobility model installed (3-D Gauss-markov, 3-D random waypoint, etc.)
        energy_model: energy consumption model installed
        residual_energy: the residual energy of drone in Joule
        sleep: if the drone is in a "sleep" state, it cannot perform packet sending and receiving operations.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2024/4/22
    """

    def __init__(self,
                 env,
                 node_id,
                 coords,
                 speed,
                 inbox,
                 simulator):
        self.simulator = simulator
        self.env = env
        self.identifier = node_id
        self.coords = coords

        random.seed(2024+self.identifier)
        self.direction = random.uniform(0, 2 * np.pi)

        random.seed(2025+self.identifier)
        self.pitch = random.uniform(-0.05, 0.05)
        self.speed = speed
        self.velocity = [self.speed * math.cos(self.direction) * math.cos(self.pitch),
                         self.speed * math.sin(self.direction) * math.cos(self.pitch),
                         self.speed * math.sin(self.pitch)]

        self.direction_mean = self.direction
        self.pitch_mean = self.pitch
        self.velocity_mean = self.speed

        self.inbox = inbox

        self.buffer = simpy.Resource(env, capacity=1)
        self.transmitting_queue = queue.Queue()
        self.waiting_list = []

        self.mac_protocol = PureAloha(self)
        self.mac_process_dict = dict()
        self.mac_process_finish = dict()
        self.mac_process_count = 0

        self.routing_protocol = Gpsr(self.simulator, self)

        self.mobility_model = GaussMarkov3D(self)

        self.energy_model = EnergyModel()
        self.residual_energy = config.INITIAL_ENERGY
        self.sleep = False

        if self.identifier != 0:
            self.env.process(self.generate_data_packet())

        self.env.process(self.feed_packet())
        self.env.process(self.energy_monitor())
        self.env.process(self.receive())

    def generate_data_packet(self, traffic_pattern='Uniform'):
        """
        Generate one data packet, it should be noted that only when the current packet has been sent can the next
        packet be started. When the drone generates a data packet, it will first put it into the "transmitting_queue",
        the drone reads a data packet from the head of the queue every very short time through "feed_packet()" function.

        :param traffic_pattern: characterize the time interval between generating data packets
        :return: none
        """

        global GLOBAL_DATA_PACKET_ID

        while True:
            if not self.sleep:
                if traffic_pattern == 'Uniform':
                    # the drone generates a data packet every 0.5s with jitter
                    yield self.env.timeout(random.randint(500000, 505000))
                elif traffic_pattern == 'Poisson':
                    # the process of generating data packets by nodes follows Poisson distribution,
                    # thus the generation interval of data packets follows exponential distribution

                    rate = 2  # on average, 2 packets are generated in 1s
                    yield self.env.timeout(round(random.expovariate(rate) * 1e6))

                GLOBAL_DATA_PACKET_ID += 1  # packet id

                # randomly choose a destination
                # all_candidate_list = [i for i in range(config.NUMBER_OF_DRONES)]
                # all_candidate_list.remove(self.identifier)
                # dst_id = random.choice(all_candidate_list)
                dst_id = 0

                destination = self.simulator.drones[dst_id]  # obtain the destination drone

                pkd = DataPacket(self, dst_drone=destination, creation_time=self.env.now,
                                 data_packet_id=GLOBAL_DATA_PACKET_ID, data_packet_length=config.DATA_PACKET_LENGTH,
                                 simulator=self.simulator)
                pkd.transmission_mode = 0  # the default transmission mode of data packet is "unicast" (0)

                self.simulator.metrics.datapacket_generated_num += 1

                logging.info('------> At time: %s, UAV: %s generates a data packet (id: %s, dst: %s)',
                             self.env.now, self.identifier, pkd.packet_id, destination.identifier)

                # yield self.env.process(self.packet_coming(pkd))  # unicast
                self.transmitting_queue.put(pkd)
            else:  # cannot generate packets if "my_drone" is in sleep state
                break

    def feed_packet(self):
        """
        It should be noted that this function is designed for those packets which need to compete for wireless channel

        Firstly, all packets received or generated will be put into the "transmitting_queue", every very short
        time, the drone will read the packet in the head of the "transmitting_queue". Then the drone will check
        if the packet is expired (exceed its maximum lifetime in the network), check the type of packet:
        1) data packet: check if the data packet exceeds its maximum re-transmission attempts. If the above inspection
           passes, routing protocol is executed to determine the next hop drone. If next hop is found, then this data
           packet is ready to transmit, otherwise, it will be put into the "waiting_queue".
        2) control packet: no need to determine next hop, so it will directly start waiting for buffer

        :return: none
        """

        while True:
            if not self.sleep:  # if drone still has enough energy to relay packets
                yield self.env.timeout(10)  # for speed up the simulation
                if not self.transmitting_queue.empty():
                    packet = self.transmitting_queue.get()  # get the packet at the head of the queue

                    if self.env.now < packet.creation_time + packet.deadline:
                        if isinstance(packet, DataPacket):
                            if packet.number_retransmission_attempt[self.identifier] < config.MAX_RETRANSMISSION_ATTEMPT:
                                # it should be noted that "final_packet" may be the data packet itself or may be a control
                                # packet, depending on whether the routing protocol can find an appropriate next hop
                                has_route, final_packet = self.routing_protocol.next_hop_selection(packet)

                                if has_route:
                                    logging.info('At time: %s, UAV: %s obtain the next hop of data packet (id: %s), '
                                                 'which is: %s, and this packet will wait buffer resource.',
                                                 self.env.now, self.identifier, packet.packet_id, packet.next_hop_id)

                                    yield self.env.process(self.packet_coming(final_packet))  # actually the data packet
                                else:
                                    logging.info('Unfortunately, at time: %s, UAV: %s cannot find appropriate next ' 
                                        'hop of data packet (id: %s), and it will put the packet into waiting queue.',
                                        self.env.now, self.identifier, packet.packet_id)

                                    self.waiting_list.append(packet)
                                    yield self.env.process(self.packet_coming(final_packet))  # actually the control packet
                        else:  # control packet but not ack
                            yield self.env.process(self.packet_coming(packet))
                    else:
                        pass  # means dropping this data packet for expiration
            else:
                break  # important to break the while loop

    def packet_coming(self, pkd):
        """
        When drone has a packet ready to transmit, yield it.
        The requirement of "ready" is:
        1) this packet is a control packet or 2) drone knows the next hop of the data packet

        :param pkd: packet that waits to enter the buffer of drone
        :return: none
        """

        if not self.sleep:
            arrival_time = self.env.now
            logging.info('Packet: %s waiting for UAV: %s buffer resource at: %s',
                         pkd.packet_id, self.identifier, arrival_time)

            with self.buffer.request() as request:
                yield request  # wait to enter to buffer

                logging.info('Packet: %s has been added to the buffer at: %s of UAV: %s, waiting time is: %s',
                             pkd.packet_id, self.env.now, self.identifier, self.env.now - arrival_time)

                pkd.number_retransmission_attempt[self.identifier] += 1

                if pkd.number_retransmission_attempt[self.identifier] == 1:
                    pkd.time_transmitted_at_last_hop = self.env.now

                logging.info('Re-transmission times of pkd: %s at UAV: %s is: %s',
                             pkd.packet_id, self.identifier, pkd.number_retransmission_attempt[self.identifier])

                # every time the drone initiates a data packet transmission, "mac_process_count" will be increased by 1
                self.mac_process_count += 1
                key = str(self.identifier) + '_' + str(self.mac_process_count)  # used to uniquely refer to a process
                mac_process = self.env.process(self.mac_protocol.mac_send(pkd))
                self.mac_process_dict[key] = mac_process
                self.mac_process_finish[key] = 0

                yield mac_process
        else:
            pass

    def energy_monitor(self):
        while True:
            yield self.env.timeout(1*1e5)  # report residual energy every 0.1s
            if self.residual_energy <= config.ENERGY_THRESHOLD:
                self.sleep = True
                # print('UAV: ', self.identifier, ' run out of energy at: ', self.env.now)

    def receive(self):
        while True:
            if not self.sleep:
                if self.inbox.items:  # non-empty list means that someone wants to transmit packet to me
                    # get the list of all drones currently transmitting packets
                    transmitting_node_list = []
                    for drone in self.simulator.drones:
                        if drone.inbox.items:
                            temp = [msg[2] for msg in drone.inbox.items]
                            transmitting_node_list += temp

                    transmitting_node_list = list(set(transmitting_node_list))  # remove duplicates

                    all_drones_send_to_me = [msg[2] for msg in self.inbox.items]

                    sinr_list = sinr_calculator(self, all_drones_send_to_me, transmitting_node_list)

                    # Receive the packet of the transmitting node corresponding to the maximum SINR
                    max_sinr = max(sinr_list)
                    if max_sinr >= config.SNR_THRESHOLD:
                        which_one = all_drones_send_to_me[sinr_list.index(max_sinr)]

                        while self.inbox.items:
                            msg = yield self.inbox.get()
                            previous_drone = self.simulator.drones[msg[2]]

                            if previous_drone.identifier is which_one:
                                logging.info('Packet %s (sending to channel at: %s) from UAV: %s is received '
                                             'by UAV: %s at time: %s', msg[0], msg[1], msg[2],
                                             self.identifier, self.simulator.env.now)
                                yield self.env.process(self.routing_protocol.packet_reception(msg[0], msg[2]))
                    else:  # sinr is lower than threshold
                        while self.inbox.items:
                            yield self.inbox.get()

                yield self.env.timeout(5)
            else:
                break
