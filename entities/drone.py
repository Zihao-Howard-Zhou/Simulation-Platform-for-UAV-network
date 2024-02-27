import simpy
import logging
import numpy as np
import random
import math
import queue
from entities.packet import DataPacket
from mac.csma_ca import CsmaCa
from mobility.gauss_markov_3d import GaussMarkov3D
from utils import config

# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=logging.DEBUG
                    )

GLOBAL_DATA_PACKET_ID = 0

class Drone:
    """
    Drone implementation

    Attributes:
        simulator:
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
        certain_channel: a store created for the drone
                         different drones will have different stores for transmitting and receiving
        buffer: each drone has a buffer to store the coming data packets
        fifo_queue: when the next hop node receives the packet, it should first temporarily store the packet in
                    fifo_queue instead of immediately yield "packet_coming" process, so that the buffer resources of
                    the previous hop node are not always occupied
        mac_protocol: installed mac protocol (CSMA/CA, ALOHA, etc.)
        mac_process_dict: a dictionary, used to store the mac_process that is triggered each time
        mac_process_finish: a dictionary, used to indicate the completion of the process
        mac_process_count: used to distinguish between different processes
        routing_protocol: installed routing protocol (GPSR, AODV, etc.)
        mobility_model: installed mobility model (3-D Gauss-markov, random waypoint, etc.)

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2024/2/22
    """

    def __init__(self,
                 env,
                 node_id: int,
                 coords: list,
                 speed: float,
                 certain_channel,
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

        self.certain_channel = certain_channel

        self.buffer = simpy.Resource(env, capacity=1)
        self.fifo_queue = queue.Queue()

        self.mac_protocol = CsmaCa(self)
        self.mac_process_dict = dict()
        self.mac_process_finish = dict()
        self.mac_process_count = 0

        self.routing_protocol = self.simulator.routing_protocol.value(self.simulator, self)

        self.mobility_model = GaussMarkov3D(self)

        if self.identifier != 0:
            self.env.process(self.generate_data_packet())

        self.env.process(self.feed_packet())

    def generate_data_packet(self, traffic_pattern='Uniform'):
        """
        Generate one data packet, it should be noted that only when the current packet has been sent can
        the next packet be started
        """

        global GLOBAL_DATA_PACKET_ID

        while True:
            if traffic_pattern is 'Uniform':
                # the drone generates a data packet every 0.5s with jitter
                yield self.env.timeout(random.randint(500000, 505000))
            elif traffic_pattern is 'Poisson':
                # the process of generating data packets by nodes follows Poisson distribution,
                # thus the generation interval of data packets follows exponential distribution

                rate = 2  # on average, 2 packets are generated in 1s
                yield self.env.timeout(round(random.expovariate(rate) * 1e6))

            GLOBAL_DATA_PACKET_ID += 1  # packet id
            destination = self.simulator.drones[0]  # obtain the destination drone

            pkd = DataPacket(self, dst_drone=destination, creation_time=self.env.now,
                             data_packet_id=GLOBAL_DATA_PACKET_ID,
                             data_packet_length=config.DATA_PACKET_LENGTH, simulator=self.simulator)

            self.simulator.metrics.datapacket_generated_num += 1

            logging.info('At time: %s, UAV: %s generate a data packet whose dst is: %s, and pkd id is: %s',
                         self.env.now, self.identifier, destination.identifier, pkd.packet_id)

            yield self.env.process(self.packet_coming(pkd, 0))  # unicast

    def packet_coming(self, pkd, tm):
        """
        When drone generates a packet or receives a data packet that is not bound for itself, yield it
        :param pkd: packet that waits to enter the buffer of drone
        :param tm: transmission mode
        :return: none
        """

        arrival_time = self.env.now
        logging.info('Packet: %s waiting for UAV: %s buffer resource at: %s',
                     pkd.packet_id, self.identifier, arrival_time)

        with self.buffer.request() as request:
            yield request  # wait to enter to buffer

            logging.info('Packet: %s has been added to the buffer at: %s of UAV: %s, waiting time is: %s',
                         pkd.packet_id, self.env.now, self.identifier, self.env.now - pkd.creation_time)

            pkd.number_retransmission_attempt[self.identifier] += 1
            logging.info('Re-transmission times of pkd: %s at UAV: %s is: %s',
                         pkd.packet_id, self.identifier, pkd.number_retransmission_attempt[self.identifier])

            self.mac_process_count += 1
            key = str(self.identifier) + str(self.mac_process_count)
            mac_process = self.env.process(self.mac_protocol.mac_send(pkd, tm, self.mac_process_count))
            self.mac_process_dict[key] = mac_process
            self.mac_process_finish[key] = 0

            yield mac_process

    def feed_packet(self):
        while True:
            yield self.env.timeout(10)  # for speed up the simulation
            if not self.fifo_queue.empty():
                data_packet = self.fifo_queue.get()
                if data_packet.number_retransmission_attempt[self.identifier] < config.MAX_RETRANSMISSION_ATTEMPT:
                    yield self.env.process(self.packet_coming(data_packet, 0))
