import logging
from utils import config
from utils.util_function import euclidean_distance

# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=config.LOGGING_LEVEL
                    )


class Phy:
    """
    Physical layer implementation

    Evaluation of delays: 1) propagation delay 2) transmission delay 3) queuing delay and 4) processing delay,
    where the propagation delay is the time it takes for bits to travel from one end of the link to the other. Since
    the signal travels in the channel as an electromagnetic wave at the speed of light, the propagation delay is
    negligible in our simulation. Transmission delay is the time needed to push all the packet bits on the transmission
    link. It mainly depends upon the size of the data and channel bandwidth (in bps). Queuing delay and processing
    delay are considered in "drone.py" and "csma_ca.py".

    Attributes:
        mac: mac protocol that installed
        env: simulation environment created by simpy
        my_drone: the drone that installed the physical protocol
        send_process: used to add function "receive" to the simulation environment

    Future work: take co-channel interference into account, calculate the SINR before receiving the packet

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2024/2/26
    """

    def __init__(self, mac):
        self.mac = mac
        self.env = mac.env
        self.my_drone = mac.my_drone
        self.send_process = None

    def unicast(self, packet, next_hop_id):
        """
        Unicast packet through the wireless channel
        :param packet: the data packet or ACK packet that needs to be transmitted
        :param next_hop_id: the identifier of the next hop drone
        :return: none
        """

        next_hop_drone = self.my_drone.simulator.drones[next_hop_id]

        # a transmission delay should be considered
        yield self.env.timeout(packet.packet_length / config.BIT_RATE * 1e6)

        # energy consumption
        energy_consumption = (packet.packet_length / config.BIT_RATE) * config.TRANSMITTING_POWER
        self.my_drone.residual_energy -= energy_consumption

        # transmit through the channel
        message = (packet, self.env.now, self.my_drone.identifier)

        self.my_drone.simulator.channel.unicast_put(message, next_hop_id)

        self.send_process = self.env.process(next_hop_drone.mac_protocol.phy.receive())
        yield self.send_process

    def broadcast(self, packet):
        """
        Broadcast packet through the wireless channel
        :param packet: tha packet (hello packet, etc.) that needs to be broadcast
        :return: none
        """

        # a transmission delay should be considered
        yield self.env.timeout(packet.packet_length / config.BIT_RATE * 1e6)

        # energy consumption
        energy_consumption = (packet.packet_length / config.BIT_RATE) * config.TRANSMITTING_POWER
        self.my_drone.residual_energy -= energy_consumption

        # transmit through the channel
        message = (packet, self.env.now, self.my_drone.identifier)

        self.my_drone.simulator.channel.broadcast_put(message)

        for drone in self.my_drone.simulator.drones:
            self.send_process = self.env.process(drone.mac_protocol.phy.receive())
            yield self.send_process

    def receive(self):
        if not self.my_drone.sleep:
            msg = yield self.my_drone.certain_channel.get()

            previous_drone = self.my_drone.simulator.drones[msg[2]]

            if euclidean_distance(self.my_drone.coords, previous_drone.coords) <= config.COMMUNICATION_RANGE:
                logging.info('UAV: %s receives the message: %s at %s, previous hop is: %s',
                             self.my_drone.identifier, msg[0], self.env.now, msg[2])
                yield self.env.process(self.my_drone.routing_protocol.packet_reception(msg[0], msg[2]))
            else:
                pass
        else:  # cannot receive packets if "my_drone" is in sleep state
            yield self.my_drone.certain_channel.get()
