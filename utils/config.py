from routing.gpsr.gpsr import Gpsr
from enum import Enum

# --------------------- simulation parameters --------------------- #
MAP_LENGTH = 1000  # m, length of the map
MAP_WIDTH = 1000  # m, width of the map
MAP_HEIGHT = 10  # m, height of the map
SIM_TIME = 15 * 1e6  # us, total simulation time (10s)
NUMBER_OF_DRONES = 20  # number of drones in the network
STATIC_CASE = 0

# ----------------------- radio parameters ----------------------- #
TRANSMITTING_POWER = 1  # Watt
LIGHT_SPEED = 3*1e8  # light speed (m/s)
CARRIER_FREQUENCY = 1*1e9  # carrier frequency (Hz)
NOISE_POWER = 4*1e-9  # noise power (Watt)
RADIO_SWITCHING_TIME = 100  # us, the switching time of the transceiver mode
RADIO_SENSITIVITY = 1e-10  # power under which signal is not sensed

# ---------------------- packet parameters ----------------------- #
MAX_TTL = 15
PACKET_LIFETIME = 10*1e6
PACKET_HEADER_LENGTH = 128  # bit
DATA_PACKET_PAYLOAD_LENGTH = 1024*8  # bit
DATA_PACKET_LENGTH = PACKET_HEADER_LENGTH + DATA_PACKET_PAYLOAD_LENGTH

ACK_PACKET_LENGTH = 128  # bit

HELLO_PACKET_HEADER_LENGTH = 128  # bit
HELLO_PACKET_PAYLOAD_LENGTH = 256  # bit
HELLO_PACKET_LENGTH = HELLO_PACKET_HEADER_LENGTH + HELLO_PACKET_PAYLOAD_LENGTH

# ------------------ physical layer parameters ------------------- #
BIT_RATE = 54 * 1e6  # 54 Mbit/s, 802.11g 20 MHz channels
BIT_TRANSMISSION_TIME = 1/BIT_RATE * 1e6
NOISE_FLOOR = 1e-9
COMMUNICATION_RANGE = 250
SENSING_RANGE = 300

# --------------------- mac layer parameters --------------------- #
SLOT_DURATION = 50  # 50 microseconds, 802.11g 2.4 GHz
SIFS_DURATION = 28  # 28 microseconds, 802.11g 2.4 GHz
DIFS_DURATION = SIFS_DURATION + (2 * SLOT_DURATION)  # 128 microseconds
MAC_HEADER_LENGTH = 34*8  # 34 byte fixed fields of a mac packet
MAX_MAC_PAYLOAD_LENGTH = 2312*8
ACK_LENGTH = MAC_HEADER_LENGTH
CW_MIN = 16
CW_MAX = 1024
ACK_TIMEOUT = 1000  # maximum waiting time for ACK (0.1s)
MAX_RETRANSMISSION_ATTEMPT = 5

# ------------------- network layer parameters ------------------- #
class RoutingProtocol(Enum):
    gpsr_protocol = Gpsr

    @staticmethod
    def keylist():
        return list(map(lambda c: c.name, RoutingProtocol))


ROUTING_PROTOCOL = RoutingProtocol.gpsr_protocol
