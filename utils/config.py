import logging
from utils.ieee_802_11 import IEEE_802_11

IEEE_802_11 = IEEE_802_11().b

# --------------------- simulation parameters --------------------- #
MAP_LENGTH = 600  # m, length of the map
MAP_WIDTH = 600  # m, width of the map
MAP_HEIGHT = 500  # m, height of the map
SIM_TIME = 35 * 1e6  # us, total simulation time
NUMBER_OF_DRONES = 15  # number of drones in the network
STATIC_CASE = 1
HETEROGENEOUS = 0  # heterogeneous network support (in terms of speed)
LOGGING_LEVEL = logging.INFO  # whether to print the detail information during simulation

# ---------- hardware parameters of drone (rotary-wing) -----------#
PROFILE_DRAG_COEFFICIENT = 0.012
AIR_DENSITY = 1.225  # kg/m^3
ROTOR_SOLIDITY = 0.05  # defined as the ratio of the total blade area to disc area
ROTOR_DISC_AREA = 0.79  # m^2
BLADE_ANGULAR_VELOCITY = 400  # radians/second
ROTOR_RADIUS = 0.5  # m
INCREMENTAL_CORRECTION_FACTOR = 0.1
AIRCRAFT_WEIGHT = 100  # Newton
ROTOR_BLADE_TIP_SPEED = 500
MEAN_ROTOR_VELOCITY = 7.2  # mean rotor induced velocity in hover
FUSELAGE_DRAG_RATIO = 0.3
INITIAL_ENERGY = 20 * 1e3  # in joule
ENERGY_THRESHOLD = 2000  # in joule

# ----------------------- radio parameters ----------------------- #
TRANSMITTING_POWER = 0.1  # in Watt
LIGHT_SPEED = 3 * 1e8  # light speed (m/s)
CARRIER_FREQUENCY = IEEE_802_11['carrier_frequency']  # carrier frequency (Hz)
NOISE_POWER = 4 * 1e-11  # noise power (Watt)
RADIO_SWITCHING_TIME = 100  # us, the switching time of the transceiver mode
SNR_THRESHOLD = 6  # dB

# ---------------------- packet parameters ----------------------- #
MAX_TTL = NUMBER_OF_DRONES + 1  # maximum time-to-live value
PACKET_LIFETIME = 10 * 1e6  # 10s
IP_HEADER_LENGTH = 20 * 8  # header length in network layer, 20 byte
MAC_HEADER_LENGTH = 14 * 8  # header length in mac layer, 14 byte

# ---------------------- physical layer -------------------------- #
PATH_LOSS_EXPONENT = 2  # for large-scale fading
PLCP_PREAMBLE = 128 + 16  # including synchronization and SFD (start frame delimiter)
PLCP_HEADER = 8 + 8 + 16 + 16  # including signal, service, length and HEC (header error check)
PHY_HEADER_LENGTH = PLCP_PREAMBLE + PLCP_HEADER  # header length in physical layer, PLCP preamble + PLCP header

ACK_HEADER_LENGTH = 16 * 8  # header length of ACK packet, 16 byte

DATA_PACKET_PAYLOAD_LENGTH = 1024 * 8  # 1024 byte
DATA_PACKET_LENGTH = IP_HEADER_LENGTH + MAC_HEADER_LENGTH + PHY_HEADER_LENGTH + DATA_PACKET_PAYLOAD_LENGTH

ACK_PACKET_LENGTH = ACK_HEADER_LENGTH + 14 * 8  # bit

HELLO_PACKET_PAYLOAD_LENGTH = 256  # bit
HELLO_PACKET_LENGTH = IP_HEADER_LENGTH + MAC_HEADER_LENGTH + PHY_HEADER_LENGTH + HELLO_PACKET_PAYLOAD_LENGTH

# define the range of packet_id of different types of packets
GL_ID_HELLO_PACKET = 10000
GL_ID_ACK_PACKET = 20000
GL_ID_VF_PACKET = 30000
GL_ID_GRAD_MESSAGE = 40000
GL_ID_CHIRP_PACKET = 50000

# ------------------ physical layer parameters ------------------- #
BIT_RATE = IEEE_802_11['bit_rate']
BIT_TRANSMISSION_TIME = 1/BIT_RATE * 1e6
BANDWIDTH = IEEE_802_11['bandwidth']
SENSING_RANGE = 600  # in meter, defines the area where a sending node can disturb a transmission from a third node

# --------------------- mac layer parameters --------------------- #
SLOT_DURATION = IEEE_802_11['slot_duration']
SIFS_DURATION = IEEE_802_11['SIFS']
DIFS_DURATION = SIFS_DURATION + (2 * SLOT_DURATION)
CW_MIN = 31
ACK_TIMEOUT = 0.1 * 1e6  # maximum waiting time for ACK (0.1 s)
MAX_RETRANSMISSION_ATTEMPT = 5
