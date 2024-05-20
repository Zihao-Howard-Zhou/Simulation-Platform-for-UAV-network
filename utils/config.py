import logging

# --------------------- simulation parameters --------------------- #
MAP_LENGTH = 700  # m, length of the map
MAP_WIDTH = 700  # m, width of the map
MAP_HEIGHT = 120  # m, height of the map 120
SIM_TIME = 10 * 1e6  # us, total simulation time (10s)
NUMBER_OF_DRONES = 10  # number of drones in the network
STATIC_CASE = 0
HETEROGENEOUS = 0  # heterogeneous network support (in terms of speed)
LOGGING_LEVEL = logging.INFO

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
TRANSMITTING_POWER = 1  # Watt
LIGHT_SPEED = 3 * 1e8  # light speed (m/s)
CARRIER_FREQUENCY = 1 * 1e9  # carrier frequency (Hz)
NOISE_POWER = 4*1e-9  # noise power (Watt)
RADIO_SWITCHING_TIME = 100  # us, the switching time of the transceiver mode
SNR_THRESHOLD = 2  # dB
RADIO_SENSITIVITY = 1e-10  # power under which signal is not sensed

# ---------------------- packet parameters ----------------------- #
MAX_TTL = 15
PACKET_LIFETIME = 10 * 1e6  # 10s
PACKET_HEADER_LENGTH = 128  # bit
DATA_PACKET_PAYLOAD_LENGTH = 1024 * 8  # bit
DATA_PACKET_LENGTH = PACKET_HEADER_LENGTH + DATA_PACKET_PAYLOAD_LENGTH

ACK_PACKET_LENGTH = 128  # bit

HELLO_PACKET_HEADER_LENGTH = 128  # bit
HELLO_PACKET_PAYLOAD_LENGTH = 256  # bit
HELLO_PACKET_LENGTH = HELLO_PACKET_HEADER_LENGTH + HELLO_PACKET_PAYLOAD_LENGTH

# ------------------ physical layer parameters ------------------- #
BIT_RATE = 54 * 1e6  # 54 Mbit/s
BIT_TRANSMISSION_TIME = 1/BIT_RATE * 1e6
BANDWIDTH = 20 * 1e6  # 20 MHz
SENSING_RANGE = 750

# --------------------- mac layer parameters --------------------- #
SLOT_DURATION = 50  # 50 microseconds, 802.11g 2.4 GHz
SIFS_DURATION = 28  # 28 microseconds, 802.11g 2.4 GHz
DIFS_DURATION = SIFS_DURATION + (2 * SLOT_DURATION)  # 128 microseconds
MAC_HEADER_LENGTH = 34*8  # 34 byte fixed fields of a mac packet
MAX_MAC_PAYLOAD_LENGTH = 2312*8
ACK_LENGTH = MAC_HEADER_LENGTH
CW_MIN = 16
CW_MAX = 1024
ACK_TIMEOUT = 1 * 1e3  # maximum waiting time for ACK (1ms)
MAX_RETRANSMISSION_ATTEMPT = 5
