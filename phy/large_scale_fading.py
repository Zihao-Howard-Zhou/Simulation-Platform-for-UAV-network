import math
from utils import config
from utils.util_function import euclidean_distance

def general_path_loss(my_drone, previous_drone):
    """
    general path loss model
    :param my_drone: the drone that receives the packet
    :param previous_drone: the drone that sends the packet
    :return: the signal-to-noise ratio
    """

    transmit_power = 10 * math.log10(config.TRANSMITTING_POWER)
    noise_power = 10 * math.log10(config.NOISE_POWER)

    distance = euclidean_distance(my_drone.coords, previous_drone.coords)
    c = config.LIGHT_SPEED
    fc = config.CARRIER_FREQUENCY
    alpha = 2

    if my_drone.identifier != previous_drone.identifier:
        path_loss = alpha * 10 * math.log10(4 * math.pi * fc * distance / c)
    else:
        path_loss = 0

    snr = transmit_power - path_loss - noise_power

    return snr

