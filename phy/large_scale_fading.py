import math
import logging
from utils import config
from utils.util_function import euclidean_distance


# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=logging.DEBUG
                    )

def sinr_calculator_general_path_loss(my_drone, previous_drones_list, all_transmitting_drones_list):
    simulator = my_drone.simulator
    transmit_power = config.TRANSMITTING_POWER
    noise_power = config.NOISE_POWER

    c = config.LIGHT_SPEED
    fc = config.CARRIER_FREQUENCY
    alpha = 2

    interference_strength = 0.0
    sinr_list = []
    for main_drone_idx in previous_drones_list:
        main_drone = simulator.drones[main_drone_idx]
        distance = euclidean_distance(my_drone.coords, main_drone.coords)

        path_loss = (c / (4 * math.pi * fc * distance))**alpha
        receive_power = transmit_power * path_loss
        # path_loss = alpha * 10 * math.log10(4 * math.pi * fc * distance / c)
        # receive_power = transmit_power - path_loss

        for interference_drone_idx in all_transmitting_drones_list:
            if interference_drone_idx != main_drone_idx and interference_drone_idx != my_drone.identifier:
                interference_drone = simulator.drones[interference_drone_idx]
                distance = euclidean_distance(my_drone.coords, interference_drone.coords)

                path_loss = (c / (4 * math.pi * fc * distance)) ** alpha
                interference_power = transmit_power * path_loss
                # path_loss = alpha * 10 * math.log10(4 * math.pi * fc * distance / c)
                # interference_power = transmit_power - path_loss
                interference_strength += interference_power

        sinr = 10*math.log10(receive_power / (interference_strength + noise_power))
        logging.info('Main node id: %s, my_drone is: %s, sinr is: %s',
                main_drone_idx, my_drone.identifier, sinr)
        sinr_list.append(sinr)
        # print('My drone is: ', my_drone.identifier, ' SINR of main node: ', main_drone, ' is: ', sinr)

    return sinr_list

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

