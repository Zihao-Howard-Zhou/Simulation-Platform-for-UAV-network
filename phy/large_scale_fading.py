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


def sinr_calculator(my_drone, main_drones_list, all_transmitting_drones_list):
    """
    calculate signal to signal-to-interference-plus-noise ratio
    :param my_drone: receiver drone
    :param main_drones_list: list of drones that wants to transmit packet to receiver
    :param all_transmitting_drones_list: list of all drones currently transmitting packet
    :return: list of sinr of each main drone
    """

    simulator = my_drone.simulator
    transmit_power = config.TRANSMITTING_POWER
    noise_power = config.NOISE_POWER

    sinr_list = []  # record the sinr of all transmitter
    receiver = my_drone

    for transmitter_id in main_drones_list:
        transmitter = simulator.drones[transmitter_id]
        interference_list = all_transmitting_drones_list[:]
        interference_list.remove(transmitter_id)

        main_link_path_loss = general_path_loss(receiver, transmitter)
        receive_power = transmit_power * main_link_path_loss
        interference_power = 0

        if len(interference_list) != 0:
            for interference_id in interference_list:
                interference = simulator.drones[interference_id]
                interference_link_path_loss = general_path_loss(receiver, interference)
                interference_power += transmit_power * interference_link_path_loss

        sinr = 10 * math.log10(receive_power / (noise_power + interference_power))
        sinr_list.append(sinr)

    return sinr_list


def general_path_loss(receiver, transmitter):
    """
    general path loss model
    :param receiver: the drone that receives the packet
    :param transmitter: the drone that sends the packet
    :return: path loss
    """

    c = config.LIGHT_SPEED
    fc = config.CARRIER_FREQUENCY
    alpha = 2  # path loss exponent

    distance = euclidean_distance(receiver.coords, transmitter.coords)

    if distance != 0:
        path_loss = (c / (4 * math.pi * fc * distance)) ** alpha
    else:
        path_loss = 1

    return path_loss

