import random
from utils import config


def get_random_start_point_3d(sim_seed):
    start_position = []
    for i in range(config.NUMBER_OF_DRONES):
        random.seed(sim_seed + i)
        position_x = random.uniform(50, config.MAP_LENGTH - 50)
        position_y = random.uniform(50, config.MAP_WIDTH - 50)
        position_z = random.uniform(10, config.MAP_HEIGHT - 10)

        start_position.append(tuple([position_x, position_y, position_z]))

    return start_position

