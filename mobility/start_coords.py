import random
from utils import config


def get_random_start_point_3d(sim_seed):
    start_position = []
    for i in range(config.NUMBER_OF_DRONES):
        random.seed(sim_seed + i)
        position_x = random.uniform(1, config.MAP_LENGTH - 1)
        position_y = random.uniform(1, config.MAP_WIDTH - 1)
        position_z = random.uniform(1, config.MAP_HEIGHT - 1)

        start_position.append(tuple([position_x, position_y, position_z]))

    return start_position
