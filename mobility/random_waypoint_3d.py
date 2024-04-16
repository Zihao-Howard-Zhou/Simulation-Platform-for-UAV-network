import numpy as np
import random
from utils import config
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RandomWaypoint3D:
    def __init__(self, drone):
        self.my_drone = drone

        # generate random waypoint
        self.waypoint_num = 20
        self.waypoint = []
