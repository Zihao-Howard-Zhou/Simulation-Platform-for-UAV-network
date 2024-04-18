import simpy
from utils import config
from simulator.simulator import Simulator

if __name__ == "__main__":
    env = simpy.Environment()
    channel_states = {i: simpy.Resource(env, capacity=1) for i in range(config.NUMBER_OF_DRONES)}
    sim = Simulator(seed=2020, env=env, channel_states=channel_states, n_drones=config.NUMBER_OF_DRONES)

    env.run(until=config.SIM_TIME)
