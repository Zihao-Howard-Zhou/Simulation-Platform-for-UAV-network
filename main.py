import simpy
from utils import config
from simulator.simulator import Simulator

if __name__ == "__main__":
    n_drones = 5
    env = simpy.Environment()
    channel_states = {i: simpy.Resource(env, capacity=1) for i in range(25)}
    sim = Simulator(seed=2025, env=env, channel_states=channel_states, n_drones=25)

    env.run(until=config.SIM_TIME)
