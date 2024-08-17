import simpy
from utils import config
from simulator.simulator import Simulator

"""
  _______  ___       ___  ___  _____  ___    _______  ___________  
 /"     "||"  |     |"  \/"  |(\"   \|"  \  /"     "|("     _   ") 
(: ______)||  |      \   \  / |.\\   \    |(: ______) )__/  \\__/  
 \/    |  |:  |       \\  \/  |: \.   \\  | \/    |      \\_ /     
 // ___)   \  |___    /   /   |.  \    \. | // ___)_     |.  |     
(:  (     ( \_|:  \  /   /    |    \    \ |(:      "|    \:  |     
 \__/      \_______)|___/      \___|\____\) \_______)     \__|     
                                                                                                                                                                                                                                      
"""

if __name__ == "__main__":
    env = simpy.Environment()
    channel_states = {i: simpy.Resource(env, capacity=1) for i in range(config.NUMBER_OF_DRONES)}
    sim = Simulator(seed=2025, env=env, channel_states=channel_states, n_drones=config.NUMBER_OF_DRONES)

    env.run(until=config.SIM_TIME)
