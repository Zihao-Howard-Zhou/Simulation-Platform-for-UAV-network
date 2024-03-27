<div align="center">
  <h1>Simulation Platform for UAV network</h1>

  <img src="https://img.shields.io/badge/Github-%40ZihaoZhouSCUT-blue" height="20">
  <img src="https://img.shields.io/badge/Contribution-Welcome-yellowgreen" height="20">
  <img src="https://img.shields.io/badge/License-MIT-brightgreen" height="20">

  <h3>Make the simulation more friendly to novices with high-fidelity modeling! </h3>
</div>

This Python-based simulation platform can realistically model various components of the UAV network, including the network layer, MAC layer and physical layer, as well as the UAV mobility model, energy model, etc. In addition, the platform can be easily extended to meet the needs of different users and develop their own protocols.

## Requiremens
- Python >= 3.3 
- Simpy >= 4.1.1
  
## Installation and usage
Firstly, download this project:
```
git clone https://github.com/ZihaoZhouSCUT/Simulation-Platform-for-UAV-network.git
```
You can even ```run main.py``` directly with one click to get a sneak peek. But we recommend that you go to ```utils/config.py``` to browse and roughly understand the simulation parameters before starting the operation, including: the size of the simulation map, the number of drones, the velocities of drones, the routing protocol adopted and so on.

## Project structure
<div align="center">
<img src="https://github.com/ZihaoZhouSCUT/Simulation-Platform-for-UAV-network/blob/master/img/structure.png" width="800px">
</div>

## Module overview
### Mobility model
Mobility model is one of the most important mudules to show the characteristics of UAV network more realistically. In this project, **Gauss-Markov 3D mobility model** and **Random Walk 3D mobility model** have been implemented. Specifically, since it is quite difficult to achieve continuous movement of drone in discrete time simulation, we set a "position_update_interval" to update the positions of drones periodically, that is, it is assumed that the drone moves continuously within this time interval. If the time interval "position_update_interval" is smaller, the simulation accuracy will be higher, but the corresponding simulation time will be longer. There will be a trade-off. Besides, the time interval that drone updates its direction can also be set manually. The trajectories of a single drone within 100 second of the simulation under the two mobility models are shown as follows:

<div align="center">
<img src="https://github.com/ZihaoZhouSCUT/Simulation-Platform-for-UAV-network/blob/master/img/mobility_model.png" width="800px">
</div>

### Energy model
The energy model of our platform is based on the work of Y. Zeng, et al. The figure below shows the power required for different drone flying speeds. The energy consumption is equal to the power multiplied by the flight time at this speed.
<div align="center">
<img src="https://github.com/ZihaoZhouSCUT/Simulation-Platform-for-UAV-network/blob/master/img/energy_model.png", width="400px">
</div>

### Routing protocol
In our current version, we support **GPSR (greedy perimeter stateless routing)**, **OPAR (optimized predictive and adaptive routing)** and **PARRoT (predictive ad-hoc routing fueled by reinforcement learning and trajectory knowledge)** for users to test. Among them, GPSR chooes the next hop solely based on the location information. By modelling the routing problem as a binary linear programming, OPAR takes hop count and link lifetime into account when selecting the next hop. Introducing reinforcement learning (Q-Learning) into routing protocol, PARRoT chooses next hop according to the maximum Q-value, but it needs time for Q-value to converage. We preliminarily tested the performance of three routing protocols by changing the flight speed of the drone. Packet delivery ratio (PDR) and average end-to-end (E2E) delay were recorded.

<div align="center">
<img src="https://github.com/ZihaoZhouSCUT/Simulation-Platform-for-UAV-network/blob/master/img/routing_model.png", width="800px">
</div>

## Design your own protocol
Our simulation platform can be expanded based on your research needs, including designing your own mobility model of drones (in ```mobility``` folder), mac protocol (in ```mac``` floder), routing protocol (in ```routing``` floder) and so on. It should be noted that energy model for drones is still under development. Next, we take routing protocols as an example to introduce how users can design their own algorithms.

 * Create a new package under the ```routing``` folder (Don't forget to add ```__init__.py```)
 * The main program of the routing protocol must contain the function: ```def next_hop_selection(self, packet)``` and ```def packet_reception(self, packet, src_drone_id)```
 * After confirming that the code logic is correct, add the relevant classes of the routing protocol you designed in the import part and the enumeration part of the protocol in ```config.py```.
   ```python
   # in config.py
   from routing.your_module_name.your_code_name import your_protocol_class
   
   class RoutingProtocol(Enum):
     your_protocol = your_protocol_class

     @staticmethof
     def keylist():
       return list(map(lambda c: c.name, RoutingProtocol))

   ROUTING_PROTOCOL = RoutingProtocol.your_protocol
   ```

## Contributing
Contributions, issues and feature requests are welcome! 

Currently, the energy model of UAVs (including energy consumption of flight, hovering and information transmission), calculation of physical layer SINR, etc. are still under development. Pull requests are welcomed!

## Show your support
Give a ⭐ is this project helped you!

## License
This project is MIT licensed.
