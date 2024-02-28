<div align="center">
  <h1>Simulation Platform for UAV network</h1>

  <img src="https://img.shields.io/badge/Github-%40ZihaoZhouSCUT-blue" height="20">
  <img src="https://img.shields.io/badge/Contribution-Welcome-yellowgreen" height="20">
  <img src="https://img.shields.io/badge/License-MIT-brightgreen" height="20">

  <h3>Make the simulation more friendly to novices with high-fidelity modeling! </h3>
</div>

This Python-based simulation platform can realistically model various components of the UAV network, including the network layer, MAC layer and physical layer, as well as the UAV mobility model, energy model, etc. In addition, the platform can be easily extended to meet the needs of different users and develop their own protocols.

## How to get started?
Firstly, download this project locally:
```
git clone https://github.com/ZihaoZhouSCUT/Simulation-Platform-for-UAV-network.git
```
after that, you need to understand some relevant configuration parameters of the simulation which are in ```utils/config.py```, you can configure the simulated map, the number of the drones, velocities, routing protocols and so on. And then just run ```main.py```. 

It should be noted that the above is just the most basic way of running, you can also go deep into this project to configure more details, such as the mobility pattern of the UAV swarm (in ```mobility``` folder), mac protocol (in ```mac``` floder), routing protocol (in ```routing``` floder) and so on based on your research needs. Additionally, energy models for drones are still under development.

This project gives a more flexible way for you to develop your own routing protocl. To do this requires the following steps:
 * Create a new package under the ```routing``` folder (Don't forget to add ```__init__.py```)
 * The main program of the routing protocol must contain the function: ```def next_hop_selection(self, packet)``` and ```def packet_reception(self, packet, src_drone_id)```
 * After confirming that the code logic is correct, add the relevant classes of the routing protocol you designed in the import part and the enumeration part of the protocol in ```config.py```.
