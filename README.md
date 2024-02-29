<div align="center">
  <h1>Simulation Platform for UAV network</h1>

  <img src="https://img.shields.io/badge/Github-%40ZihaoZhouSCUT-blue" height="20">
  <img src="https://img.shields.io/badge/Contribution-Welcome-yellowgreen" height="20">
  <img src="https://img.shields.io/badge/License-MIT-brightgreen" height="20">

  <h3>Make the simulation more friendly to novices with high-fidelity modeling! </h3>
</div>

This Python-based simulation platform can realistically model various components of the UAV network, including the network layer, MAC layer and physical layer, as well as the UAV mobility model, energy model, etc. In addition, the platform can be easily extended to meet the needs of different users and develop their own protocols.

## Requiremens
- Python 3.3 and up
- Simpy 4.1.1
  
## Installation and usage
Firstly, download this project:
```
git clone https://github.com/ZihaoZhouSCUT/Simulation-Platform-for-UAV-network.git
```
You can even ```run main.py``` directly with one click to get a sneak peek. But we recommend that you go to ```utils/config.py``` to browse and roughly understand the simulation parameters before starting the operation, including: the size of the simulation map, the number of drones, the velocities of drones, the routing protocol adopted and so on.

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

## Contribute
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

 Please make sure to update tests as appropriate.
