from collections import defaultdict


class DsdvRoutingTable:
    """
    Routing table of DSDV (Destination-Sequenced Distance Vector)

    type of the routing table: dictionary
    the structure of the routing table is:
    {dst1: [next hop, metric (hop count), seq_num of dst1, updated time1],
     dst2: [next hop, metric (hop count), seq_num of dst2, updated time2],
     ...}

    Attributes:
        env: simulation environment
        routing_table: dictionary in python, core member
        entry_life_time: lifetime of each item in the neighbor table

    References:
        [1] Perkins, C. E., and Bhagwat, P.,"Highly dynamic destination-sequenced distance-vector routing (DSDV) for
            mobile computer," ACM SIGCOMM computer communication review, vol. 24, no. 4, pp. 234-244, 1994.
        [2] He. G, "Destination-sequenced distance vector (DSDV) protocol," Networking Laboratory, Helsinki University
            of Technology, 135, pp. 1-9, 2002.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/4/14
    Updated at: 2024/4/14
    """

    def __init__(self, env, my_drone):
        self.env = env
        self.my_drone = my_drone
        self.routing_table = defaultdict(list)
        self.routing_table[self.my_drone.identifier] = [self.my_drone.identifier, 0, self.my_drone.identifier*2, self.env.now]
        self.entry_life_time = 5 * 1e6  # unit: us (5s)

    # determine if the routing table is empty
    def is_empty(self):
        return not bool(self.routing_table)

    # get the updated time of certain item
    def get_updated_time(self, drone_id):
        if drone_id not in self.routing_table.keys():
            raise RuntimeError('This item is not in the routing table')
        else:
            return self.routing_table[drone_id][-1]

    # update item according to the receiving packet
    def update_item(self, packet, cur_time):
        for dst_id in packet.routing_table.keys():
            src_drone = packet.src_drone
            metric = packet.routing_table[dst_id][1]
            seq_num = packet.routing_table[dst_id][2]
            if dst_id not in self.routing_table.keys():
                # 获取该dst_id的seq_num
                self.routing_table[dst_id] = [src_drone.identifier, metric+1, seq_num, cur_time]
            elif seq_num > self.routing_table[dst_id][2]:
                self.routing_table[dst_id] = [src_drone.identifier, metric+1, seq_num, cur_time]
            elif seq_num == self.routing_table[dst_id][2]:
                if metric < self.routing_table[dst_id][1]:
                    self.routing_table[dst_id] = [src_drone.identifier, metric+1, seq_num, cur_time]
            else:
                pass

    # delete the specified item
    def remove_item(self, drone_id):
        del self.routing_table[drone_id]

    # remove the expired item
    def purge(self):
        if not bool(self.routing_table):
            # it means that the neighbor table is empty
            return

        for key in list(self.routing_table):
            updated_time = self.get_updated_time(key)
            if updated_time + self.entry_life_time < self.env.now:  # expired
                self.remove_item(key)

    # determine if it has the item to certain destination
    def has_entry(self, dst_id):
        if dst_id not in self.routing_table.keys():
            next_hop_id = self.my_drone.identifier
        else:
            # get the next hop to the destination
            next_hop_id = self.routing_table[dst_id][0]

        return next_hop_id

    # print routing table
    def print_neighbor(self, my_drone):
        print('|----------Routing Table of: ', my_drone.identifier, ' ----------|')
        for key in self.routing_table.keys():
            print('Dst_id: ', key, ', next hop is: ', self.routing_table[key][0],
                  ', metric is: ', self.routing_table[key][1],
                  'seq_num (dst_id) is: ', self.routing_table[key][2],
                  'updated time is: ', self.routing_table[key][3])
        print('|-----------------------------------------------------------------|')
