from collections import defaultdict


class GradCostTable:
    """
    Cost table of GRAd (Gradient Routing in ad hoc networks) (v1.0)

    Type of the cost table: dictionary
    the format of the cost table is:
    {target_id 1: [seq_#, est_cost1, updated time1], target_id 2: [seq_#, est_cost2, updated time2],...}
    Explanation:
    1) "target_id": is the identifier of a remote drone to which this cost entry refers
    2) "seq_#": the highest sequence number received so far in a message from "target_id". When compared against the
        seq_# of a newly arrived message, this field discriminates between a new message and a copy of a previously
        received message
    3) "est_cost": the most recent and best estimated cost (number of hops in this version) for delivering a message
        to "target_id"
    4) "updated time": this field is used to determine if the entry is expired

    The cost table can answer two question:
    1) "Is this message a copy of a previously received message?" This is determined by comparing the sequence number
        in the incoming message against the last sequence number recorded in the cost table
    2) "What is the estimated cost of sending a message to a certain target drone?" In cost table, each "target_id" is
        associated with "est_cost"

    References:
        [1] Poor R. Gradient routing in ad hoc networks[J]. 2000.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/4/20
    Updated at: 2024/4/20
    """

    def __init__(self, env, my_drone):
        self.env = env
        self.my_drone = my_drone
        self.cost_table = defaultdict(list)
        self.entry_life_time = 5 * 1e6  # unit: us (2s)

    # determine if the cost table is empty
    def is_empty(self):
        return not bool(self.cost_table)

    # get the estimated cost for "target_id"
    def get_est_cost(self, target_id):
        if target_id not in self.cost_table.keys():
            raise RuntimeError('This item is not in the cost table')
        else:
            return self.cost_table[target_id][1]

    # get the updated time of certain entry
    def get_updated_time(self, drone_id):
        if drone_id not in self.cost_table.keys():
            raise RuntimeError('This item is not in the cost table')
        else:
            return self.cost_table[drone_id][-1]

    # delete the specified item
    def remove_entry(self, drone_id):
        del self.cost_table[drone_id]

    # remove the expired item
    def purge(self):
        if self.is_empty():
            # it means that the neighbor table is empty
            return

        for key in list(self.cost_table):
            updated_time = self.get_updated_time(key)
            if updated_time + self.entry_life_time < self.env.now:  # expired
                self.remove_entry(key)

    # update entry, core function
    def update_entry(self, grad_message, cur_time):
        originator_id = grad_message.originator.identifier  # remote drone
        seq_num = grad_message.seq_num
        accrued_cost = grad_message.accrued_cost

        if originator_id is not self.my_drone.identifier:
            if originator_id not in self.cost_table.keys():  # no matching entry is found
                self.cost_table[originator_id] = [seq_num, accrued_cost, cur_time]  # create a new entry
            elif self.cost_table[originator_id][0] < seq_num:  # incoming message is fresher
                self.cost_table[originator_id] = [seq_num, accrued_cost, cur_time]  # entry is updated
            elif accrued_cost < self.cost_table[originator_id][1]:
                self.cost_table[originator_id][1] = accrued_cost
                self.cost_table[originator_id][2] = cur_time
        else:
            pass

    # used to determine if it has a route for delivering a data packet
    def has_entry(self, target_id):
        has_route = False
        if target_id in self.cost_table.keys():
            has_route = True

        return has_route

    def print_cost_table(self):
        print('|----------Neighbor Table of: ', self.my_drone.identifier, ' ----------|')
        for key in self.cost_table.keys():
            print('Target_id: ', key, ', seq_#: ', self.cost_table[key][0], ', est_cost: ', self.cost_table[key][1],
                  ', updated time is: ', self.cost_table[key][2])
        print('|-----------------------------------------------------------------|')
