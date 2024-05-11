import logging
from collections import defaultdict
from utils import config


# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=config.LOGGING_LEVEL
                    )


class QldvTable:
    def __init__(self, env, my_drone):
        self.env = env
        self.my_drone = my_drone
        self.entry_life_time = 0.25 * 1e6  # unit: us (0.25s)

        self.neighbor_table = defaultdict(list)  # used to record the available (valid) neighbor
        self.q_table = {self.my_drone.identifier: {self.my_drone.identifier: 1}}

        self.learning_rate = 1

    def regular_update(self, packet, current_time):
        """
        Regular update when drone receives hello packet from its neighbor
        :param packet: hello packet
        :param current_time: receiving time
        :return: none
        """

        action_id = packet.src_drone.identifier  # potential action
        self.neighbor_table[action_id] = [current_time]  # mark this action (neighbor) as "valid"

        if action_id is not self.my_drone.identifier:  # the hello packet is not broadcast by myself
            for item in packet.update_material:
                dst_id = item[0]  # destination
                max_q = item[1]  # max Q-value
                action_for_max_q = item[2]  # a_(t+1) corresponding to max Q-value

                gamma = 0.75
                if action_id is dst_id:  # means that destination is my neighbor
                    f = 1  # completion indicator
                    reward = 1
                else:
                    f = 0
                    reward = 0

                # prevent from routing loop
                if (action_for_max_q != self.my_drone.identifier) and (dst_id != self.my_drone.identifier):
                    if dst_id not in self.q_table.keys():  # no Q-value was recorded to this destination
                        self.q_table[dst_id] = {}  # create a new record
                        self.q_table[dst_id][action_id] = self.learning_rate * (reward + gamma * (1 - f) * max_q)
                    elif action_id not in self.q_table[dst_id].keys():  # a new route to this destination
                        self.q_table[dst_id][action_id] = self.learning_rate * (reward + gamma * (1 - f) * max_q)
                    elif action_id in self.q_table[dst_id].keys():  # regular update
                        self.q_table[dst_id][action_id] = ((1 - self.learning_rate) * self.q_table[dst_id][action_id] +
                                                           self.learning_rate * (reward + gamma * (1 - f) * max_q))
                else:
                    pass
        else:
            pass

    def urgent_update(self, packet):
        """
        When a broken link is detected, an emergency packet should be broadcast to advertise the breaking
        :param packet: emergency packet
        :return next_error_update_material: used to further broadcast
        """

        next_error_update_material = []
        action_id = packet.src_drone.identifier  # potential action
        error_update_material = packet.error_update_material

        for item in error_update_material:
            dst_id = item[0]  # destination
            indicator = item[1]  # indicate if this action still has other ways to destination
            max_q = item[2]  # max Q-value
            action_for_max_q = item[3]  # a_(t+1) corresponding to the max Q-value
            if indicator == "no route":
                if (dst_id in self.q_table.keys()) and (action_id in self.q_table[dst_id].keys()):
                    del self.q_table[dst_id][action_id]

                    # check if I have any other route to "dst_id"
                    if not bool(self.q_table[dst_id]):
                        # no other route to "dst_id"
                        error_msg = [dst_id, 'no route', None, None]
                        error_update_material.append(error_msg)
                    else:
                        alternative_max_q = -1000
                        alternative_action_for_max_q = None
                        for alternative_action_id in list(self.q_table[dst_id].keys()):
                            if self.q_table[dst_id][alternative_action_id] > alternative_max_q:
                                alternative_max_q = self.q_table[dst_id][alternative_action_id]
                                alternative_action_for_max_q = alternative_action_id

                        error_msg = [dst_id, 'has route', alternative_max_q, alternative_action_for_max_q]
                        next_error_update_material.append(error_msg)

            elif indicator == "has route":
                if action_for_max_q != self.my_drone.identifier:
                    if (dst_id in self.q_table.keys()) and (action_id in self.q_table[dst_id].keys()):
                        gamma = 0.75
                        if action_id is dst_id:  # means that destination is my neighbor
                            f = 1  # completion indicator
                            reward = 1
                        else:
                            f = 0
                            reward = 0

                        self.q_table[dst_id][action_id] = self.learning_rate * (reward + gamma * (1 - f) * max_q)

                        alternative_max_q = -1000
                        alternative_action_for_max_q = None
                        for alternative_action_id in list(self.q_table[dst_id].keys()):
                            if self.q_table[dst_id][alternative_action_id] > alternative_max_q:
                                alternative_max_q = self.q_table[dst_id][alternative_action_id]
                                alternative_action_for_max_q = alternative_action_id

                        error_msg = [dst_id, 'has route', alternative_max_q, alternative_action_for_max_q]
                        next_error_update_material.append(error_msg)
                else:  # it means that routing loop occurs
                    if (dst_id in self.q_table.keys()) and (action_id in self.q_table[dst_id].keys()):
                        del self.q_table[dst_id][action_id]

        return next_error_update_material

    def get_update_material(self):
        update_material = []
        for key1 in self.q_table.keys():  # traverse all recorded destinations
            max_q_for_key1 = -1000
            action_for_max_q = None
            for key2 in self.q_table[key1].keys():  # traverse all available actions
                if self.q_table[key1][key2] > max_q_for_key1:
                    max_q_for_key1 = self.q_table[key1][key2]
                    action_for_max_q = key2

            update_material.append([key1, max_q_for_key1, action_for_max_q])

        return update_material

    def purge(self):
        """
        To detect the broken links, it should be executed periodically
        :return flag, error_update_material
        """

        error_update_material = []
        flag = 0

        for action_id in list(self.neighbor_table):
            updated_time = self.neighbor_table[action_id][-1]
            if updated_time + self.entry_life_time < self.env.now:  # expired
                flag = 1  # indicates that some links have broken
                del self.neighbor_table[action_id]  # delete this neighbor

                for dst_id in list(self.q_table.keys()):
                    if action_id in self.q_table[dst_id].keys():
                        del self.q_table[dst_id][action_id]

                    # check if there is any other route to "dst_id"
                    if not bool(self.q_table[dst_id]):
                        # no other routes to "dst_id"
                        error_msg = [dst_id, 'no route', None, None]
                        error_update_material.append(error_msg)
                    else:
                        alternative_max_q = -1000
                        alternative_action_for_max_q = None
                        for alternative_action_id in list(self.q_table[dst_id].keys()):  # the remaining optional actions
                            if self.q_table[dst_id][alternative_action_id] > alternative_max_q:
                                alternative_max_q = self.q_table[dst_id][alternative_action_id]
                                alternative_action_for_max_q = alternative_action_id

                        error_msg = [dst_id, 'has route', alternative_max_q, alternative_action_for_max_q]
                        error_update_material.append(error_msg)

        return flag, error_update_material

    def best_neighbor(self, dst_drone):
        dst_id = dst_drone.identifier
        next_hop_id = self.my_drone.identifier
        max_q = -1000

        for action_id in self.neighbor_table.keys():  # available neighbor
            if (dst_id in self.q_table.keys()) and (action_id in self.q_table[dst_id].keys()):
                if self.q_table[dst_id][action_id] > max_q:
                    max_q = self.q_table[dst_id][action_id]
                    next_hop_id = action_id

        return next_hop_id
