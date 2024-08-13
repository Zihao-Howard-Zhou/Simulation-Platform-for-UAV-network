class IEEE_802_11:
    def __init__(self):
        # IEEE 802.11a
        self.a = {'carrier_frequency': 5 * 1e9,  # 5 GHz
                  'bit_rate': 54 * 1e6,  # 54 Mbps
                  'bandwidth': 20 * 1e6,  # 20 MHz
                  'slot_duration': 9,  # microseconds
                  'SIFS': 16}

        # IEEE 802.11b
        self.b = {'carrier_frequency': 2.4 * 1e9,  # 2.4 GHz
                  'bit_rate': 11 * 1e6,  # 11 Mbps
                  'bandwidth': 20 * 1e6,  # 20 MHz
                  'slot_duration': 20,  # microseconds
                  'SIFS': 10}

        # IEEE 802.11g
        self.g = {'carrier_frequency': 2.4 * 1e9,  # 2.4 GHz
                  'bit_rate': 54 * 1e6,  # 54 Mbps
                  'bandwidth': 20 * 1e6,  # 20 MHz
                  'slot_duration': 9,  # microseconds
                  'SIFS': 10}
