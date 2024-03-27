import matplotlib.pyplot as plt
import numpy as np

pdr_gpsr = [66.949, 63.7288, 59.4915, 55.08, 53.644]
pdr_opar = [74.83, 79.406, 78.2203, 73.205, 69.83]
pdr_parrot = [65.6779, 55.677, 49.406, 40.508, 34.49]

n_groups = 5
fig, ax = plt.subplots()
index = np.arange(n_groups)
bar_width = 0.2
opacity = 0.5
error_config = {'ecolor': '0.3'}

rects1 = ax.bar(index, pdr_gpsr, bar_width,
                alpha=opacity, color='orangered',
                error_kw=error_config,
                label='GPSR')

rects2 = ax.bar(index + bar_width, pdr_opar, bar_width,
                alpha=opacity, color='green',
                error_kw=error_config,
                label='OPAR')

rects3 = ax.bar(index + bar_width + bar_width, pdr_parrot, bar_width,
                alpha=opacity, color='cornflowerblue',
                error_kw=error_config,
                label='PARRoT')

ax.set_xticks(index + 3 * bar_width / 3)
ax.set_xticklabels(('10', '20', '30', '40', '50'))
ax.legend()
plt.ylim(0, 100)
plt.ylabel("Packet delivery ratio (%)")
plt.xlabel("Drone velocity (m/s)")
plt.show()

e2e_gpsr = [4.0215, 3.74, 4.313, 4.4473, 3.887]
e2e_opar = [4.741, 4.985, 4.606, 4.4614, 4.915]
e2e_parrot = [6.918, 6.233, 7.29, 6.043, 6.776]

fig2, ax = plt.subplots()
index = np.arange(n_groups)
bar_width = 0.2
opacity = 0.5
error_config = {'ecolor': '0.3'}

rects1_1 = ax.bar(index, e2e_gpsr, bar_width,
                alpha=opacity, color='orangered',
                error_kw=error_config,
                label='GPSR')

rects2_2 = ax.bar(index + bar_width, e2e_opar, bar_width,
                alpha=opacity, color='green',
                error_kw=error_config,
                label='OPAR')

rects3_3 = ax.bar(index + bar_width + bar_width, e2e_parrot, bar_width,
                alpha=opacity, color='cornflowerblue',
                error_kw=error_config,
                label='PARRoT')

ax.set_xticks(index + 3 * bar_width / 3)
ax.set_xticklabels(('10', '20', '30', '40', '50'))
ax.legend()
plt.ylabel("Average end-to-end delay (ms)")
plt.xlabel("Drone velocity (m/s)")
plt.show()
