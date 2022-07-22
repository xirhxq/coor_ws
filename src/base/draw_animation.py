from cProfile import label
from random import random
from turtle import color
from matplotlib import projections
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib
import os
import pandas as pd
import re
import numpy as np
import matplotlib.animation as animation
import time
import math

tic = time.time()
ptn = re.compile('.*_Manager.txt')
src = 'src/base/data/'
files = os.listdir(src)

txt_files = [src + f for f in files if re.match(ptn, f)]

txt_files.sort(key=lambda fp: os.path.getctime(fp), reverse=True)

newest = txt_files[0]
print(newest)

fig = plt.figure(dpi=300)
ax = plt.axes(projection='3d')

color_list = plt.cm.tab20

n = 1000
data = pd.read_csv(newest, sep='\t')
data = data.drop(data.head(40).index)

total_length = int(len(data['buav_1_x']) / 100)

max_x, min_x, max_y, min_y, max_z, min_z = -2000, 2000, -2000, 2000, -2000, 2000
l = []
for i in range(1, 7):
    uavnum = str(i)
    max_x = max(max_x, max(data['buav_' + uavnum + '_x']))
    min_x = min(min_x, min(data['buav_' + uavnum + '_x']))
    max_y = max(max_y, max(data['buav_' + uavnum + '_y']))
    min_y = min(min_y, min(data['buav_' + uavnum + '_y']))
    max_z = max(max_z, max(data['buav_' + uavnum + '_z']))
    min_z = min(min_z, min(data['buav_' + uavnum + '_z']))
    tmp_l = ax.plot(data['buav_' + uavnum + '_x'], data['buav_' + uavnum + '_y'], data['buav_' + uavnum + '_z'],
                    color=color_list(i), label='bUAV #' + uavnum, alpha=0.8)
    l.append(tmp_l[0])


ax.legend(loc=1, frameon=True, fontsize=7, ncol=2, edgecolor='grey')

min_x, max_x = 1.1 * min_x - 0.1 * max_x, 1.1 * max_x - 0.1 * min_x
min_y, max_y = 1.1 * min_y - 0.1 * max_y, 1.1 * max_y - 0.1 * min_y
min_z, max_z = 1.1 * min_z - 0.1 * max_z, 1.1 * max_z - 0.1 * min_z

max_x = 0
ax.set_xlim(min_x, max_x)
ax.set_ylim(min_y, max_y)
ax.set_zlim(min_z, max_z)
ax.view_init(0, 0)
plt.gca().set_box_aspect((max_x - min_x, max_y - min_y,  10 * (max_z - min_z)))
plt.title('bUAV Trajectory')

def update(num):
    progress_percentage = num / total_length * 100
    elap_time = time.time() - tic
    if num > 0:
        eta = (100 - progress_percentage) / progress_percentage * elap_time
    else:
        eta = np.nan
    print("\r[%s%%]|%s elap: %.2fs eta: %.2fs" % (math.ceil(progress_percentage), "#" * (math.ceil(progress_percentage) // 2),
                                     elap_time, eta), end="")
    for i in range(1, 7):
        uavnum = str(i)
        # print(data['buav_' + uavnum + '_x'][0:100 * num + 1: 100].values.tolist())
        l[i-1].set_data(data['buav_' + uavnum + '_x'][0:100 * num + 1: 100].values.tolist(),
                        data['buav_' + uavnum + '_y'][0:100 * num + 1: 100].values.tolist())
        l[i-1].set_3d_properties(data['buav_' + uavnum + '_z'][0:100 * num + 1: 100].values.tolist())
    return l


ani = animation.FuncAnimation(fig, update, total_length, interval=10, blit=False)
ani.save(newest[:-4] + '.mp4', writer='ffmpeg', fps=15)
print('\nCompleted!!!!!!!!!!!!!!!')
# plt.savefig(newest[:-4] + '.png')
# plt.show()