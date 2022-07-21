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

ptn = re.compile('.*_15-32_Manager.txt')
src = '/Users/xirhxq/Documents/BIT/2022_Spring_Summer/UAV_Inst/MBZIRC2023/coor_ws/coor_ws/src/search/data/'
files = os.listdir(src)

txt_files = [src + f for f in files if re.match(ptn, f)]

txt_files.sort(key=lambda fp: os.path.getctime(fp), reverse=True)

newest = txt_files[0]
print(newest)

fig = plt.figure()
ax = plt.axes(projection='3d')

color_list = plt.cm.tab20

n = 1000
data = pd.read_csv(newest, sep='\t')
data = data.drop(data.head(40).index)

max_x, min_x, max_y, min_y, max_z, min_z = -2000, 2000, -2000, 2000, -2000, 2000
l = []
for i in range(1, 11):
    uavnum = str(i)
    max_x = max(max_x, max(data['suav_' + uavnum + '_x']))
    min_x = min(min_x, min(data['suav_' + uavnum + '_x']))
    max_y = max(max_y, max(data['suav_' + uavnum + '_y']))
    min_y = min(min_y, min(data['suav_' + uavnum + '_y']))
    max_z = max(max_z, max(data['suav_' + uavnum + '_z']))
    min_z = min(min_z, min(data['suav_' + uavnum + '_z']))
    tmp_l = ax.plot(data['suav_' + uavnum + '_x'], data['suav_' + uavnum + '_y'], data['suav_' + uavnum + '_z'],
                    color=color_list(i), label='sUAV #' + uavnum, alpha=0.8)
    l.append(tmp_l[0])


ax.legend(loc=1, frameon=True, fontsize=7, ncol=2, edgecolor='grey')

min_x, max_x = 1.1 * min_x - 0.1 * max_x, 1.1 * max_x - 0.1 * min_x
min_y, max_y = 1.1 * min_y - 0.1 * max_y, 1.1 * max_y - 0.1 * min_y
min_z, max_z = 1.1 * min_z - 0.1 * max_z, 1.1 * max_z - 0.1 * min_z

ax.set_xlim(min_x, max_x)
ax.set_ylim(min_y, max_y)
ax.set_zlim(min_z, max_z)
ax.view_init(0, -60)
plt.gca().set_box_aspect((max_x - min_x, max_y - min_y,  (max_z - min_z)))
plt.title('sUAV Trajectory')

def update(num):
    # print("----------------------{}".format(num))
    for i in range(1, 11):
        uavnum = str(i)
        # print(data['suav_' + uavnum + '_x'][:num].values.tolist())
        l[i-1].set_data(data['suav_' + uavnum + '_x'][0:100 * num + 1: 100].values.tolist(),
                        data['suav_' + uavnum + '_y'][0:100 * num + 1: 100].values.tolist())
        l[i-1].set_3d_properties(data['suav_' + uavnum + '_z'][0:100 * num + 1: 100].values.tolist())
    return l


ani = animation.FuncAnimation(fig, update, int(len(data['suav_1_x']) / 100), interval=10, blit=False)

ani.save(newest[:-4] + '.mp4', writer='ffmpeg', fps=15)
# plt.savefig(newest[:-4] + '.png')
# plt.show()