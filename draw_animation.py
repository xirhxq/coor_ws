from cProfile import label
from codecs import ascii_encode
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

target = 'search' # 'search' or 'base'
scale = 'whole' # 'whole' or 'tight'

total_num = 10 if target == 'search' else 6

tic = time.time()
ptn = re.compile('.*_Manager.txt')
src = 'src/' + target + '/data/'

txt_files = []
for dp, dn, fn in os.walk(src):
    for f in fn:
        if re.match(ptn, f):
            txt_files.append(os.path.join(dp, f))

# txt_files.sort(key=lambda fp: os.path.getctime(fp), reverse=True)
txt_files.sort(reverse=True)

newest = txt_files[0]
print(newest)

fig = plt.figure(dpi=300)
ax = plt.axes(projection='3d')

color_list = plt.cm.tab20

n = 1000
data = pd.read_csv(newest, sep='\t')

l = [target[0] + 'uav_' + str(i) + '_' + chr(j) for i in range(1, total_num + 1) for j in range(ord('x'), ord('z') + 1)]

real_l = [i for i in l if data[i].any(axis=0)]
print(real_l)

data = data.loc[(data[real_l]!=0).all(axis=1), :]
print(data)

skip_num = 20

total_length = int(len(data[target[0] + 'uav_1_x']) / skip_num)

max_x, min_x, max_y, min_y, max_z, min_z = -2000, 2000, -2000, 2000, -2000, 2000
l = []
for i in range(1, total_num + 1):
    uavnum = str(i)
    tmp_l = ax.plot(data[target[0] + 'uav_' + uavnum + '_x'], data[target[0] + 'uav_' + uavnum + '_y'], data[target[0] + 'uav_' + uavnum + '_z'],
                    color=color_list(i), label=target[0] + 'UAV #' + uavnum, alpha=0.8)
    l.append(tmp_l[0])
    if abs(min(data[target[0] + 'uav_' + uavnum + '_x'])) < 10:
        continue
    max_x = max(max_x, max(data[target[0] + 'uav_' + uavnum + '_x']))
    min_x = min(min_x, min(data[target[0] + 'uav_' + uavnum + '_x']))
    max_y = max(max_y, max(data[target[0] + 'uav_' + uavnum + '_y']))
    min_y = min(min_y, min(data[target[0] + 'uav_' + uavnum + '_y']))
    max_z = max(max_z, max(data[target[0] + 'uav_' + uavnum + '_z']))
    min_z = min(min_z, min(data[target[0] + 'uav_' + uavnum + '_z']))

l_vessel = []
for i in range(7):
    vsl_name = 'vessel_' + chr(ord('a') + i)
    tmp_l = ax.plot(data[vsl_name + '_x'], data[vsl_name + '_y'], data[vsl_name + '_z'], color=color_list(i), label=vsl_name, alpha=0.8)
    l_vessel.append(tmp_l[0])

ax.legend(loc=1, frameon=True, fontsize=5, ncol=2, edgecolor='grey')

min_x, max_x = 1.1 * min_x - 0.1 * max_x, 1.1 * max_x - 0.1 * min_x
min_y, max_y = 1.1 * min_y - 0.1 * max_y, 1.1 * max_y - 0.1 * min_y
min_z, max_z = 1.1 * min_z - 0.1 * max_z, 1.1 * max_z - 0.1 * min_z

if target[0] == 'b':
    max_x = 0

if scale == 'whole':
    max_x = 3162.28 / 2
    min_x = -max_x
    max_y = max_x
    min_y = -max_y

ax.set_xlim(min_x, max_x)
ax.set_ylim(min_y, max_y)
ax.set_zlim(min_z, max_z)
ax.view_init(30, -60)
plt.gca().set_box_aspect((max_x - min_x, max_y - min_y,  2 * (max_z - min_z)))
plt.title(target[0] + 'UAV Trajectory')

def update(num):
    progress_percentage = num / total_length * 100
    elap_time = time.time() - tic
    if num > 0:
        eta = (100 - progress_percentage) / progress_percentage * elap_time
    else:
        eta = np.nan
    print("\r[%s%%]|%s elap: %.2fs eta: %.2fs" % (math.ceil(progress_percentage), "#" * (math.ceil(progress_percentage) // 2),
                                     elap_time, eta), end="")
    for i in range(1, total_num + 1):
        uavnum = str(i)
        # print(data[target[0] + 'uav_' + uavnum + '_x'][0:skip_num * num + 1: skip_num].values.tolist())
        l[i-1].set_data(data[target[0] + 'uav_' + uavnum + '_x'][0:skip_num * num + 1: skip_num].values.tolist(),
                        data[target[0] + 'uav_' + uavnum + '_y'][0:skip_num * num + 1: skip_num].values.tolist())
        l[i-1].set_3d_properties(data[target[0] + 'uav_' + uavnum + '_z'][0:skip_num * num + 1: skip_num].values.tolist())
    for i in range(7):
        vsl_name = 'vessel_' + chr(ord('a') + i)
        l_vessel[i].set_data(data[vsl_name + '_x'][0:skip_num * num + 1: skip_num].values.tolist(),
                             data[vsl_name + '_y'][0:skip_num * num + 1: skip_num].values.tolist())
        l_vessel[i].set_3d_properties(data[vsl_name + '_z'][0:skip_num * num + 1: skip_num].values.tolist())
    return l, l_vessel


ani = animation.FuncAnimation(fig, update, total_length, interval=skip_num, blit=False)
ani.save(newest[:-4] + '.mp4', writer='ffmpeg', fps=15)
print('\nCompleted!!!!!!!!!!!!!!!')
# plt.savefig(newest[:-4] + '.png')
# plt.show()