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

ptn = re.compile('.*.txt')
src = '/home/ps/coor_ws/src/search/data/'
files = os.listdir(src)

txt_files = [src + f for f in files if re.match(ptn, f)]

txt_files.sort(key=lambda fp: os.path.getctime(fp), reverse=True)

newest = txt_files[0]

newest_files = [f for f in txt_files if newest[:-5] == f[:-5]]
newest_files.sort()

print(newest_files)

print('find {}'.format(newest))

fig = plt.figure()
ax = plt.axes(projection='3d')

color_list = plt.cm.tab20
print(type(color_list))
for i in range(4):
    print(color_list(i))

max_x, min_x, max_y, min_y, max_z, min_z = -2000, 2000, -2000, 2000, -2000, 2000
uavnums = ''
for f in newest_files:
    data = pd.read_csv(f, sep='\t')
    print("sUAV {}".format(f[-5]))
    uavnums = uavnums + f[-5]
    data.drop(labels=0, axis=0, inplace=True)
    print(data.head())
    ax.plot3D(data['uav_pos_x'], data['uav_pos_y'], data['uav_pos_z'],
          color=color_list(int(f[-5])), label="sUAV {}".format(f[-5]))
    max_x = max(max_x, max(data['uav_pos_x']))
    min_x = min(min_x, min(data['uav_pos_x']))
    max_y = max(max_y, max(data['uav_pos_y']))
    min_y = min(min_y, min(data['uav_pos_y']))
    max_z = max(max_z, max(data['uav_pos_z']))
    min_z = min(min_z, min(data['uav_pos_z']))

ax.legend(loc='best')
ax.set_xlim(min_x, max_x)
ax.set_ylim(min_y, max_y)
ax.set_zlim(min_z, max_z)
plt.gca().set_box_aspect((max_x - min_x, max_y - min_y, 5 * (max_z - min_z)))
plt.title('sUAV Trajectory')

plt.savefig(newest[:-5] + uavnums + '.png')
plt.show()