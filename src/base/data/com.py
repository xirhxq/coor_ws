import os
from cv2 import RETR_LIST
import pandas as pd
import re
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

target = 'base'

ptn = re.compile('.*_Manager.txt')
src = 'src/' + target + '/data/'

txt_files = []
for dp, dn, fn in os.walk(src):
    for f in fn:
        if re.match(ptn, f):
            txt_files.append(os.path.join(dp, f))

txt_files.sort(reverse=True)
newest = txt_files[0]
print(newest)


fig = plt.figure(dpi=300, figsize=(20, 10))
ax = plt.axes()


data = pd.read_csv(newest, sep='\t')
print(data)

dis_ls = data['dis']
rt_ls = data['r/t']

len = len(dis_ls)

success_rate = []

for index in range(len):
    if rt_ls[index] == 'r':
        ax.add_patch(Circle(xy=(dis_ls[index], 0), radius=0.1, alpha=0.5, color="g"))
    else:
        ax.add_patch(Circle(xy=(dis_ls[index], 1), radius=0.1, alpha=0.5, color="r"))
    total_com = 0
    success_com = 0
    for j in range(index, len):
        if dis_ls[j] - dis_ls[index] > 50:
            break
        if rt_ls[j] == 'r':
            success_com += 1
        else:
            total_com += 1
    success_rate.append(success_com / total_com)


ax.set_xlim([600, 1200])
ax.set_ylim([-10, 30])

# plt.savefig(newest[:-4] + 'com.jpg')
# plt.show()


fg = plt.figure(dpi=300, figsize=(20, 10))
ax = plt.axes()

plt.title('Package success rate with distance')
plt.xlabel('Distance (m)')
plt.ylabel('Package success rate (%)')

ax.plot(data['dis'], success_rate)

plt.savefig(newest[:-11] + 'package_loss.jpg')
plt.show()
