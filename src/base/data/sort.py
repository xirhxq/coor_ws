from curses import newpad
from email.mime import base
from hashlib import new
import os
import shutil

base_dir = 'src/base/data/'

ls = [l for l in os.listdir(base_dir) if '.txt' in l or '.png' in l or '.mp4' in l or '.gif' in l]

for l in ls:
    date = l[:5]
    new_path = os.path.join(base_dir, date)
    if not os.path.exists(new_path):
        os.mkdir(new_path)
    shutil.move(os.path.join(base_dir, l), os.path.join(new_path, l))
