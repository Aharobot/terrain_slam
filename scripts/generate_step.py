import matplotlib.pyplot as plt
from matplotlib import cm, colors
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import sys

# Create a step
x, y = np.mgrid[-6.0:6.0:100j, -6.0:6.0:100j]

mu, sigma = 0, 0.4 # mean and standard deviation
tfx, tfy, tfz = 8.0, 8.0, 0.5
yaw = 0.3

if len(sys.argv) > 1:
  sigma = float(sys.argv[1])
if len(sys.argv) > 2:
  tfx = float(sys.argv[2])
if len(sys.argv) > 3:
  tfy = float(sys.argv[3])
if len(sys.argv) > 4:
  tfz = float(sys.argv[4])
if len(sys.argv) > 5:
  yaw = float(sys.argv[5])

num_points = 100*100
HEADER = '# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {}\nDATA ascii\n'.format(num_points, num_points)

source = open('p1.pcd', 'w')
target = open('p2.pcd', 'w')
source.write(HEADER)
target.write(HEADER)
for i in range(0, 100):
  for j in range(0, 100):
    if x[i][j] > 0 and y[i][j] < 0:
      z = 5.0 + np.random.normal(mu, sigma, 1)[0]
    else:
      z = 2.0 + np.random.normal(mu, sigma, 1)[0]
    source.write('{} {} {}\n'.format(x[i][j], y[i][j], z))
    target.write('{} {} {}\n'.format(x[i][j]*np.cos(yaw) + y[i][j]*np.sin(yaw) + tfx,
                                     - x[i][j]*np.sin(yaw) + y[i][j]*np.cos(yaw) + tfy,
                                     z + tfz))
