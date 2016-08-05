import matplotlib.pyplot as plt
from matplotlib import cm, colors
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Create a sphere
r = 2.5
pi = np.pi
cos = np.cos
sin = np.sin
phi, theta = np.mgrid[0.01:pi:50j, 0.01:pi:50j]
x = r*sin(phi)*cos(theta)
z = r*sin(phi)*sin(theta) + 3.0
y = r*cos(phi)
mu, sigma = 0, 0.2 # mean and standard deviation
tfx, tfy, tfz = 1.2, -0.8, 0.5

num_points = 50*50
HEADER = '# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {}\nDATA ascii\n'.format(num_points, num_points)

source = open('p1.pcd', 'w')
target = open('p2.pcd', 'w')
source.write(HEADER)
target.write(HEADER)
for i in range(0, 50):
  for j in range(0, 50):
    noisex = np.random.normal(mu, sigma, 1)[0]
    noisey = np.random.normal(mu, sigma, 1)[0]
    noisez = np.random.normal(mu, sigma, 1)[0]
    source.write('{} {} {}\n'.format(x[i][j] + noisex, y[i][j] + noisey, z[i][j] + noisez))
    noisex = np.random.normal(mu, sigma, 1)[0]
    noisey = np.random.normal(mu, sigma, 1)[0]
    noisez = np.random.normal(mu, sigma, 1)[0]
    target.write('{} {} {}\n'.format(x[i][j] + noisex + tfx, y[i][j] + noisey + tfy, z[i][j] + noisez + tfz))
