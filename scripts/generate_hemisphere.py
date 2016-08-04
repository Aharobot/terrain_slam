import matplotlib.pyplot as plt
from matplotlib import cm, colors
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Create a sphere
r = 1
pi = np.pi
cos = np.cos
sin = np.sin
phi, theta = np.mgrid[0.0:pi:100j, 0.0:pi:100j]
x = r*sin(phi)*cos(theta)
z = r*sin(phi)*sin(theta)
y = r*cos(phi)

x = x.tolist()

for i in range(0, 100):
  for j in range(0, 100):
    print x[i][j], y[i][j], z[i][j]

print '\n'


for i in range(0, 100):
  for j in range(0, 100):
    print x[i][j] + 0.25, y[i][j] + 0.3, z[i][j] + 0.1