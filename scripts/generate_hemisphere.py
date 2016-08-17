#!/usr/bin/env python
import numpy as np
import sys

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
tfx, tfy, tfz = 1.2, -0.8, 0.

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

num_points = 50*50
HEADER = '# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {}\nDATA ascii\n'.format(num_points, num_points)

source = open('p1.pcd', 'w')
target = open('p2.pcd', 'w')
source.write(HEADER)
target.write(HEADER)

noisex = np.random.normal(mu, sigma, 50*50)
noisey = np.random.normal(mu, sigma, 50*50)
noisez = np.random.normal(mu, sigma, 50*50)

noisex2 = np.random.normal(mu, sigma, 50*50)
noisey2 = np.random.normal(mu, sigma, 50*50)
noisez2 = np.random.normal(mu, sigma, 50*50)
for i in range(0, 50):
  for j in range(0, 50):
    source.write('{} {} {}\n'.format(x[i][j] + noisex[50*i+j], y[i][j] + noisey[50*i+j], z[i][j] + noisez[50*i+j]))
    target.write('{} {} {}\n'.format(x[i][j] + noisex2[50*i+j] + tfx, y[i][j] + noisey2[50*i+j] + tfy, z[i][j] + noisez2[50*i+j] + tfz))
