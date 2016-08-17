#!/usr/bin/env python
import subprocess
import numpy
from sys import exit
import re


generate_hemisphere = ['python', 'generate_hemisphere.py']
generate_step = ['python', 'generate_step.py']
register_clouds = ['./../build/test_adjuster p1.pcd p2.pcd']

noises = [0.2, 0.3, 0.4]#[0.1, 0.5, 1.0, 1.5, 2.0]
xs = [1.0, 2.0, 4.0, 6.0]
ys = [1.0, 2.0, 4.0, 6.0]
zs = [0, 0.5, 1.0]
yaws = [0, 0.1, 0.2]

cost = []
convergence = []
transform = numpy.zeros((720, 4))
f = open('workfile', 'w')
idx = 0
s =  'i noise convergence cost x y z yaw fx fy fz fyaw\n'
f.write(s)
print s,
for noise in noises:
  for x in xs:
    for y in ys:
      for z in zs:
        for yaw in yaws:
          args = []
          args.append(str(noise))
          args.append(str(x))
          args.append(str(y))
          args.append(str(z))
          args.append(str(yaw))
          generate_clouds = generate_step + args
          # print generate_clouds
          # print 'Calling generate'
          cmd = subprocess.call(generate_clouds)
          # print register_clouds
          # print 'Calling register'
          cmd = subprocess.Popen(register_clouds, shell=True, stdout=subprocess.PIPE)
          for line in cmd.stdout:
            if re.search("CONVERGENCE", line):
              if line[11] == '?':
                convergence.append(int(line[13]))
            if re.search("FINAL COST", line):
              words = line.split()
              cost.append(float(words[2]))
            if re.search("After", line):
              words = line.replace('(',' ').replace(')',' ').replace(',',' ').split()
              transform[idx][0] = float(words[1])
              transform[idx][1] = float(words[2])
              transform[idx][2] = float(words[3])
              transform[idx][3] = float(words[7])
          print idx, noise, convergence[idx], cost[idx], x, y, z, yaw, transform[idx][0], transform[idx][1], transform[idx][2], transform[idx][3]
          s = idx, noise, convergence[idx], cost[idx], x, y, z, yaw, transform[idx][0], transform[idx][1], transform[idx][2], transform[idx][3]
          f.write(', '.join(str(c) for c in s))
          f.write('\n')
          idx = idx + 1
f.close()


