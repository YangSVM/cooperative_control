#!/usr/bin/env python3

import numpy as np

import matplotlib.pyplot as plt

pos = np.loadtxt('line_test_meiyuan.txt',delimiter=',')
pos_real = np.loadtxt('line_result.txt', delimiter=',')
plt.figure(1)
plt.plot(pos[:,0], pos[:,1], 'k-')
plt.plot(pos_real[:,0], pos_real[:,1], 'r-')
plt.gca().set_aspect('equal', adjustable='box')
plt.show()


pos = np.loadtxt('shuangyixian2.txt',delimiter=',')
pos_real = np.loadtxt('shuangyixian_result2.txt', delimiter=',')
plt.figure(1)
plt.plot(pos[:,0], pos[:,1], 'k-')
plt.plot(pos_real[:,0], pos_real[:,1], 'r-')
plt.gca().set_aspect('equal', adjustable='box')
plt.show()