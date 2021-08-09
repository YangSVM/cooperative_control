import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('roadMap_lzjSouth1.txt')
data2 = np.loadtxt('rawMap_lzjSouth.txt')
#data3 = np.loadtxt('rawMap2215.txt')
l1=plt.plot(data[:,1],data[:,2],'r')
l2=plt.plot(data2[:,1],data2[:,2],'g*')
#l3=plt.plot(data3[:,1],data3[:,2],'b')
plt.legend(labels=['1','2'])
plt.show()
