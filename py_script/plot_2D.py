#coding:utf-8
#jiang zheng 2019/10/26
#from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np

data1 = np.loadtxt("1570604619.400730_OutPose.txt")
data2 = np.loadtxt("1570865167.624038_VSLAM.txt")

first1  = data1[:, 1]
second1 = data1[:, 2]

first2  = data2[:, 1]
second2 = data2[:, 2]

plt.plot(first1, second1, '.', label='data1', markersize=4., color='r')
plt.plot(first2, second2, '.', label='data2', markersize=4., color='b')

plt.title("x-y")
plt.xlabel("x")
plt.ylabel("y")

plt.legend(loc="best", markerscale=5., numpoints=2, fontsize=14)

plt.grid() 
plt.show() 

