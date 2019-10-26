#coding:utf-8
#jiang zheng 2019/10/26
import matplotlib.pyplot as plt
import numpy as np

data1 = np.loadtxt("1570604619.400730_OutPose.txt")
data2 = np.loadtxt("1570865167.624038_VSLAM.txt")

first1  = data1[:, 1]
second1 = data1[:, 2]

first2  = data2[:, 1]
second2 = data2[:, 2]

plt.scatter(first1, second1, label='data1', marker='o', s=2.0, color='r')
plt.scatter(first2, second2, label='data2', marker='o', s=2.0, color='b')

plt.title("x-y")
plt.xlabel("x")
plt.ylabel("y")

plt.legend(loc="best", markerscale=5., scatterpoints=2, fontsize=12)

plt.grid() 
plt.show() 

