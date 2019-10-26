#coding:utf-8
#jiang zheng 2019/10/26
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np

data1 = np.loadtxt("1570604619.400730_OutPose.txt")
data2 = np.loadtxt("1570865167.624038_VSLAM.txt")

first1  = data1[:, 1]
second1 = data1[:, 2]
third1  = data1[:, 3]

#first2  = data2[:, 1]
#second2 = data2[:, 2]
#third2  = data2[:, 3]

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.scatter(first1, second1, third1, label='data1', c='r', marker='.', s=20.0 )   

ax.set_xlabel('X Label') 
ax.set_ylabel('Y Label') 
ax.set_zlabel('Z Label') 

plt.legend(loc="best", markerscale=5., scatterpoints=2, fontsize=12)
  
plt.show()
