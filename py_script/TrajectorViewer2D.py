#coding:utf-8
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np

data1 = np.loadtxt("1566789057.180936.txt")
first = data1[:, 0]
second = data1[:, 6]

plt.plot(first, second, "-")

plt.title("ap2.0_yaw")
plt.xlabel("time")
plt.ylabel("yaw")

plt.grid() 
plt.show() 

