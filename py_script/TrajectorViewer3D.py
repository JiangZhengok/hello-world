#coding:utf-8
# import necessary module
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np

# load data from file
# you can replace this using with open
data1 = np.loadtxt("1566798917.070794.txt")
data2 = np.loadtxt("1566799311.456864.txt")
data3 = np.loadtxt("1566799631.015579.txt")
first = data1[:, 1] - data1[0, 1]
second = data1[:, 2] - data1[0, 2]
third = data1[:, 3] - data1[0, 3]

first2 = data2[:, 1] - data2[0, 1]
second2 = data2[:, 2] - data2[0, 2]
third2 = data2[:, 3] - data2[0, 3]

first3 = data3[:, 1] - data3[0, 1]
second3 = data3[:, 2] - data3[0, 2]
third3 = data3[:, 3] - data3[0, 3]

# print to check data
print first
print second
print third

# new a figure and set it into 3d
fig = plt.figure()
ax = fig.gca(projection='3d')

# set figure information
ax.set_title("AP2.0")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

# draw the figure, the color is r = read
figure1 = ax.plot(first, second, third, c='r')
figure2 = ax.plot(first2, second2, third2, c='b')
figure1 = ax.plot(first3, second3, third3, c='g')

plt.show()
