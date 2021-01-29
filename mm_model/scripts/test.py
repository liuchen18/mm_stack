#!/usr/bin/env python 
from base_control import base_class
import matplotlib.pyplot as plt
'''
dir='../data/base_path.txt'
base=base_class(dir)
base.x_spline.show_figure()
'''

a_x=[]
a_y=[]
d_x=[]
d_y=[]
with open('../data/actual_ee_path.txt','r') as f:
    for line in f.readlines():
        data=list(map(float,line.split()))
        a_x.append((data[0]))
        a_y.append((data[1]))

with open('../data/ee_path.txt','r') as f:
    for line in f.readlines():
        data=list(map(float,line.split()))
        d_x.append((data[1])/1000)
        d_y.append((data[2])/1000)

d_index=[i for i in range(len(d_x))]
a_index=[i*0.001 for i in range(len(a_x))]

plt.figure(1)
plt.plot(a_x,a_y,label='actual path')
plt.plot(d_x,d_y,label='desired path')
plt.legend(loc='upper right')
plt.title('ee path')
plt.show()


base_a_x=[]
base_a_y=[]
base_d_x=[]
base_d_y=[]
with open('../data/actual_base_path.txt','r') as f:
    for line in f.readlines():
        data=list(map(float,line.split()))
        base_a_x.append((data[0]))
        base_a_y.append((data[1]))

with open('../data/base_path.txt','r') as f:
    for line in f.readlines():
        data=list(map(float,line.split()))
        base_d_x.append((data[1])/1000)
        base_d_y.append((data[2])/1000)

d_index=[i for i in range(len(d_x))]
a_index=[i*0.001 for i in range(len(a_x))]

plt.figure(1)
plt.plot(base_a_x,base_a_y,label='actual path')
plt.plot(base_d_x,base_d_y,label='desired path')
plt.legend(loc='upper right')
plt.title('base path')
plt.show()


