#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.ticker import (MultipleLocator, FormatStrFormatter, AutoMinorLocator)

from mpl_toolkits.mplot3d import axes3d

resolution=10
tl=2000


data=[[0.0 for i in range(tl/resolution)] for j in range(tl/resolution)]
valid_num=0
plane_num=0
with open('reachability_t.txt','r') as f:
    for line in f.readlines():
        num=list(map(float,line.split()))
        '''
        #yz plane
        if num[0]==0:
            plane_num+=1
            #print(str(int(num[1]/resolution+tl/resolution/2))+' '+str(int(num[2]/resolution+tl/resolution/2))+' '+str(num[3]))
            data[int(num[1]/resolution+tl/resolution/2)][int(num[2]/resolution)]=num[3]/0.16
            if num[3]!=0:
                valid_num+=1
        '''
        #xy plane
        if num[2]==500:
            plane_num+=1
            #print(str(int(num[1]/resolution+tl/resolution/2))+' '+str(int(num[2]/resolution))+' '+str(num[3]))
            data[int(num[0]/resolution+tl/resolution/2)][int(num[1]/resolution+tl/resolution/2)]=num[3]/0.16
            if num[3]!=0:
                valid_num+=1

        '''
        #xz plane
        if num[1]==0:
            plane_num+=1
            #print(str(int(num[1]/resolution+tl/resolution/2))+' '+str(int(num[2]/resolution+tl/resolution/2))+' '+str(num[3]))
            data[int(num[0]/resolution+tl/resolution/2)][int(num[2]/resolution)]=num[3]/0.16
            if num[3]!=0:
                valid_num+=1
        '''

print('plane num is: '+str(plane_num))
print('valid num is: '+str(valid_num))

y=np.array([i for i in range(-1000, 1000,resolution)])
z=np.array([i for i in range(-1000, 1000,resolution)])

print(np.size(y))
print(np.size(z))

y,z=np.meshgrid(y,z)
m=np.array(data)
print(np.size(m))


'''
fig=plt.figure('3D Surface')
ax = plt.gca(projection='3d')
plt.title('normalized manipulability in XZ plane', fontsize=15)
ax.set_xlabel('x/mm', fontsize=14)
ax.set_ylabel('z/mm', fontsize=14)
#ax.set_zlabel('normalized manipulability', fontsize=14)
plt.tick_params(labelsize=10)
surf=ax.plot_surface(z,y,m, rstride=2,cstride=2,cmap='jet')
fig.colorbar(surf, shrink=0.5, aspect=5)
plt.show()
'''

colors=["white","blueviolet","royalblue","aqua","springgreen","greenyellow","yellow","orangered","red"]
clrmap=mcolors.LinearSegmentedColormap.from_list("mycmap",colors)

fig=plt.figure('2D plane')
plt.pcolor(z,y,m,cmap=clrmap)
plt.title('normalized manipulability in Z=1100 mm plane', fontsize=15)

plt.gca().set_aspect('equal', 'box')

plt.gca().set_xlabel("x (mm)",fontsize=12.5)
plt.gca().set_ylabel("y (mm)",fontsize=12.5)

clb=plt.colorbar()

#clb.ax.yaxis.set_major_locator(MultipleLocator(0.025))
clb.ax.yaxis.set_minor_locator(MultipleLocator(0.005))
clb.ax.tick_params(labelsize=10)

#clb.ax.set_title('unit',fontsize=10)

plt.show()