#!/usr/bin/env python
import matplotlib.pyplot as  plt

import numpy as np
from mpl_toolkits.mplot3d import axes3d

resolution=10
tl=2700

# get data from txt file
data=[[0.0 for i in range(tl/resolution)] for j in range(tl/resolution)]
valid_num=0
plane_num=0
with open('reachability_t.txt','r') as f:
    for line in f.readlines():
        num=list(map(float,line.split()))

        
        #yz plane
        if num[0]==0:
            plane_num+=1
            #print(str(int(num[1]/resolution+tl/resolution/2))+' '+str(int(num[2]/resolution+tl/resolution/2))+' '+str(num[3]))
            data[int(num[1]/resolution+tl/resolution/2)][int(num[2]/resolution)]=num[3]/0.16
            if num[3]!=0:
                valid_num+=1
        '''
        #xz plane
        if num[1]==0:
            plane_num+=1
            #print(str(int(num[1]/resolution+tl/resolution/2))+' '+str(int(num[2]/resolution+tl/resolution/2))+' '+str(num[3]))
            data[int(num[0]/resolution+tl/resolution/2)][int(num[2]/resolution+tl/resolution/2)]=num[3]/0.16
            if num[3]!=0:
                valid_num+=1
        '''

#process the data to prepare the fitting
Z=[i*10 for i in range(135)]
inner_bound=[0.0 for i in range(135)]
outer_bound=[0.0 for i in range(135)]
last_average=0.0
thresholt=0.5
for i in range(1,134):
    last_average=0.0
    got_inner=False
    for j in range(tl/resolution/2+1,tl/resolution-1):
        #print(str(i)+' '+str(j))
        average=(data[j][i]+data[j-1][i-1]+data[j-1][i]+data[j-1][i+1]+data[j][i-1]+data[j][i+1]+data[j+1][i-1]+data[j+1][i]+data[j+1][i+1])/9

        if average>thresholt and last_average<thresholt and not got_inner and i<100:
            inner_bound[i]=(j-tl/resolution/2)*10
            got_inner=True
        if average < thresholt and last_average>thresholt:
            outer_bound[i]=(j-tl/resolution/2)*10
            
        last_average=average

#get the fitted curve
Z=np.array(Z)
inner_threshold=75
outer_threshold=112

inner_bound=np.array(inner_bound)
inner=np.polyfit(Z[0:inner_threshold],inner_bound[0:inner_threshold],4)
inner_poly=np.poly1d(inner)

outer_bound=np.array(outer_bound)
outer=np.polyfit(Z[1:outer_threshold],outer_bound[1:outer_threshold],4)
outer_poly=np.poly1d(outer)

plt.figure(1)
plt.title('fitted curve for the bound', fontsize=15)
plt.plot(Z[1:],inner_bound[1:],label='inner bound')
plt.plot(Z[1:],outer_bound[1:],label='outer bound')
plt.plot(Z[0:inner_threshold],inner_poly(Z[0:inner_threshold]),label='fitted inner bound')
plt.plot(Z[1:outer_threshold],outer_poly(Z[1:outer_threshold]),label='fitted outer bound')
plt.legend(loc='upper right')
plt.show()

print(inner_poly)
'''
            4            3            2
-1.004e-08 x + 1.42e-05 x - 0.008378 x + 2.323 x + 115.9
'''

print(outer_poly)
'''
           4             3            2
-3.14e-09 x + 6.064e-06 x - 0.004525 x + 1.365 x + 725
'''

        