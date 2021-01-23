#!/usr/bin/env python

import matplotlib.pyplot as  plt
import copy
import random
import math
import time
data=[]
index=[i for i in range(200)]
for i in range(200):
    
    data.append(math.sin(i/45.0*math.pi)+0.005*random.randint(-100,100))

smooth_data=copy.deepcopy(data)
starttime = time.time()
for iteration in range(1000):
    for i in range(2,198):
        correction=2.0 * (smooth_data[i+2] - 4 * smooth_data[i+1] + 6 * smooth_data[i] - 4 * smooth_data[i-1] + smooth_data[i-2])
        #correction=math.cos(smooth_data[i])

        #print(correction)
        current=smooth_data[i]

        smooth_data[i]=current-0.01*correction
time.sleep(1)
endtime = time.time()

print (starttime)
print(endtime)
print(endtime-starttime)
plt.figure(1)
plt.plot(index,data,label='origin')
plt.plot(index,smooth_data,label='smoothed')
plt.legend(loc='upper right')
plt.show()





