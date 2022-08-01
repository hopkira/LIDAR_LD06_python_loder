from cmath import sqrt
import serial
from CalcLidarData import CalcLidarData
import math
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt

ser = serial.Serial(port='/dev/lidar360',
                    baudrate=230400,
                    timeout=5.0,
                    bytesize=8,
                    parity='N',
                    stopbits=1)

tmpString = ""

angles = list()
distances = list()
lidar_start = 15
lidar_end = 76
segments = 90

# Create a number of bins covering 360 degrees (2xPI radians)
angle_bins = pd.interval_range(start = 0, end = 2*math.pi, periods = segments)
# Calculate a list of mid-points to calculate cartesian co-ords
mid_points = angle_bins.mid.tolist()
x1 = 7.0 * np.cos(mid_points)
y1 = 5.5 * np.sin(mid_points)
ellipse = np.column_stack((x1,y1))
origin = [0,0]
boundary = np.linalg.norm(ellipse - origin, axis=1)
boundary = boundary[lidar_start:lidar_end]
mid_points = mid_points[lidar_start:lidar_end] # narrow list to angles that the device can see

#last = time.time()
try:
    i = 0
    while True:
        loopFlag = True
        flag2c = False
        # collect 429 readings (complete circle)
        if(i % 40 == 39):
            # create a numpy array from angles/distances pairs
            readings = np.column_stack((angles,distances))
            # work out which reading fits in which angle bin
            bin_index = pd.cut(readings[:,0], angle_bins)
            # put the distances into the right bins
            binned_distances = pd.Series(readings[:,1])
            # choose the closest reading in each bin
            min_dists = binned_distances.groupby([bin_index]).min()
            # turn the 90 min dist readings into an array
            min_dists = min_dists.values.reshape(segments)
            # narrow the min distances to the angles that can be seen
            min_dists = min_dists[lidar_start:lidar_end]
            # Check if it is safe to turn
            if  np.amin(min_dists - boundary) < 0:
                print('Not safe to turn')
            # Visualize
            # convert the polar co-ordinates into x and y arrrays
            # x = min_dists * np.cos(mid_points)
            # y = min_dists * np.sin(mid_points)
            # final = np.column_stack((x,y))
            # plt.plot(x,y)
            # plt.plot(x1,y1)
            # plt.gca().invert_yaxis()
            # plt.show() 
            #now = time.time()
            #print(now-last)
            #last = now
            # Now get next set of readings
            angles.clear()
            distances.clear()
            i = 0


        while loopFlag:
            b = ser.read()
            tmpInt = int.from_bytes(b, 'big')
            
            if (tmpInt == 0x54):
                tmpString +=  b.hex()+" "
                flag2c = True
                continue
            
            elif(tmpInt == 0x2c and flag2c):
                tmpString += b.hex()

                if(not len(tmpString[0:-5].replace(' ','')) == 90 ):
                    tmpString = ""
                    loopFlag = False
                    flag2c = False
                    continue

                lidarData = CalcLidarData(tmpString[0:-5])

                # Add angles and distance data to the lists
                angles.extend(lidarData.Angle_i)
                distances.extend(lidarData.Distance_i)

                tmpString = ""
                loopFlag = False
            else:
                tmpString += b.hex()+" "
            
            flag2c = False

        i +=1

except KeyboardInterrupt:
    ser.close()