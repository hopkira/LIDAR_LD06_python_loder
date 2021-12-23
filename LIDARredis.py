import serial
import binascii
from CalcLidarData import CalcLidarData
import math
import sys
import pandas as pd
import numpy as np

ser = serial.Serial(port='/dev/lidar360',
                    baudrate=230400,
                    timeout=5.0,
                    bytesize=8,
                    parity='N',
                    stopbits=1)

tmpString = ""
lines = list()
angles = list()
distances = list()

angle_bins = pd.interval_range(start = 0, end = 2*math.pi, periods = 90)

i = 0
while True:
    loopFlag = True
    flag2c = False

    if(i % 40 == 39):

        readings = np.column_stack((angles,distances))
        bin_index = pd.cut(readings[:,0], angle_bins)
        binned_distances = pd.Series(readings[:,1])
        totals = binned_distances.groupby([bin_index].mean())
        totals = totals.values.reshape(90,1)

        print("angles:",min(angles),max(angles),"distances:",min(distances),max(distances))
        print(angles)
        print(distances)
        print(totals)
        i = 0
        sys.exit(0)

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

            # process data
            angles.extend(lidarData.Angle_i)
            distances.extend(lidarData.Distance_i)
            ### finish

            tmpString = ""
            loopFlag = False
        else:
            tmpString += b.hex()+" "
        
        flag2c = False

    i +=1

ser.close()