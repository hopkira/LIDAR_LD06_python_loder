import serial
import binascii
from CalcLidarData import CalcLidarData
import math
import sys

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

i = 0
while True:
    loopFlag = True
    flag2c = False

    if(i % 40 == 39):
        print("angles:",min(angles),max(angles),"distances:",min(distances),max(distances))
        print(angles)
        print(distances)
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