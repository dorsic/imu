#!/usr/bin/python
#	This is the base code needed to get usable angles from a BerryIMU 
#	using a Complementary filter. The readings can be improved by 
#	adding more filters, E.g Kalman, Low pass, median filter, etc..
#   See berryIMU.py for more advanced code.
#
#	For this code to work correctly, BerryIMU must be facing the
#	correct way up. This is when the Skull Logo on the PCB is facing down.
#
#   Both the BerryIMUv1 and BerryIMUv2 are supported
#
#	http://ozzmaker.com/

from __future__ import print_function
import time
import math
import IMU
import datetime
import os
import sys

RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
AA =  0.40      # Complementary filter constant
LA_So = 0.061   # Accelerometer resolution for FS = +- 2g; [mG/LSB]
M_GN = 0.14     # Magnetic sensitivity for FS = +-4 gauss; [mgauss/LSB]
G_So = 0.0175    # Angular rate sensitivity for FS = +- 500 dps; [dps/LSB]

def fileName():
    return '/home/pi/imu/I' + str(time.time()).split('.')[0]+".csv"

def logData(f, data):
    if f:
        f.write(data)
    else:
        print(data, end="")

################# Compass Calibration values ############
# Use calibrateBerryIMU.py to get calibration values 
# Calibrating the compass isnt mandatory, however a calibrated 
# compass will result in a more accurate heading values.

magXmin =  0
magYmin =  0
magZmin =  0
magXmax =  0
magYmax =  0
magZmax =  0

gyroXangle = 0.0
gyroYangle = 0.0
gyroZangle = 0.0
CFangleX = 0.0
CFangleY = 0.0

IMU.detectIMU()     #Detect if BerryIMUv1 or BerryIMUv2 is connected.
IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass

f = open(fileName(), "w") if len(sys.argv)>1 and sys.argv[1] == '-f' else None
a = datetime.datetime.now()

# write header
#print("ts \tloop \tACCx \tACCy \tACCz \tGx \tGy \tGz \tAccXangle \tAccYangle \tGYRx \tGYRy \tGYRz \tgyro_rate_x \tgyro_rate_y \tgyro_date_z \tgyroXangle \tgyroYangle \tgyroZangle \tCFangleX \tCFangleY \theading \tpitch \troll \tcompensatedHeading")

logData(f, "ts \tloop \tACCx \tACCy \tACCz \tGx \tGy \tGz \tGYRx \tGYRy \tGYRz \tgyro_rate_x \tgyro_rate_y \tgyro_rate_z \tgyro_angle_x \tgyro_angle_y \tgyro_angle_z \tCFangleX \tCFangleY \tMAGx \tMAGy \tMAGz \theading \tpitch \troll \tcompensatedHeading\n")
try:
    while True:
#        try:
            #Read the accelerometer,gyroscope and magnetometer values
            ACCx = IMU.readACCx()
            ACCy = IMU.readACCy()
            ACCz = IMU.readACCz()
            GYRx = IMU.readGYRx()
            GYRy = IMU.readGYRy()
            GYRz = IMU.readGYRz()
            MAGx = IMU.readMAGx()
            MAGy = IMU.readMAGy()
            MAGz = IMU.readMAGz()

            # convert acceleration to G
            Gx = ACCx * LA_So / 1000.0
            Gy = ACCy * LA_So / 1000.0
            Gz = ACCz * LA_So / 1000.0

            #Convert Gyro raw to degrees per second
            rate_gyr_x =  GYRx * G_So
            rate_gyr_y =  GYRy * G_So
            rate_gyr_z =  GYRz * G_So

            #Apply compass calibration    
            MAGx -= (magXmin + magXmax) /2 
            MAGy -= (magYmin + magYmax) /2 
            MAGz -= (magZmin + magZmax) /2 

            ##Calculate loop Period(LP). How long between Gyro Reads
            b = datetime.datetime.now() - a
            a = datetime.datetime.now()
            t = time.time()
            LP = b.microseconds/(1000000*1.0)

            #Calculate the angles from the gyro. 
            gyroXangle+=rate_gyr_x*LP
            gyroYangle+=rate_gyr_y*LP
            gyroZangle+=rate_gyr_z*LP

            #Convert Accelerometer values to degrees
            AccXangle =  (math.atan2(ACCy,ACCz)*RAD_TO_DEG)
            AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG

            #convert the values to -180 and +180
            AccYangle = AccYangle - 270.0 if AccYangle > 90.0 else AccYangle + 90.0 

            #Complementary filter used to combine the accelerometer and gyro values.
            CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
            CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle

            #Calculate heading
            heading = 180.0 * math.atan2(MAGy,MAGx)/M_PI
            #Only have our heading between 0 and 360
            heading = heading + 360.0 if heading < 0 else heading

            #Normalize accelerometer raw values.
            accD = math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
            accXnorm = ACCx/accD
            accYnorm = ACCy/accD
            accZnorm = ACCz/accD

            #Calculate pitch and roll
            pitch = math.asin(accXnorm)
            roll = -math.asin(accYnorm/math.cos(pitch))

            #Calculate the new tilt compensated values
            magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
            
            #The compass and accelerometer are orientated differently on the LSM9DS0 and LSM9DS1 and the Z axis on the compass
            #is also reversed. This needs to be taken into consideration when performing the calculations
            magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)   #LSM9DS1

            #Calculate tilt compensated heading
            tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI
            tiltCompensatedHeading = tiltCompensatedHeading + 360.0 if tiltCompensatedHeading < 0 else tiltCompensatedHeading

            logData(f, "{0:.2f} {1:.6f}".format(t, LP))
            logData(f, " {0} {1} {2}".format(ACCx, ACCy, ACCz))
            logData(f, " {0:.4f} {1:.4f} {2:.4f}".format(Gx, Gy, Gz))
            logData(f, " {0} {1} {2}".format(GYRx, GYRy, GYRz))
            logData(f, " {0:.5f} {1:.5f} {2:.5f}".format(rate_gyr_x, rate_gyr_y, rate_gyr_z))
            logData(f, " {0:.4f} {1:.4f} {2:.4f}".format(gyroXangle, gyroYangle, gyroZangle))
            logData(f, " {0:.4f} {1:.4f}".format(CFangleX,CFangleY))
            logData(f, " {0} {1} {2}".format(MAGx, MAGy, MAGz))
            logData(f, " {0:.2f} {1:.2f} {2:.2f} {3:.2f}".format(heading, pitch, roll, tiltCompensatedHeading))

            #print a new line
            logData(f, "\n")
            #time.sleep(0.0)
#        except:
#            pass
except (KeyboardInterrupt, SystemExit):
    f.close()
    pass



