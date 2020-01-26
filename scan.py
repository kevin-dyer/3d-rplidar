#!/usr/bin/env python

from adafruit_rplidar import RPLidar
from adafruit_servokit import ServoKit
import RPi.GPIO as GPIO
import time
import os
import shutil
#import open3d as o3d

#from rplidar import RPLidar
lidar = RPLidar(None, '/dev/ttyUSB0')
from math import cos, sin, pi, floor, radians

kit = ServoKit(channels=16)
#info = lidar.get_info()
#print(info)


#Buttons
GPIO.setmode(GPIO.BCM)
GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP) #button 2
GPIO.setup(19, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)



# TODO: pass in config for start/stop angels and resolution
def scan():
    #health = lidar.get_health()
    #print(health)

    #for i, scan in enumerate(lidar.iter_scans()):
    #    print('%d: Got %d measurments' % (i, len(scan)))
    #    if i > 10:
    #        break

    # polar angle = tiltAngle
    # azmuthal angle = panAngle



    scan_data = []
    # raw_data = []
    tiltStep = 0.5
    isUp = True
    maxPoints = 10000
    minTilt = 75
    maxTilt = 125
    restAngle = 100
    tiltAngle = minTilt
    tiltArmLength = 82.55 #in mm, = 3.25 inches
    timestamp = int(time.time())
    try:
        # os.remove("./scan.csv")
        # os.remove("./raw_scan.csv")

        filename = 'scan-%d.csv'% timestamp
        f = open('./' + filename, "a+")
        # f2 = open("./raw_scan.csv", "a+")

        #print(lidar.info)

        print("homing tilt servo.")
        kit.servo[0].angle = tiltAngle

        #lidar.start_motor()
        lidar.connect()
        lidar.start_motor()

        time.sleep(2)

        print("starting scan")
        lidar.clear_input()




        for scan in lidar.iter_scans():
            for (_, panAngle, distance) in scan:
                #scan_data[min([359, floor(angle)])] = distance

                
                #x = distance * sin(panAngle) * cos(tiltAngle)
                #y = distance * sin(panAngle) * sin(tiltAngle)
                #z = distance * cos(panAngle)
                panRad = radians(panAngle)
                tiltRad = radians(tiltAngle) #off vertical
                trueTilt = radians(90) + tiltRad * sin(panRad) # Tilt angle adjusted based on panAngle


                # Offset to adjust for lidar sensor movement along tiltArmLength arc
                deltaZ = tiltArmLength * (cos(tiltRad) - 1)
                deltaY = tiltArmLength * sin(tiltRad)


                x = distance * sin(trueTilt) * cos(panRad)
                y = distance * sin(trueTilt) * sin(panRad) + deltaY
                z = distance * cos(trueTilt) + deltaZ
                
                #scan_data.append({'x': x, 'y': y, 'z': z})
                scan_data.append("%f,%f,%f" % (x, y, z) )
                #scan_data.append([x, y, z])
                #scan_data.append(x + ',' + y + ',' + z)

                # raw_data.append("%f,%f,%f" % (tiltAngle, panAngle, distance))

            scan_string = '\n'.join(scan_data) + '\n'
            f.write(scan_string)

            # raw_string = '\n'.join(raw_data) + '\n'
            # f2.write(raw_string)

            # print("tiltAngle: ", tiltAngle, ", tiltRad: ", tiltRad)

            #print("scan data len: ", len(scan_data))

            scan_data = []
            raw_data = []
            kit.servo[0].angle = tiltAngle


            if tiltAngle >= maxTilt:
                isUp = False

                #exit scan
                break

            if tiltAngle <= 0:
                isUp = True
            if isUp:
                tiltAngle = tiltAngle + tiltStep
            else:
                tiltAngle = tiltAngle - tiltStep

            #if maxPoints < len(scan_data):
            #    break


        f.close()
        # f2.close()
        lidar.stop()
        lidar.stop_motor()

        shutil.move('./%s'% filename, '/media/pi/USB30FD/%s'% filename)

       # pcd = o3d.geometry.PointCloud()
       # pcd.points = o3d.utility.Vector3dVector(xyz)
       # o3d.io.write_point_cloud("./sync.ply", pcd)

       # pcd_load = o3d.io.read_point_cloud("./sync.ply")
       # o3d.visualization.draw_geometries([pcd_load])

    except KeyboardInterrupt:
        print('Stopping.')


    #time.sleep(2)
    kit.servo[0].angle = restAngle

    lidar.stop()
    lidar.stop_motor()
    #lidar.disconnect()

    time.sleep(2)



lidar.stop()
lidar.stop_motor()
# BUtton Code
while True:

    #time.sleep(1)
    input_state = GPIO.input(26)
    if input_state == False:
        print('Button 1 Pressed')

        scan()
        # lidar.stop()
        # lidar.stop_motor()

lidar.stop()
lidar.stop_motor()
lidar.disconnect()
