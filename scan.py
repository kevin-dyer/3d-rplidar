#!/usr/bin/env python

from adafruit_rplidar import RPLidar
from adafruit_servokit import ServoKit
import RPi.GPIO as GPIO
import time
import os
import shutil
from subprocess import call

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
def scan(deltaTilt=45, halfPan=False, highRes=False):
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
    isUp = True
    restAngle = 100 #Angle where lidar is horizontal
    tiltArmLength = 82.55 #in mm, = 3.25 inches - distance from lidar to servo tilt axis
    minTilt = restAngle - deltaTilt
    maxTilt = restAngle + deltaTilt
    tiltStep = highRes ? 0.1 : 0.5
    
    tiltAngle = minTilt
    timestamp = int(time.time())
    try:
        # os.remove("./scan.csv")
        # os.remove("./raw_scan.csv")

        filename = 'scan-%s-%s-%d.csv'% timestamp, halfPan ? 'half' : 'full', highRes ? 'h' : 'l'
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

                #Only save half rotation if halfPan === True
                if halfPan && panAngle > 180:
                    break
                
                #x = distance * sin(panAngle) * cos(tiltAngle)
                #y = distance * sin(panAngle) * sin(tiltAngle)
                #z = distance * cos(panAngle)
                panRad = radians(panAngle)
                tiltRad = radians(tiltAngle) #off vertical
                # trueTilt = radians(90) + tiltRad * sin(panRad) # Tilt angle adjusted based on panAngle
                trueTilt = radians(restAngle) + tiltRad * sin(panRad) # Tilt angle adjusted based on panAngle - not sure if I shoujld use restAngle or 90


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
    btn1 = GPIO.input(19)
    btn2 = GPIO.input(26)
    btn3 = GPIO.input(6)
    btn4 = GPIO.input(13)

    #Scan front side in high resolution
    if btn1 == False:
        print("Button 1 pressed, scanning")
        scan(45, True, True)

    # Scan front side low resolution
    elif btn2 == False:
        print('Button 2 Pressed')
        scan(45, True, False)

    # Scan entire area, low res
    elif btn3 == False:
        print("Button 3")
        scan(90, False, False)

    # Shutdown
    elif btn4 == False:
        print("Button 4 pressed, shutting down")
        call("sudo shutdown -h now", shell=True)



lidar.stop()
lidar.stop_motor()
lidar.disconnect()
