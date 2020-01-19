from adafruit_rplidar import RPLidar
from adafruit_servokit import ServoKit
import time
import os
#import open3d as o3d

#from rplidar import RPLidar
lidar = RPLidar(None, '/dev/ttyUSB0')
from math import cos, sin, pi, floor, radians

kit = ServoKit(channels=16)
#info = lidar.get_info()
#print(info)



#health = lidar.get_health()
#print(health)

#for i, scan in enumerate(lidar.iter_scans()):
#    print('%d: Got %d measurments' % (i, len(scan)))
#    if i > 10:
#        break

# polar angle = tiltAngle
# azmuthal angle = panAngle

scan_data = []
raw_data = []
tiltAngle = 0
tiltStep = 0.5
isUp = False
maxPoints = 10000
maxTilt = 40
tiltArmLength = 82.55 #in mm, = 3.25 inches

try:
    os.remove("./scan.csv")
    os.remove("./raw_scan.csv")
    f = open("./scan.csv", "a+")
    f2 = open("./raw_scan.csv", "a+")

    print(lidar.info)

    print("homing tilt servo.")
    kit.servo[0].angle = 0
    time.sleep(2)

    print("starting scan")




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

            raw_data.append("%f,%f,%f" % (tiltAngle, panAngle, distance))

        scan_string = '\n'.join(scan_data) + '\n'
        f.write(scan_string)

        raw_string = '\n'.join(raw_data) + '\n'
        f2.write(raw_string)

        print("tiltAngle: ", tiltAngle)

        print("scan data len: ", len(scan_data))

        scan_data = []
        raw_data = []
        kit.servo[0].angle = tiltAngle


        if tiltAngle >= maxTilt:
            isUp = False

            #exit scan
            break

        if tiltAngle == 0:
            isUp = True
        if isUp:
            tiltAngle = tiltAngle + tiltStep
        else:
            tiltAngle = tiltAngle - tiltStep

        #if maxPoints < len(scan_data):
        #    break


    f.close()
    f2.close()

   # pcd = o3d.geometry.PointCloud()
   # pcd.points = o3d.utility.Vector3dVector(xyz)
   # o3d.io.write_point_cloud("./sync.ply", pcd)

   # pcd_load = o3d.io.read_point_cloud("./sync.ply")
   # o3d.visualization.draw_geometries([pcd_load])

except KeyboardInterrupt:
    print('Stopping.')


lidar.stop()
lidar.stop_motor()
#time.sleep(1)

lidar.disconnect()


#time.sleep(2)
kit.servo[0].angle = 0
time.sleep(2)

