from math import cos, sin, pi, floor, radians
import os
import csv
import sys
import copy
import numpy as np
import open3d as o3d

restAngle = 100
tiltArmLength = 82.55

if __name__ == "__main__":

    # generate some neat n times 3 matrix using a variant of sync function
    # x = np.linspace(-3, 3, 401)
    # mesh_x, mesh_y = np.meshgrid(x, x)
    # z = np.sinc((np.power(mesh_x, 2) + np.power(mesh_y, 2) * 10))
    # z_norm = (z - z.min()) / (z.max() - z.min())
    # xyz = np.zeros((np.size(mesh_x), 3))
    # xyz[:, 0] = np.reshape(mesh_x, -1)
    # xyz[:, 1] = np.reshape(mesh_y, -1)
    # xyz[:, 2] = np.reshape(z_norm, -1)
    # print('xyz')
    # print(xyz)
    filename =  './raw_scan.csv'

    if len(sys.argv) > 1:
        filename = str(sys.argv[1])

    print("filename: " + filename)
    # with open('./scan.csv') as csv_file:
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        # xyz = np.array([])
        xyz = []
        for (_tiltAngle, _panAngle, _distance) in csv_reader:
            # print("tiltAngle: ", type(_tiltAngle))
            tiltAngle = float(_tiltAngle)
            panAngle = float(_panAngle)
            distance = float(_distance)
            panRad = radians(180 - panAngle) # inverse angle
            deltaTiltRad = radians(tiltAngle - restAngle) # 0 at top (when laser is at 90deg)
            trueTilt = radians(90) + deltaTiltRad * sin(panRad) # Tilt angle adjusted based on panAngle

            # Offset to adjust for lidar sensor movement along tiltArmLength arc
            # deltaZ = tiltArmLength * (1 - cos(deltaTiltRad))
            # deltaY = -tiltArmLength * sin(deltaTiltRad)
            deltaZ = 0
            deltaY = 0


            x = distance * sin(trueTilt) * cos(panRad)
            y = distance * sin(trueTilt) * sin(panRad) + deltaY
            z = distance * cos(trueTilt) + deltaZ

            #scan_data.append({'x': x, 'y': y, 'z': z})
            # xyz.append("%f,%f,%f" % (x, y, z) )
            xyz.append([x,y,z])

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        o3d.io.write_point_cloud("./sync.ply", pcd)

        # Load saved point cloud and visualize it
        pcd_load = o3d.io.read_point_cloud("./sync.ply")


        o3d.visualization.draw_geometries([pcd_load])

    # convert Open3D.o3d.geometry.PointCloud to numpy array
    # xyz_load = np.asarray(pcd_load.points)
    # print('xyz_load')
    # print(xyz_load)

    # save z_norm as an image (change [0,1] range to [0,255] range with uint8 type)
    # img = o3d.geometry.Image((z_norm * 255).astype(np.uint8))
    # o3d.io.write_image("./sync.png", img)
    # o3d.visualization.draw_geometries([img])
