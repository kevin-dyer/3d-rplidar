#!/usr/bin/env python3


import copy
import numpy as np
import open3d as o3d
import sys

import csv
#import open3d

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
    filename =  './scan_bkup.csv'

    if len(sys.argv) > 1:
        filename = str(sys.argv[1])

    print("filename: " + filename)
    # with open('./scan.csv') as csv_file:
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        # xyz = np.array([])
        xyz = []
        for row in csv_reader:


            # floatRow = [float(i) for i in row]
            # print(len(floatRow))

            xyz.append(row)
            # xyz.append(floatRow)
            # np.append(xyz, floatRow)
            line_count += 1

            # if line_count == 0:
            #     print(f'Column names are {", ".join(row)}')
            #     line_count += 1
            # else:
            #     print(f'\t{row[0]} works in the {row[1]} department, and was born in {row[2]}.')
            #     line_count += 1
        # print(f'Processed {line_count} lines., xyz: ', xyz)
        # print(xyz)

        # Pass xyz to Open3D.o3d.geometry.PointCloud and visualize

        # print('shape')
        # print(xyz)
        # print(xyz.shape[1])
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
