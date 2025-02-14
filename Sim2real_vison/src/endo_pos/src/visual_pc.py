#!/usr/bin/env python3

import open3d as o3d

def visualize_point_cloud(filename):
    # Load point cloud from file
    point_cloud = o3d.io.read_point_cloud(filename)
    
    # Visualize point cloud
    o3d.visualization.draw_geometries([point_cloud])

if __name__ == '__main__':
    # Specify the point cloud file to visualize
    point_cloud_file = "point_cloud.ply"
    visualize_point_cloud(point_cloud_file)