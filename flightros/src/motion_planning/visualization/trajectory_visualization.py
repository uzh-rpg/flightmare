import matplotlib.cm as cm
import os
from pathlib import Path
import matplotlib
import numpy as np
import open3d as o3d
import pandas as pd

viz_list = []

print("Loading pointcloud...")
pointcloud = o3d.io.read_point_cloud(
    Path(__file__).absolute().parent.as_posix() + "/data/point_cloud.ply")

obstacles_numpy = np.asarray(pointcloud.points)
viz_list.append(pointcloud)

print("Loading full search...")

vertices = np.loadtxt(
    Path(__file__).absolute().parent.as_posix() + "/data/vertices.txt")
edges = np.loadtxt(Path(__file__).absolute(
).parent.parent.as_posix() + "/data/edges.txt")


colors = [[0, 1, 0] for i in range(edges.shape[0])]
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(vertices),
    lines=o3d.utility.Vector2iVector(edges),
)
line_set.colors = o3d.utility.Vector3dVector(colors)

viz_list.append(line_set)

print("Loading optimal trajectories...")

data = np.loadtxt(Path(__file__).absolute().parent.as_posix() +
                  "/data/solution_path.txt")

points = data[:, 0:3]

tot_points = points.shape[0]

lines = [[0, 1]]
lines = np.array(lines)
for i in range(1, tot_points):
    lines = np.vstack((lines, np.array([i-1, i])))

colors = [[1, 0, 0] for i in range(len(lines))]
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(lines),
)
line_set.colors = o3d.utility.Vector3dVector(colors)

viz_list.append(line_set)

# all trajectories are loaded, visualize
o3d.visualization.draw_geometries(viz_list)
