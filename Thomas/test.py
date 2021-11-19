import open3d as o3d
import numpy as np

print("Let\'s draw a cubic using o3d.geometry.LineSet")
points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0], [0, 0, 1], [1, 0, 1],
            [0, 1, 1], [1, 1, 1]])
lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7],
            [0, 4], [1, 5], [2, 6], [3, 7]]
colors = [[1, 0, 0] for i in range(len(lines))]
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(lines)
line_set.colors = o3d.utility.Vector3dVector(colors)

print("TYPE : ", type(line_set))

vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window()

vis.add_geometry(line_set)

t = np.array([1., 1., 1.]) / 10000.
while True :
    points = points + t
    line_set.points = o3d.utility.Vector3dVector(points)
    vis.update_geometry(line_set)
    if not vis.poll_events():
        break
    vis.update_renderer()