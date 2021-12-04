import open3d as o3d
import numpy as np
import math
import itertools


class MeshGenerator(object):
    def __init__(self, mesh_path):
        self.graph = None

        # Code adapted from Open3D tutorial
        print('Loading the mesh:')
        self.mesh = o3d.io.read_triangle_mesh(mesh_path)
        print(self.mesh)
        print('Vertices:')
        print(np.asarray(self.mesh.vertices))
        print('Triangles:')
        print(np.asarray(self.mesh.triangles))
        return

    def visualize_mesh(self):
        print("Try to render a mesh with normals:")
        print("Computing normal and rendering it.")
        self.mesh.compute_vertex_normals()
        o3d.visualization.draw_geometries([self.mesh])
        return

    def visualize_graph(self):
        pcd = self.mesh.sample_points_uniformly(number_of_points=1000)
        o3d.visualization.draw_geometries([pcd])
        return


def main():
    generator = MeshGenerator("../Meshes/duck_pbs.glb")
    generator.visualize_graph()

if __name__ == "__main__":
    main()