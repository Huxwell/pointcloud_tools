import numpy as np
import open3d as o3d

pcd = o3d.io.read_point_cloud("path_to_file.pcd")
np.asarray(pcd.points).astype('float32').tofile("path_to_file.bin")
