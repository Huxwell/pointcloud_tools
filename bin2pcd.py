import numpy as np
import open3d as o3d

bin_pcd = np.fromfile("path_to_bin_pointsclod_file_format_used_by_kitti", dtype=np.float32)
points = bin_pcd.reshape((-1, 4))[:, 0:3] # Reshape and drop reflection values - optional
o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
o3d.io.write_point_cloud("path_to_file.pcd", o3d_pcd, write_ascii=True)
