"""
Author: Filip Drapejkowski
Loosly based on https://github.com/dryabokon/fusion
Converts single .db3 file (named the same as containing dir, except for the extension) to set of pcd files.
Intensity is converted to rgb, so that CVAT doesn't crash.
"""

import pandas as pd
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import open3d as o3d
import colorsys
import struct
import numpy as np


def hsl_from_intensity_to_rgb(h, s=1.0, l=0.5):
        h = h / 255
        r, g, b = colorsys.hls_to_rgb(h, l, s)
        r = int(r*255)
        g = int(g*255)
        b = int(b*255)
        return r,g,b

def convert_argb_to_hex_int( a: "int", r:"int", g:"int", b: "int") -> "int":
    hex_string = "".join([hex(el).replace("0x","") for el in [a,r,g,b]])
    return int(hex_string, 16) #should be 32, but 16 returns proper values

def export_lidar_frames(folder_in_lidar, folder_out):
    with Reader(folder_in_lidar) as reader:
        connections = [x for x in reader.connections if x.topic == '/livox/lidar']
        i=0
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            df_frame = import_lidar_rawdata(rawdata, connection)
            pcd = o3d.t.geometry.PointCloud() 
            pcd.point["positions"] = o3d.core.Tensor(df_frame.values[:,:3])
            intensities = df_frame.values[:,3].reshape((-1,1))

            intensities = [[convert_argb_to_hex_int(99,*hsl_from_intensity_to_rgb(int(inte[0])))] for inte in intensities]

            pcd.point["intensities"] = o3d.core.Tensor(intensities, dtype=o3d.core.Dtype.UInt32)
            outpath = folder_out+'/%06d.pcd'%i
            print(outpath)
            o3d.t.io.write_point_cloud(outpath, pcd, write_ascii=True)

            with open(outpath) as r:
                text = r.read().replace("intensities", "rgb").replace("8 8 8 4","8 8 8 8")
            with open(outpath, "w") as w:
                w.write(text)
            i+=1
    return

def float_from_bytes(bytes0,big_endian=False):
    if big_endian:
        fmt = '>f'
    else:
        fmt = '<f'
    flt = struct.unpack(fmt, bytes0)[0]

    return flt
    
def import_lidar_rawdata( rawdata, connection):
    msg = deserialize_cdr(rawdata, connection.msgtype)
    frame = []
    for r in range(int(msg.row_step / msg.point_step)):
        x = float_from_bytes(msg.data[msg.point_step * r + 0:msg.point_step * r + 4])
        y = float_from_bytes(msg.data[msg.point_step * r + 4:msg.point_step * r + 8])
        z = float_from_bytes(msg.data[msg.point_step * r + 8:msg.point_step * r + 12])
        i = float_from_bytes(msg.data[msg.point_step * r +12:msg.point_step * r + 16])
        frame.append([x, y, z, i])

    th_intensity = 28

    frame = np.array(frame)
    frame = frame[~np.all(frame == 0, axis=1)]
    frame = frame[frame[:,3]>th_intensity]
    df_frame = pd.DataFrame(frame)
    # print(df_frame.iloc[:,-1].max()) to get intuition about max values and their range
    return df_frame

if __name__ == '__main__':
    export_lidar_frames('rosbag2_20xx_xx_xx-xx_xx_xx', 'pcds_output')