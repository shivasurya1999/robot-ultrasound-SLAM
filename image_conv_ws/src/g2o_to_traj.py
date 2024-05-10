#!/usr/bin/env python3
"""
AUTHOR: SHIVA SURYA LOLLA
DATE: 16TH APRIL 2024

INPUT:  fused.g2o file containing the vertices and edges in the format desired after pose fusion 
        synced_fusionbag13_data_cam1.txt containing the timestamps for each vertex ID
        
OUTPUT: fused_traj.txt file containing the refined cam 1 poses 
"""

import re

# def read_g2o_vertices(g2o_filepath):
#     vertices = {}
#     with open(g2o_filepath, 'r') as file:
#         for line in file:
#             if line.startswith("VERTEX_SE3:QUAT"):
#                 parts = line.strip().split()
#                 vertex_id = int(parts[1])
#                 tx, ty, tz = map(float, parts[2:5])
#                 qx, qy, qz, qw = map(float, parts[5:9])
#                 vertices[vertex_id] = (tx, ty, tz, qx, qy, qz, qw)
#     return vertices

def read_g2o_vertices(g2o_filepath):
    vertices = {}
    with open(g2o_filepath, 'r') as file:
        vertex_id = 0  # Assign sequential IDs based on line order
        for line in file:
            if line.startswith("VERTEX_SE3:QUAT"):
                parts = line.strip().split()
                tx, ty, tz = map(float, parts[2:5])
                qx, qy, qz, qw = map(float, parts[5:9])
                vertices[vertex_id] = (tx, ty, tz, qx, qy, qz, qw)
                vertex_id += 1
    return vertices

def read_timestamps(timestamp_filepath):
    timestamps = {}
    with open(timestamp_filepath, 'r') as file:
        vertex_id = 0  # Assume an implicit ID based on line order if no explicit ID is available.
        for line in file:
            if line.strip():
                timestamp = line.strip().split()[0]  # Assumes the first part is always the timestamp
                timestamps[vertex_id] = timestamp
                vertex_id += 1
    return timestamps


def save_trajectory(vertices, timestamps, output_filepath):
    with open(output_filepath, 'w') as file:
        for vertex_id in sorted(vertices.keys()):
            tx, ty, tz, qx, qy, qz, qw = vertices[vertex_id]
            timestamp = timestamps.get(vertex_id, "UNKNOWN")
            # Write the trajectory file with the timestamp and pose
            # Format: timestamp tx ty tz qx qy qz qw
            file.write(f"{timestamp} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")

if __name__ == '__main__':
    g2o_filepath = '/home/ssuryalolla/fusionlab_files/data/pose_fusion/result__new4.g2o'
    timestamp_filepath = '/home/ssuryalolla/fusionlab_files/data/pose_fusion/synced_fusionbag13_data_cam1.txt'
    output_filepath = '/home/ssuryalolla/fusionlab_files/data/pose_fusion/fused_trajnew4_cam1.txt'
    
    vertices = read_g2o_vertices(g2o_filepath)
    timestamps = read_timestamps(timestamp_filepath)
    save_trajectory(vertices, timestamps, output_filepath)

    print(f"Trajectory saved to {output_filepath}")
