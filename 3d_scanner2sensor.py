import numpy as np
from stl import mesh
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull

def merge_scan_data(scan_data_1, scan_data_2):
    """
    Merges data from two sensors into a single dataset.
    """
    return np.array(scan_data_1 + scan_data_2)

def convert_to_cartesian(scan_data):
    """
    Converts distance-angle-height data to 3D Cartesian coordinates.
    """
    cartesian_points = []
    
    for distance, angle, height in scan_data:
        x = distance * np.cos(angle)
        y = distance * np.sin(angle)
        z = height  # Use height directly
        cartesian_points.append([x, y, z])
    
    return np.array(cartesian_points)

def generate_stl(points, filename="scanned_model.stl"):
    """
    Generates an STL file from scanned 3D points.
    """
    hull = ConvexHull(points)
    faces = [points[simplex] for simplex in hull.simplices]
    
    mesh_data = mesh.Mesh(np.zeros(len(faces), dtype=mesh.Mesh.dtype))
    for i, face in enumerate(faces):
        for j in range(3):
            mesh_data.vectors[i][j] = face[j]
    
    mesh_data.save(filename)
    print(f"STL file saved as {filename}")

def plot_3d(points):
    """
    Plots the 3D scanned object and labels top and bottom faces.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    hull = ConvexHull(points)
    faces = [points[simplex] for simplex in hull.simplices]
    
    ax.add_collection3d(Poly3DCollection(faces, alpha=0.7, facecolor='cyan', edgecolor='black'))
        
    # Identify the top and bottom points
    z_values = points[:, 2]
    top_index = np.argmax(z_values)
    bottom_index = np.argmin(z_values)
    
    top_center = np.mean(points[:, :2], axis=0)
    ax.text(top_center[0], top_center[1], points[top_index, 2] + 5, 'Top Side', color='red', fontsize=12, fontweight='bold', ha='center')
    bottom_center = np.mean(points[:, :2], axis=0)
    ax.text(bottom_center[0], bottom_center[1], points[bottom_index, 2] - 5, 'Bottom Side', color='blue', fontsize=12, fontweight='bold', ha='center')
    
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    plt.title("3D Scanned Model (Dual Sensor)")
    plt.show()

import random

def get_random_scan_data(num_points=30, max_distance=20, max_height=50, angle_noise=0.05, distance_noise=1.0):
    """
    Generates a random dataset with slight errors to simulate real-world scanning differences.
    """
    scan_data_1, scan_data_2 = get_random_scan_data()
    
    
    for _ in range(num_points):
        angle = random.uniform(0, 2 * np.pi)
        distance = random.uniform(5, max_distance) + random.uniform(-distance_noise, distance_noise)
        height = random.uniform(0, max_height)
        scan_data_1.append((distance, angle, height))
        
        # Slight error in second sensor's data
        angle_2 = angle + random.uniform(-angle_noise, angle_noise)
        distance_2 = distance + random.uniform(-distance_noise, distance_noise)
        height_2 = height + random.uniform(-1, 1)  # Small height offset
        scan_data_2.append((distance_2, angle_2, height_2))
    
    return scan_data_1, scan_data_2

def main():
    # Replace this with actual scanned data from two sensors
    scan_data_1 = [(10, 0, 0), (10, np.pi/4, 5), (10, np.pi/2, 10), (10, 3*np.pi/4, 15), (10, np.pi, 20)]
    scan_data_2 = [(10, np.pi/8, 2), (10, 3*np.pi/8, 7), (10, 5*np.pi/8, 12), (10, 7*np.pi/8, 17), (10, 9*np.pi/8, 22)]
    
    merged_scan_data = merge_scan_data(scan_data_1, scan_data_2)
    cartesian_points = convert_to_cartesian(merged_scan_data)
    generate_stl(cartesian_points)
    plot_3d(cartesian_points)

if __name__ == "__main__":
    main()
