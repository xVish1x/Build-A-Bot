import numpy as np
from stl import mesh
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull

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

def get_random_scan_data():
    """
    Generates a random dataset for testing the 3D scanning system.
    """
    np.random.seed(42)  # Ensures repeatability
    num_points = 30  # Number of scanned points
    max_distance = 20  # Maximum distance from the center
    max_height = 50  # Maximum height of the object

    scan_data = []
    for _ in range(num_points):
        distance = np.random.uniform(5, max_distance)  # Random distance
        angle = np.random.uniform(0, 2 * np.pi)  # Random angle in radians
        height = np.random.uniform(0, max_height)  # Random height in Z-axis
        scan_data.append((distance, angle, height))

    return scan_data


def plot_3d(points):
    """
    Plots the 3D scanned object and labels top and bottom faces.
    """
    """
    Plots the 3D scanned object.
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
    ax.text(top_center[0], top_center[1], points[top_index, 2] + 5, 'Top', color='red', fontsize=12, fontweight='bold', ha='center')
    bottom_center = np.mean(points[:, :2], axis=0)
    ax.text(bottom_center[0], bottom_center[1], points[bottom_index, 2] - 5, 'Bottom', color='blue', fontsize=12, fontweight='bold', ha='center')
    
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    plt.title("3D Scanned Model")
    plt.show()

def main():
    # Replace this with actual scanned data
    scan_data = get_random_scan_data()
    cartesian_points = convert_to_cartesian(scan_data)
    generate_stl(cartesian_points)
    plot_3d(cartesian_points)

if __name__ == "__main__":
    main()
