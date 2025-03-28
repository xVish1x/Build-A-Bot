import numpy as np
from stl import mesh
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull

import numpy as np

def get_sample_data():
    """
    Returns a sample dataset for an upright cylinder.
    - Base center at (0, 0, 0)
    - Top center at (0, 0, height)
    - Uses 20 points for each circular face.
    """
    radius = 50  # Cylinder radius
    height = 100  # Cylinder height
    num_points = 20  # Points per circle

    # Generate points for bottom and top circles
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)  # 20 points in a circle
    bottom_circle = np.array([[radius * np.cos(a), radius * np.sin(a), 0] for a in angles])
    top_circle = np.array([[radius * np.cos(a), radius * np.sin(a), height] for a in angles])

    # Combine all points
    points = np.vstack([bottom_circle, top_circle])

    return points


def generate_faces(points):
    """
    Automatically generates faces using Convex Hull for any set of 3D points.
    """
    hull = ConvexHull(points)  # Compute the convex hull
    faces = [points[simplex] for simplex in hull.simplices]  # Extract face triangles
    return np.array(faces)

def create_stl(points, output_file="solid_model.stl"):
    """
    Generates an STL file from any 3D point set.
    """
    faces = generate_faces(points)
    
    # Create the STL mesh
    mesh_data = mesh.Mesh(np.zeros(len(faces), dtype=mesh.Mesh.dtype))
    for i, face in enumerate(faces):
        for j in range(3):  # Triangular faces
            mesh_data.vectors[i][j] = face[j]

    # Save as STL
    mesh_data.save(output_file)
    print(f"STL file saved as {output_file}")

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import numpy as np
from scipy.spatial import ConvexHull

def plot_3d(points):
    """
    Plots the 3D object showing only outer edges while hiding internal mesh lines.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Generate convex hull faces
    hull = ConvexHull(points)
    faces = [points[simplex] for simplex in hull.simplices]

    # Create the filled 3D surface
    ax.add_collection3d(Poly3DCollection(
        faces, alpha=0.7, facecolor='cyan', edgecolor='none'  # Hide inner mesh edges
    ))

    # Extract only the outer edges
    edges = set()
    for simplex in hull.simplices:
        for i in range(len(simplex)):
            edge = tuple(sorted((simplex[i], simplex[(i + 1) % len(simplex)])))
            edges.add(edge)
    
    # Convert edges to 3D line segments
    edge_lines = [[points[i], points[j]] for i, j in edges]
    ax.add_collection3d(Line3DCollection(
        edge_lines, color='black', linewidths=1  # Show only the outer edges
    ))

    # Add labels for top and bottom points
    z_values = points[:, 2]
    top_index = np.argmax(z_values)
    bottom_index = np.argmin(z_values)

    ax.text(*points[top_index], "Top", color='red', fontsize=12, fontweight='bold')
    ax.text(*points[bottom_index], "Bottom", color='blue', fontsize=12, fontweight='bold')

    # Set axis labels and view angle
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    plt.title("3D Solid Model (Outer Edges Only)")
    
    # Auto scale to fit shape
    ax.set_box_aspect([1, 1, 1])
    
    plt.show()


def main():
    print("Generating 3D solid model from input coordinates...")
    points = get_sample_data()  # Replace with real data input
    create_stl(points)
    plot_3d(points)

if __name__ == "__main__":
    main()
