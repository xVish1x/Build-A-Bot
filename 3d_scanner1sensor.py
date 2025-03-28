import serial
import numpy as np
from stl import mesh
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Serial port configuration
SERIAL_PORT = 'COM3'  # Replace with your Arduino's port
BAUD_RATE = 115200

def read_serial_data(serial_connection):
    """Reads data from the Arduino and returns it as a list of (angle, distance1, distance2)."""
    data_points = []
    try:
        while True:
            line = serial_connection.readline().decode('utf-8').strip()
            if line:
                # Parse the incoming serial data
                try:
                    angle, distance1, distance2 = map(int, line.split(","))
                    data_points.append((angle, distance1, distance2))
                    print(f"Angle: {angle}, Sensor 1: {distance1} mm, Sensor 2: {distance2} mm")
                except ValueError:
                    print(f"Invalid data: {line}")
    except KeyboardInterrupt:
        # Stop reading on Ctrl+C
        print("\nStopping data collection.")
    return data_points

def generate_3d_points(data_points, radius_offset=100):
    """
    Converts angle and distance data into 3D Cartesian points.
    Assumes sensors are offset along the Y-axis.
    """
    points = []
    for angle, d1, d2 in data_points:
        theta = np.radians(angle)
        # Sensor 1 position
        x1 = d1 * np.cos(theta)
        y1 = d1 * np.sin(theta)
        z1 = 0
        points.append((x1, y1, z1))

        # Sensor 2 position (offset by radius_offset in Y-axis)
        x2 = d2 * np.cos(theta)
        y2 = d2 * np.sin(theta) + radius_offset
        z2 = 0
        points.append((x2, y2, z2))
    return np.array(points)

def create_stl(points, output_file="scan_model.stpl"):
    """
    Creates an STL file from the scanned 3D points.
    The model is generated as a set of triangular faces.
    """
    num_points = len(points)
    if num_points < 3:
        raise ValueError("Insufficient points to create an STL file.")

    # Create triangles connecting sequential points (simple triangulation)
    triangles = []
    for i in range(0, num_points - 2, 2):
        # Triangulate between consecutive points
        triangles.append([points[i], points[i + 1], points[i + 2]])

    # Convert triangles into NumPy array
    triangles = np.array(triangles)

    # Create a mesh object
    mesh_data = mesh.Mesh(np.zeros(triangles.shape[0], dtype=mesh.Mesh.dtype))
    for i, triangle in enumerate(triangles):
        for j in range(3):
            mesh_data.vectors[i][j] = triangle[j]

    # Write to STL file
    mesh_data.save(output_file)
    print(f"STL file saved as {output_file}")

def main():
    print("Starting 3D scan...")
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as serial_connection:
        print("Press Ctrl+C to stop data collection.")
        data_points = read_serial_data(serial_connection)

    if data_points:
        print("Data collection complete. Generating 3D points...")
        points = generate_3d_points(data_points)
        print(f"Collected {len(points)} points.")

        # Save the points to an STL file
        output_file = "scanned_model.stpl"  # Change extension to .stpl
        create_stl(points, output_file)

        # Optionally visualize the points
        plot_3d(points)
    else:
        print("No data collected.")

def plot_3d(points):
    """Plots the 3D points using Matplotlib."""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Extract X, Y, Z coordinates
    x_coords = points[:, 0]
    y_coords = points[:, 1]
    z_coords = points[:, 2]

    # Plot the points
    ax.scatter(x_coords, y_coords, z_coords, c='b', marker='o')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title('3D Scan Visualization')
    plt.show()

if __name__ == "__main__":
    main()
