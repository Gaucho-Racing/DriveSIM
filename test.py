import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load track data
track_file = "track.csv"  # Replace with your file path
track = pd.read_csv(track_file)

# Initialize parameters
x, y = [0], [0]  # Starting point
theta = 0        # Initial angle in radians

# Helper function for calculating arc points
def plot_arc(x0, y0, radius, start_angle, arc_length, resolution=100):
    # Calculate the angle increment
    end_angle = start_angle + (arc_length / radius)
    angles = np.linspace(start_angle, end_angle, resolution)
    x_arc = x0 + radius * np.sin(angles)
    y_arc = y0 - radius * (1 - np.cos(angles))
    return x_arc, y_arc, end_angle

# Process each sector
for _, sector in track.iterrows():
    sector_length = sector['sector_length']
    radius = sector['radius']
    
    if radius == 0:  # Straight sector
        # Extend in the current direction
        dx = sector_length * np.cos(theta)
        dy = sector_length * np.sin(theta)
        x.append(x[-1] + dx)
        y.append(y[-1] + dy)
    else:  # Curved sector
        # Plot the arc
        x0 = x[-1] - radius * np.sin(theta)
        y0 = y[-1] + radius * np.cos(theta)
        x_arc, y_arc, theta = plot_arc(x0, y0, radius, theta, sector_length)
        x.extend(x_arc)
        y.extend(y_arc)

# Plot the track
plt.figure(figsize=(12, 12))
plt.plot(x, y, marker='o', markersize=2, label='Track Path')
plt.title("Track Layout")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()
