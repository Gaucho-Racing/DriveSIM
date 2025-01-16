import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
# Load the track data from the CSV file
df = pd.read_csv("track.csv")

# Define the step size
step_size = 1  # You can change this value to any desired step size

# Create a new list to hold the discretized track data
discretized_data = []

# Iterate over each row in the original data
for _, row in df.iterrows():
  sector_length = row['length']
  curve_radius = row['radius']

  # Calculate the number of intervals for this sector based on the step size
  num_segments = int(sector_length // step_size)  # The whole segments
  remaining_length = sector_length % step_size  # The remaining fraction if not exactly a whole number

  # For straight sections (curve_radius == 0)
  if curve_radius == 0:
    for _ in range(num_segments):
      discretized_data.append({'length': step_size, 'radius': 0})
    
    # If there's remaining length (fractional part), append it as the last segment
    if remaining_length > 0:
      discretized_data.append({'length': remaining_length, 'radius': 0})

  else:
    # For curved sections, we break them into segments with the same curve radius
    for _ in range(num_segments):
      discretized_data.append({'length': step_size, 'radius': curve_radius})
    
    # If there's remaining length (fractional part), append it as the last segment
    if remaining_length > 0:
      discretized_data.append({'length': remaining_length, 'radius': curve_radius})

# Convert the discretized data into a DataFrame
discretized_df = pd.DataFrame(discretized_data)

# Save the discretized track data to a new CSV file
discretized_df.to_csv("discretized_track_layout.csv", index=False)

# Load the discretized track data
discretized_df = pd.read_csv("discretized_track_layout.csv")

# Initial coordinates
x, y = 0, 0

# Track direction (start by going along the x-axis)
theta = -math.pi/2  # Angle in radians (0 means along the x-axis)

# List to store the track's x, y coordinates
track_x = [x]
track_y = [y]

# Store the previous radius to determine if direction is changing
previous_radius = 0

# Iterate through the discretized track data
for _, row in discretized_df.iterrows():
    length = row['length']
    radius = row['radius']
    
    if radius == 0:  # Straight section
        # Move along the current direction (x or y)
        x += length * np.cos(theta)
        y += length * np.sin(theta)
    else:  # Curved section
        # The angle change is determined by the arc length (length / radius)
        delta_theta = length / radius

        # Break the curved segment into smaller steps for smoother transitions
        steps = int(length)  # Number of small steps in the curve
        for step in range(steps):
            delta_theta_step = delta_theta / steps  # Smaller angle change per step
            
            # Move along the circular arc with small angle increments
            x_center = x - radius * np.sin(theta)
            y_center = y + radius * np.cos(theta)
            
            # Update angle and position
            theta += delta_theta_step
            x = x_center + radius * np.sin(theta)
            y = y_center - radius * np.cos(theta)
            
            # Store the new coordinates
            track_x.append(x)
            track_y.append(y)
        
        # If there's remaining length (fractional part), process it
        remaining_length = length - steps
        if remaining_length > 0:
            delta_theta_step = delta_theta * (remaining_length / length)  # Adjust for the last portion of the arc
            x_center = x - radius * np.sin(theta)
            y_center = y + radius * np.cos(theta)
            
            # Update the final position after the fractional length
            theta += delta_theta_step
            x = x_center + radius * np.sin(theta)
            y = y_center - radius * np.cos(theta)
            track_x.append(x)
            track_y.append(y)
        
        # Store the current radius for the next iteration to detect direction change
        previous_radius = radius
        
        
# Save the track coordinates to a CSV file
df = pd.DataFrame({'x': track_x, 'y': track_y})
df.to_csv("track_x_y.csv", index=False)


# Plot the track layout in Cartesian coordinates
plt.figure(figsize=(10, 10))
plt.plot(track_x, track_y, label='Track', color='b')

# Set the plot title and labels
plt.title('Track Layout in Cartesian Coordinates')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.axis('equal')  # Equal scaling for both axes
plt.grid(True)

# Show the plot
plt.show()

