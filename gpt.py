import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.collections as mcoll

# Function to compute the distance between two points
def distance(x1, y1, x2, y2):
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# Function to compute the maximum velocity based on turn radius
def max_velocity(turn_radius, a_max_centripetal):
    return np.sqrt(abs(turn_radius * a_max_centripetal))

# Function to compute optimal velocities and calculate lap time
def compute_optimal_velocities_and_laptime(track_df, a_max, a_max_dec, a_max_centripetal):
    n_points = len(track_df)
    velocities = np.zeros(n_points)

    # Start at the maximum allowed velocity for the last point
    velocities[-1] = max_velocity(track_df['radius'].iloc[-1], a_max_centripetal)

    # Backward pass: Enforce centripetal and deceleration limits
    for i in range(n_points - 2, -1, -1):
        max_vel = max_velocity(track_df['radius'].iloc[i], a_max_centripetal)
        x1, y1 = track_df.loc[i, ['x', 'y']]
        x2, y2 = track_df.loc[i + 1, ['x', 'y']]
        dist = distance(x1, y1, x2, y2)
        time_to_travel = dist / velocities[i + 1]
        max_velocity_change = a_max_dec * time_to_travel
        velocities[i] = min(max_vel, velocities[i + 1] + max_velocity_change)

    # Forward pass: Enforce acceleration limits
    for i in range(1, n_points):
        max_vel = max_velocity(track_df['radius'].iloc[i], a_max_centripetal)
        x1, y1 = track_df.loc[i - 1, ['x', 'y']]
        x2, y2 = track_df.loc[i, ['x', 'y']]
        dist = distance(x1, y1, x2, y2)
        time_to_travel = dist / velocities[i - 1]
        max_velocity_change = a_max * time_to_travel
        velocities[i] = min(velocities[i], velocities[i - 1] + max_velocity_change)

    # Calculate segment times and total lap time
    segment_times = []
    total_time = 0
    for i in range(1, n_points):
        x1, y1 = track_df.loc[i - 1, ['x', 'y']]
        x2, y2 = track_df.loc[i, ['x', 'y']]
        dist = distance(x1, y1, x2, y2)
        avg_velocity = (velocities[i - 1] + velocities[i]) / 2
        time = dist / avg_velocity
        segment_times.append(time)
        total_time += time

    # Add velocities and segment times to the DataFrame
    track_df['velocity'] = velocities
    track_df['segment_time'] = [0] + segment_times  # First segment time is 0
    print(f"Total Lap Time: {total_time:.2f} seconds")
    return track_df, total_time

# Function to plot the speed map
def plot_speed_map(track_df):
    # Create the plot
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Extract x, y, and velocity columns
    x = track_df['x'].values
    y = track_df['y'].values
    velocity = track_df['velocity'].values
    
    # Normalize velocity for color mapping
    norm = plt.Normalize(velocity.min(), velocity.max())
    cmap = plt.get_cmap('viridis')  # Choose a colormap (e.g., 'viridis', 'plasma', 'coolwarm')
    
    # Create line segments for the track
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    
    # Create a LineCollection to apply gradient coloring
    lc = mcoll.LineCollection(segments, cmap=cmap, norm=norm)
    lc.set_array(velocity)  # Use velocity values for the gradient
    lc.set_linewidth(2)  # Set line width
    line = ax.add_collection(lc)
    
    # Add a colorbar
    cbar = fig.colorbar(line, ax=ax)
    cbar.set_label('Velocity (m/s)')
    
    # Set plot details
    ax.set_title('Speed Map with Gradient')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.axis('equal')  # Keep aspect ratio equal for proper representation
    ax.grid(True)
    
    # Show the plot
    plt.show()

# Example usage
track_data = pd.read_csv('data/xy.csv')
track_df = pd.DataFrame(track_data)

# Define constraints
a_max = 2  # Maximum acceleration (m/s^2)
a_max_dec = 2  # Maximum deceleration (m/s^2)
a_max_centripetal = 10  # Maximum centripetal acceleration (m/s^2)

# Compute optimal velocities and lap time
track_df, total_time = compute_optimal_velocities_and_laptime(track_df, a_max, a_max_dec, a_max_centripetal)

# Print the DataFrame with velocities and lap time
print(track_df)
print(f"Total Lap Time: {total_time:.2f} seconds")

# Plot the speed map
plot_speed_map(track_df)
