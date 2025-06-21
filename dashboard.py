from plotly.subplots import make_subplots
import plotly.graph_objects as go
import numpy as np
from vehicle import *
from main import START_LINE, dt

def display(vehicle_state_array, track_xy, total_time):
    # Separate telemetry by laps
    laps = []
    current_lap = []
    last_pos = START_LINE
    cooldown = 50
    cooldown_counter = 0
    lap_stats = []  # Initialize lap_stats list
    
    for state in vehicle_state_array:
        current_lap.append(state)
        if cooldown_counter > 0:
            cooldown_counter -= 1
            continue
            
        if (state.location[0] - START_LINE[0])**2 + (state.location[1] - START_LINE[1])**2 < 1:
            if len(current_lap) > 100:  # Minimum points for a lap
                # Calculate lap statistics
                lap_time = len(current_lap) * dt
                avg_speed = np.mean([state.speed for state in current_lap])
                max_speed = np.max([state.speed for state in current_lap])
                min_speed = np.min([state.speed for state in current_lap])
                
                lap_stats.append({
                    'Lap': len(laps) + 1,
                    'Time': f"{lap_time:.2f}s",
                    'Avg Speed': f"{avg_speed:.2f} m/s",
                    'Max Speed': f"{max_speed:.2f} m/s",
                    'Min Speed': f"{min_speed:.2f} m/s"
                })
                
                laps.append(current_lap)
                current_lap = [state]
                cooldown_counter = cooldown
    
    # Only add the last lap if it's long enough to be considered a complete lap
    if len(current_lap) > 100:  # Same minimum points threshold
        lap_time = len(current_lap) * dt
        avg_speed = np.mean([state.speed for state in current_lap])
        max_speed = np.max([state.speed for state in current_lap])
        min_speed = np.min([state.speed for state in current_lap])
        
        lap_stats.append({
            'Lap': len(laps) + 1,
            'Time': f"{lap_time:.2f}s",
            'Avg Speed': f"{avg_speed:.2f} m/s",
            'Max Speed': f"{max_speed:.2f} m/s",
            'Min Speed': f"{min_speed:.2f} m/s"
        })
        laps.append(current_lap)

    # Create figure with subplots
    fig = make_subplots(
        rows=4, cols=2,  # Changed to 4 rows to add acceleration plot
        subplot_titles=("Track and Car Location", "Lap Statistics", "Simulated Telemetry", "Acceleration Circle", "Speed", "", "Acceleration", ""),  # Added title for acceleration circle
        specs=[[{"type": "scatter", "colspan": 1}, {"type": "table", "rowspan": 2}],
               [{"type": "scatter"}, {"type": "scatter"}],
               [{"type": "scatter"}, None],
               [{"type": "scatter"}, None]],
        column_widths=[0.7, 0.3]  # Make the table column narrower
    )
    
    # Add track (always visible)
    fig.add_trace(go.Scatter(
        x=track_xy['x'], 
        y=track_xy['y'], 
        mode='lines', 
        name='Track',
        line=dict(color='white'),
        visible=True
    ), row=1, col=1)

    # Create buttons for dropdown
    buttons = []
    traces_per_lap = 8  # Number of traces per lap (path, velocity map, throttle, brake, steering, speed, acceleration, acceleration circle)
    
    # Calculate average speed for each track point
    track_speeds = {}  # Dictionary to store speeds at each track point
    for lap_data in laps:
        for state in lap_data:
            key = (round(state.location[0], 2), round(state.location[1], 2))
            if key not in track_speeds:
                track_speeds[key] = []
            track_speeds[key].append(state.speed)
    
    avg_track_speeds = {k: sum(v)/len(v) for k, v in track_speeds.items()}
    
    # Add "All Laps" option with sequential data
    all_laps_time = 0
    all_laps_x = []
    all_laps_y = []
    all_laps_speed = []
    all_laps_throttle = []
    all_laps_brake = []
    
    for lap_idx, lap_data in enumerate(laps):
        lap_time = len(lap_data) * dt
        time_array = np.linspace(all_laps_time, all_laps_time + lap_time, len(lap_data))
        all_laps_time += lap_time
        
        all_laps_x.extend([state.location[0] for state in lap_data])
        all_laps_y.extend([state.location[1] for state in lap_data])
        all_laps_speed.extend([state.speed for state in lap_data])
        all_laps_throttle.extend([(state.throttle) if state.throttle >= 0 else 0 for state in lap_data])
        all_laps_brake.extend([-(state.throttle) if state.throttle <= 0 else 0 for state in lap_data])
    
    # Add all laps traces
    fig.add_trace(go.Scatter(
        x=all_laps_x,
        y=all_laps_y,
        mode='lines',
        line=dict(color='cyan'),
        name='Car Path All Laps'
    ), row=1, col=1)
    
    # Average speed map for all laps
    avg_speed_x = []
    avg_speed_y = []
    avg_speed_values = []
    for (x, y), speeds in track_speeds.items():
        avg_speed_x.append(x)
        avg_speed_y.append(y)
        avg_speed_values.append(sum(speeds)/len(speeds))
    
    normalized_avg_speed = (np.array(avg_speed_values) - min(avg_speed_values)) / (max(avg_speed_values) - min(avg_speed_values))
    
    fig.add_trace(go.Scatter(
        x=avg_speed_x,
        y=avg_speed_y,
        mode='markers',
        marker=dict(color=normalized_avg_speed, colorscale='turbo', size=2),
        name='Average Velocity Map',
        hovertemplate="Avg Speed: %{text:.2f} m/s<extra></extra>",
        text=avg_speed_values
    ), row=1, col=1)
    
    # Add sequential telemetry for all laps
    time_array = np.linspace(0, all_laps_time, len(all_laps_speed))
    
    # Throttle
    fig.add_trace(go.Scatter(
        x=time_array,
        y=all_laps_throttle,
        mode='lines',
        line=dict(color='lime'),
        name='Throttle',
        hovertemplate="Throttle: %{y:.2f}<extra></extra>"
    ), row=2, col=1)
    
    # Brake
    fig.add_trace(go.Scatter(
        x=time_array,
        y=all_laps_brake,
        mode='lines',
        line=dict(color='red'),
        name='Brake',
        hovertemplate="Brake: %{y:.2f}<extra></extra>"
    ), row=2, col=1)

    # Steering
    fig.add_trace(go.Scatter(
        x=time_array,
        y=[state.steering for state in vehicle_state_array],
        mode='lines',
        line=dict(color='white'),
        name='Steering',
        hovertemplate="Steering: %{y:.2f}<extra></extra>"
    ), row=2, col=1)

    # Speed
    fig.add_trace(go.Scatter(
        x=time_array,
        y=all_laps_speed,
        mode='lines',
        line=dict(color='darkblue'),
        name='Speed',
        hovertemplate="Speed: %{y:.2f} m/s<extra></extra>"
    ), row=3, col=1)

    # Compute acceleration (finite difference of speed)
    all_laps_acceleration = np.gradient(all_laps_speed, time_array)

    # Add acceleration plot (row=4, col=1)
    fig.add_trace(go.Scatter(
        x=time_array,
        y=all_laps_acceleration,
        mode='lines',
        line=dict(color='magenta'),
        name='Acceleration',
        hovertemplate="Acceleration: %{y:.2f} m/s²<extra></extra>"
    ), row=4, col=1)

    # Add acceleration circle for all laps (row=2, col=2)
    # Calculate longitudinal and lateral accelerations from velocity changes
    all_laps_acc_x = []
    all_laps_acc_y = []
    for lap_data in laps:
        lap_velocities = np.array([[state.velocity[0], state.velocity[1]] for state in lap_data])
        lap_headings = np.array([state.heading for state in lap_data])
        
        # Calculate acceleration in world coordinates
        if len(lap_velocities) > 1:
            acc_world = np.gradient(lap_velocities, dt, axis=0)
            
            # Convert to car coordinates (longitudinal/lateral)
            for i, (acc_w, heading) in enumerate(zip(acc_world, lap_headings)):
                # Rotate acceleration to car frame
                acc_long = acc_w[0] * np.cos(heading) + acc_w[1] * np.sin(heading)  # longitudinal
                acc_lat = -acc_w[0] * np.sin(heading) + acc_w[1] * np.cos(heading)  # lateral
                all_laps_acc_x.append(acc_long)
                all_laps_acc_y.append(acc_lat)
        else:
            all_laps_acc_x.extend([0] * len(lap_data))
            all_laps_acc_y.extend([0] * len(lap_data))
    
    fig.add_trace(go.Scatter(
        x=all_laps_acc_x,
        y=all_laps_acc_y,
        mode='markers',
        marker=dict(color='blue', size=2, opacity=0.6),
        name='Acceleration Circle All Laps',
        hovertemplate="Longitudinal Acc: %{x:.2f} m/s²<br>Lateral Acc: %{y:.2f} m/s²<extra></extra>"
    ), row=2, col=2)

    # Add individual lap traces
    for lap_idx, lap_data in enumerate(laps):
        lap_time = len(lap_data) * dt
        car_speed_array = np.array([state.speed for state in lap_data])
        normalized_speed = (car_speed_array - car_speed_array.min()) / (car_speed_array.max() - car_speed_array.min())
        time_array = np.linspace(0, lap_time, len(lap_data))
        
        # Car path
        fig.add_trace(go.Scatter(
            x=[state.location[0] for state in lap_data],
            y=[state.location[1] for state in lap_data],
            mode='lines',
            line=dict(color='cyan'),
            name=f'Car Path Lap {lap_idx + 1}',
            visible=False
        ), row=1, col=1)
        
        # Velocity map
        fig.add_trace(go.Scatter(
            x=[state.location[0] for state in lap_data],
            y=[state.location[1] for state in lap_data],
            mode='markers',
            marker=dict(color=normalized_speed, colorscale='turbo', size=2),
            name=f'Velocity Map Lap {lap_idx + 1}',
            hovertemplate="%{text}<extra></extra>",
            text=[state.display() for state in lap_data],
            visible=False
        ), row=1, col=1)
        
        # Throttle
        fig.add_trace(go.Scatter(
            x=time_array,
            y=[(state.throttle) if state.throttle >= 0 else 0 for state in lap_data],
            mode='lines',
            line=dict(color='lime'),
            name=f'Throttle Lap {lap_idx + 1}',
            hovertemplate="Throttle: %{y:.2f}<extra></extra>",
            visible=False
        ), row=2, col=1)
        
        # Brake
        fig.add_trace(go.Scatter(
            x=time_array,
            y=[-(state.throttle) if state.throttle <= 0 else 0 for state in lap_data],
            mode='lines',
            line=dict(color='red'),
            name=f'Brake Lap {lap_idx + 1}',
            hovertemplate="Brake: %{y:.2f}<extra></extra>",
            visible=False
        ), row=2, col=1)
        
        # Steering for each lap
        fig.add_trace(go.Scatter(
            x=time_array,
            y=[state.steering for state in lap_data],
            mode='lines',
            line=dict(color='black'),
            name=f'Steering Lap {lap_idx + 1}',
            hovertemplate="Steering: %{y:.2f}<extra></extra>",
            visible=False
        ), row=2, col=1)
        
        # Speed
        fig.add_trace(go.Scatter(
            x=time_array,
            y=car_speed_array,
            mode='lines',
            line=dict(color='darkblue'),
            name=f'Speed Lap {lap_idx + 1}',
            hovertemplate="Speed: %{y:.2f} m/s<extra></extra>",
            visible=False
        ), row=3, col=1)

        # Acceleration
        car_accel_array = np.gradient(car_speed_array, time_array)
        fig.add_trace(go.Scatter(
            x=time_array,
            y=car_accel_array,
            mode='lines',
            line=dict(color='magenta'),
            name=f'Acceleration Lap {lap_idx + 1}',
            hovertemplate="Acceleration: %{y:.2f} m/s²<extra></extra>",
            visible=False
        ), row=4, col=1)

        # Acceleration Circle
        lap_velocities = np.array([[state.velocity[0], state.velocity[1]] for state in lap_data])
        lap_headings = np.array([state.heading for state in lap_data])
        lap_acc_x = []
        lap_acc_y = []
        
        # Calculate acceleration in world coordinates
        if len(lap_velocities) > 1:
            acc_world = np.gradient(lap_velocities, dt, axis=0)
            
            # Convert to car coordinates (longitudinal/lateral)
            for i, (acc_w, heading) in enumerate(zip(acc_world, lap_headings)):
                # Rotate acceleration to car frame
                acc_long = acc_w[0] * np.cos(heading) + acc_w[1] * np.sin(heading)  # longitudinal
                acc_lat = -acc_w[0] * np.sin(heading) + acc_w[1] * np.cos(heading)  # lateral
                lap_acc_x.append(acc_long)
                lap_acc_y.append(acc_lat)
        else:
            lap_acc_x = [0] * len(lap_data)
            lap_acc_y = [0] * len(lap_data)
            
        fig.add_trace(go.Scatter(
            x=lap_acc_x,
            y=lap_acc_y,
            mode='markers',
            marker=dict(color='red', size=3, opacity=0.7),
            name=f'Acceleration Circle Lap {lap_idx + 1}',
            hovertemplate="Longitudinal Acc: %{x:.2f} m/s²<br>Lateral Acc: %{y:.2f} m/s²<extra></extra>",
            visible=False
        ), row=2, col=2)

    # Add table with corrected data formatting
    fig.add_trace(
        go.Table(
            header=dict(
                values=['Lap', 'Time', 'Avg Speed', 'Max Speed', 'Min Speed'],  # Fixed column names
                fill_color='rgba(50, 50, 50, 1)',
                align='left',
                font=dict(color='white', size=12)
            ),
            cells=dict(
                values=[
                    [lap['Lap'] for lap in lap_stats],
                    [f"{float(lap['Time'].replace('s', '')):.2f}s" for lap in lap_stats],
                    [f"{float(lap['Avg Speed'].replace('m/s', '')):.2f} m/s" for lap in lap_stats],
                    [f"{float(lap['Max Speed'].replace('m/s', '')):.2f} m/s" for lap in lap_stats],
                    [f"{float(lap['Min Speed'].replace('m/s', '')):.2f} m/s" for lap in lap_stats]
                ],
                fill_color='rgba(30, 30, 30, 1)',
                align='left',
                font=dict(color='white', size=11)
            )
        ),
        row=1, col=2
    )

    # Fix the initial visibility settings
    total_traces = 1 + traces_per_lap + (len(laps) * traces_per_lap) + 1  # Track + All laps traces + Individual lap traces + Table
    # print(f"Total traces: {total_traces}")
    # print(f"Number of laps: {len(laps)}")
    # print(f"Traces per lap: {traces_per_lap}")
    # print(f"Actual number of traces in fig: {len(fig.data)}")
    
    # Print out all trace names to debug
    # for i, trace in enumerate(fig.data):
    #     print(f"Trace {i}: {trace.name}")
    
    initial_visibility = [False] * total_traces
    initial_visibility[0] = True  # Track always visible
    initial_visibility[1:traces_per_lap + 1] = [True] * traces_per_lap  # All laps traces
    initial_visibility[-1] = True  # Keep table visible

    # Update all traces to have the correct initial visibility
    for i in range(len(fig.data)):
        fig.data[i].visible = initial_visibility[i]

    # Update the buttons list
    buttons = []
    
    # All Laps button
    all_laps_visibility = initial_visibility.copy()
    # print("\nAll Laps visibility:", all_laps_visibility)
    buttons.append(dict(
        args=[{"visible": all_laps_visibility}],
        label="All Laps",
        method="update"
    ))

    # Individual lap buttons
    for lap_idx in range(len(laps)):
        lap_visibility = [False] * total_traces
        lap_visibility[0] = True  # Track always visible
        start_idx = traces_per_lap + 1 + (lap_idx * traces_per_lap)
        end_idx = start_idx + traces_per_lap
        lap_visibility[start_idx:end_idx] = [True] * traces_per_lap
        lap_visibility[-1] = True  # Keep table visible
        
        # print(f"\nLap {lap_idx + 1} visibility:")
        # print(f"Start index: {start_idx}")
        # print(f"End index: {end_idx}")
        # print(f"Visibility array: {lap_visibility}")
        
        buttons.append(dict(
            args=[{"visible": lap_visibility}],
            label=f"Lap {lap_idx + 1}",
            method="update"
        ))

    # Update layout
    fig.update_layout(
        height=1300,  # Increased height for extra plot
        width=1200,
        title_text="GR25 DriveSIM Endurance Results",
        showlegend=True,
        plot_bgcolor='white',
        paper_bgcolor='white',
        font=dict(color='black'),
        title_font=dict(color='black'),
        updatemenus=[
            dict(
                type="dropdown",
                direction="down",
                x=0.9,
                y=1.15,
                showactive=True,
                buttons=buttons
            ),
            dict(
                type="buttons",
                direction="left",
                x=0.02,
                y=1.15,
                showactive=True,
                buttons=[
                    dict(
                        args=[{
                            "plot_bgcolor": "white",
                            "paper_bgcolor": "white",
                            "font.color": "black",
                            "title.font.color": "black",
                            "xaxis.gridcolor": "lightgray",
                            "xaxis.zerolinecolor": "lightgray",
                            "xaxis.color": "black",
                            "yaxis.gridcolor": "lightgray", 
                            "yaxis.zerolinecolor": "lightgray",
                            "yaxis.color": "black",
                            "xaxis2.gridcolor": "lightgray",
                            "xaxis2.zerolinecolor": "lightgray", 
                            "xaxis2.color": "black",
                            "yaxis2.gridcolor": "lightgray",
                            "yaxis2.zerolinecolor": "lightgray",
                            "yaxis2.color": "black",
                            "xaxis3.gridcolor": "lightgray",
                            "xaxis3.zerolinecolor": "lightgray",
                            "xaxis3.color": "black",
                            "yaxis3.gridcolor": "lightgray",
                            "yaxis3.zerolinecolor": "lightgray",
                            "yaxis3.color": "black",
                            "xaxis4.gridcolor": "lightgray",
                            "xaxis4.zerolinecolor": "lightgray",
                            "xaxis4.color": "black",
                            "yaxis4.gridcolor": "lightgray",
                            "yaxis4.zerolinecolor": "lightgray",
                            "yaxis4.color": "black",
                            "xaxis5.gridcolor": "lightgray",
                            "xaxis5.zerolinecolor": "lightgray",
                            "xaxis5.color": "black",
                            "yaxis5.gridcolor": "lightgray",
                            "yaxis5.zerolinecolor": "lightgray",
                            "yaxis5.color": "black"
                        }],
                        label="☀️ Light",
                        method="relayout"
                    ),
                    dict(
                        args=[{
                            "plot_bgcolor": "black",
                            "paper_bgcolor": "black", 
                            "font.color": "white",
                            "title.font.color": "white",
                            "xaxis.gridcolor": "gray",
                            "xaxis.zerolinecolor": "gray",
                            "xaxis.color": "white",
                            "yaxis.gridcolor": "gray",
                            "yaxis.zerolinecolor": "gray", 
                            "yaxis.color": "white",
                            "xaxis2.gridcolor": "gray",
                            "xaxis2.zerolinecolor": "gray",
                            "xaxis2.color": "white",
                            "yaxis2.gridcolor": "gray",
                            "yaxis2.zerolinecolor": "gray",
                            "yaxis2.color": "white",
                            "xaxis3.gridcolor": "gray",
                            "xaxis3.zerolinecolor": "gray",
                            "xaxis3.color": "white",
                            "yaxis3.gridcolor": "gray",
                            "yaxis3.zerolinecolor": "gray",
                            "yaxis3.color": "white",
                            "xaxis4.gridcolor": "gray",
                            "xaxis4.zerolinecolor": "gray",
                            "xaxis4.color": "white",
                            "yaxis4.gridcolor": "gray",
                            "yaxis4.zerolinecolor": "gray",
                            "yaxis4.color": "white",
                            "xaxis5.gridcolor": "gray",
                            "xaxis5.zerolinecolor": "gray",
                            "xaxis5.color": "white",
                            "yaxis5.gridcolor": "gray",
                            "yaxis5.zerolinecolor": "gray",
                            "yaxis5.color": "white"
                        }],
                        label="🌙 Dark",
                        method="relayout"
                    )
                ]
            )
        ]
    )
    
    fig.update_xaxes(title_text="Time (s)", row=2, col=1, gridcolor='lightgray', zerolinecolor='lightgray', color='black')
    fig.update_xaxes(title_text="Time (s)", row=3, col=1, gridcolor='lightgray', zerolinecolor='lightgray', color='black')
    fig.update_xaxes(title_text="Time (s)", row=4, col=1, gridcolor='lightgray', zerolinecolor='lightgray', color='black')  # New for acceleration
    fig.update_xaxes(title_text="Longitudinal Acceleration (m/s²)", row=2, col=2, gridcolor='lightgray', zerolinecolor='lightgray', color='black')  # Acceleration circle
    fig.update_yaxes(title_text="Speed (m/s)", row=3, col=1, gridcolor='lightgray', zerolinecolor='lightgray', color='black')
    fig.update_yaxes(title_text="Acceleration (m/s²)", row=4, col=1, gridcolor='lightgray', zerolinecolor='lightgray', color='black')  # New for acceleration
    fig.update_yaxes(title_text="Throttle / Brake / Steering", row=2, col=1, 
                     gridcolor='lightgray', zerolinecolor='lightgray', color='black',
                     range=[-1, 2])  # Set y-axis range for inputs
    fig.update_yaxes(title_text="Lateral Acceleration (m/s²)", row=2, col=2, gridcolor='lightgray', zerolinecolor='lightgray', color='black')  # Acceleration circle
    fig.update_yaxes(scaleanchor="x", scaleratio=1, row=1, col=1, gridcolor='lightgray', zerolinecolor='lightgray', color='black')
    fig.update_yaxes(scaleanchor="x5", scaleratio=1, row=2, col=2, gridcolor='lightgray', zerolinecolor='lightgray', color='black')  # Make acceleration circle square

    fig.show()
