from plotly.subplots import make_subplots
import plotly.graph_objects as go
import numpy as np

def display(car_speed_array, driver_x, driver_y, track_x_list, track_y_list, driver_throttle_array, driver_steering_array, total_time):
    # Create subplots
    fig = make_subplots(rows=2, cols=1, subplot_titles=("Track and Car Location", "Simulated Telemetry"))

    # Normalize the speed (still for the color map, but not for hover)
    car_speed_array = np.array(car_speed_array)
    normalized_speed = (car_speed_array - car_speed_array.min()) / (car_speed_array.max() - car_speed_array.min())

    # Track and Car Location with colormap of speed
    fig.add_trace(go.Scatter(
        x=track_x_list, y=track_y_list, mode='lines', name='Track',
        line=dict(color='white')
    ), row=1, col=1)
    fig.add_trace(go.Scatter(
        x=driver_x, y=driver_y, mode='lines',
        line=dict(color='cyan'), name='Car Path'
    ), row=1, col=1)
    fig.add_trace(go.Scatter(
        x=driver_x, y=driver_y, mode='markers',
        marker=dict(color=normalized_speed, colorscale='Turbo', size=5),
        name='Speed Colormap',
        hovertemplate="Speed: %{text:.2f} m/s<extra></extra>",
        text=car_speed_array  # Display original speed in hover
    ), row=1, col=1)

    # Car Speed
    fig.add_trace(go.Scatter(
        x=np.linspace(0, total_time, len(car_speed_array)),
        y=car_speed_array, mode='lines',
        line=dict(color='lime'), name='Speed',
        hovertemplate="Speed: %{y:.2f} m/s<extra></extra>"  # Show non-normalized speed in the car speed plot
    ), row=2, col=1)

    # Driver Throttle and Steering
    fig.add_trace(go.Scatter(
        x=np.linspace(0, total_time, len(driver_throttle_array)),
        y=driver_throttle_array, mode='lines',
        line=dict(color='orange'), name='Throttle',
        hovertemplate="Throttle: %{y:.2f}<extra></extra>"
    ), row=2, col=1)
    fig.add_trace(go.Scatter(
        x=np.linspace(0, total_time, len(driver_steering_array)),
        y=driver_steering_array, mode='lines',
        line=dict(color='magenta'), name='Steering',
        hovertemplate="Steering: %{y:.2f}<extra></extra>"
    ), row=2, col=1)

    # Update layout for dark mode
    fig.update_layout(
        height=900, width=1000, title_text="Simulation Results",
        showlegend=True,
        plot_bgcolor='black',
        paper_bgcolor='black',
        font=dict(color='white'),
        title_font=dict(color='white')
    )
    fig.update_xaxes(title_text="Time (s)", row=2, col=1, gridcolor='gray', zerolinecolor='gray', range=[0, total_time])
    fig.update_xaxes(title_text="Time (s)", row=3, col=1, gridcolor='gray', zerolinecolor='gray', range=[0, total_time])
    fig.update_yaxes(title_text="Speed (m/s)", row=2, col=1, gridcolor='gray', zerolinecolor='gray')
    fig.update_yaxes(title_text="Throttle / Brake / Steering", row=3, col=1, gridcolor='gray', zerolinecolor='gray')
    fig.update_yaxes(scaleanchor="x", scaleratio=1, row=1, col=1, gridcolor='gray', zerolinecolor='gray')

    fig.show()
