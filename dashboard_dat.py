import pandas as pd
import numpy as np
from plotly.subplots import make_subplots
import plotly.graph_objects as go

# Manually specify the correct column names from the header
column_names = [
    'Name','Brake.Hyd.Sys.pMC','Car.Distance','Car.Fr1.rx','Car.Fr1.ry','Car.Fr1.rz','Car.Fr1.tx','Car.Fr1.ty','Car.Fr1.tz',
    'Car.Gen.ax_1','Car.Gen.ay_1','Car.Gen.vx_1','Car.Pitch','Car.Roll','Car.SideSlipAngle','Car.Yaw','Car.YawRate','Car.ax','Car.ay','Car.tx','Car.ty','Car.v',
    'Car.vFL','Car.vFR','Car.vRL','Car.vRR','DM.Brake','DM.Clutch','DM.Gas','DM.GearNo','DM.Long.StepDist','DM.ManNo','DM.Steer.Ang','DM.Steer.Trq',
    'Driver.Brake','Driver.Gas','Driver.Steer.Ang','PT.Engine.rotv','PwrD.Tire.CambDeflFL','PwrD.Tire.CambDeflFR','PwrD.Tire.LatSlipFL','PwrD.Tire.LatSlipFR',
    'PwrD.Tire.LongSlipFL','PwrD.Tire.LongSlipFR','PwrD.Tire.RollResistFL','PwrD.Tire.RollResistFR','PwrD.Tire.ToeSlipFL','PwrD.Tire.ToeSlipFR',
    'PwrD.Tire.TotalFL','PwrD.Tire.TotalFR','PwrL.Aero','PwrL.Brake','PwrL.PT','PwrL.Tire','PwrL.Total','PwrS.Aero','PwrS.Brake','PwrS.PT','PwrS.Tire',
    'PwrS.Total','Steer.WhlAng','Time','Vhcl.Fr1.x','Vhcl.Fr1.y','Vhcl.v'
]

# Read the .dat file (tab-delimited, skip first 3 header lines)
dat_file = 'Endurance_130548.dat'
df = pd.read_csv(dat_file, delimiter='\t', skiprows=3, names=column_names, engine='python')

try:
    # Extract relevant columns
    time = np.arange(len(df)) * 0.2  # If no time column, assume 0.2s step (adjust if needed)
    x = df['Vhcl.Fr1.x'].values
    y = df['Vhcl.Fr1.y'].values
    speed = df['Vhcl.v'].values
    throttle = df['Driver.Gas'].values
    brake = df['Driver.Brake'].values
    steering = df['Driver.Steer.Ang'].values
except KeyError as e:
    print('Column KeyError:', e)
    print('Available columns:', list(df.columns))
    raise

# Compute acceleration (finite difference of speed)
acceleration = np.gradient(speed, time)

# Normalize throttle, brake, and steering
throttle_norm = (throttle - np.nanmin(throttle)) / (np.nanmax(throttle) - np.nanmin(throttle)) if np.nanmax(throttle) != np.nanmin(throttle) else throttle
brake_norm = (brake - np.nanmin(brake)) / (np.nanmax(brake) - np.nanmin(brake)) if np.nanmax(brake) != np.nanmin(brake) else brake
steering_norm = (steering - np.nanmin(steering)) / (np.nanmax(steering) - np.nanmin(steering)) * 2 - 1 if np.nanmax(steering) != np.nanmin(steering) else steering

# Create figure with subplots (similar to dashboard.py)
fig = make_subplots(
    rows=4, cols=1,
    subplot_titles=("Track and Car Location", "Inputs (Throttle/Brake/Steering)", "Speed", "Acceleration"),
    vertical_spacing=0.08
)

# Track and car path (flip x and y, color by speed)
fig.add_trace(go.Scatter(
    x=y, y=x, mode='lines', name='Car Path (Y vs X)',
    line=dict(color='gray', width=1),
    showlegend=False
), row=1, col=1)
fig.add_trace(go.Scatter(
    x=y, y=x, mode='markers', name='Speed Gradient',
    marker=dict(
        color=speed,
        colorscale='turbo',
        size=4,
        colorbar=dict(title='Speed (m/s)'),
        showscale=True
    ),
    showlegend=False
), row=1, col=1)

# Inputs (normalized)
fig.add_trace(go.Scatter(
    x=time, y=throttle_norm, mode='lines', name='Throttle (norm)', line=dict(color='lime')
), row=2, col=1)
fig.add_trace(go.Scatter(
    x=time, y=brake_norm, mode='lines', name='Brake (norm)', line=dict(color='red')
), row=2, col=1)
fig.add_trace(go.Scatter(
    x=time, y=steering_norm, mode='lines', name='Steering (norm)', line=dict(color='black')
), row=2, col=1)

# Speed
fig.add_trace(go.Scatter(
    x=time, y=speed, mode='lines', name='Speed', line=dict(color='darkblue')
), row=3, col=1)

# Acceleration
fig.add_trace(go.Scatter(
    x=time, y=acceleration, mode='lines', name='Acceleration', line=dict(color='magenta')
), row=4, col=1)

# Update layout
fig.update_layout(
    height=1200,
    width=1000,
    title_text="DriveSIM Endurance Telemetry from .dat File",
    showlegend=True,
    plot_bgcolor='white',
    paper_bgcolor='white',
    font=dict(color='black'),
    title_font=dict(color='black'),
)

fig.update_xaxes(title_text="Time (s)", row=2, col=1, gridcolor='lightgray', zerolinecolor='lightgray', color='black')
fig.update_xaxes(title_text="Time (s)", row=3, col=1, gridcolor='lightgray', zerolinecolor='lightgray', color='black')
fig.update_xaxes(title_text="Time (s)", row=4, col=1, gridcolor='lightgray', zerolinecolor='lightgray', color='black')
fig.update_yaxes(title_text="Speed (m/s)", row=3, col=1, gridcolor='lightgray', zerolinecolor='lightgray', color='black')
fig.update_yaxes(title_text="Acceleration (m/s²)", row=4, col=1, gridcolor='lightgray', zerolinecolor='lightgray', color='black')
fig.update_yaxes(title_text="Normalized Throttle / Brake [0-1], Steering [-1,1]", row=2, col=1, gridcolor='lightgray', zerolinecolor='lightgray', color='black')
fig.update_yaxes(scaleanchor="x", scaleratio=1, row=1, col=1, gridcolor='lightgray', zerolinecolor='lightgray', color='black')

fig.show() 