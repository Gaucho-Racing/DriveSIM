import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from math import pi
import math
import track_gen
from vehicle import *
from matplotlib import collections as mcoll
from plotly.subplots import make_subplots
import plotly.graph_objects as go

NUM_LAPS = 22
dt = 0.01 # seconds
TRACK_MODEL = "track.csv"

##############################
#       MAIN FUNCTION        #
##############################

dt = 0.01 # seconds

def main():
    Prep_battery_kWH_array()

    car_location_array = []
    car_heading_array = []
    car_velocity_array = []
    car_speed_array = []
    driver_throttle_array = []
    driver_steering_array = []
    
    car_location = [0, 0]
    car_heading = -pi/2 # radians, CCW is positive
    total_time = 0 # seconds
    entry_speed = 0 # m/s
    car_velocity = np.array([entry_speed, 0], dtype = np.float32) # m/s

    driver_gain_speed = 0.2

    driver_gain_P_direction = 5
    driver_gain_I_direction = 0.1
    driver_gain_D_direction = 0
    driver_integral_direction = 0
    driver_last_delta_direction = 0

    driver_gain_lookahead = 0.03
    driver_offset_lookahead = 1
    driver_corner_accel = 10

    
    track = pd.read_csv(TRACK_MODEL)
    d_track = track_gen.discretize_track(track, 1)
    track_xy = track_gen.generate_cartesian(d_track)
    track_x_list = track_xy['x']
    track_y_list = track_xy['y']
    track_r_list = track_xy['radius']
    target_location = [0, 0]
    while total_time < 10:
        # find target position
        car_velocity_heading = math.atan2(car_velocity[1], car_velocity[0])
        car_speed = (car_velocity[0]**2 + car_velocity[1]**2)**0.5 * math.cos(car_heading - car_velocity_heading)
        distance_min = 1e12
        distance_min_idx = 0
        for i in range(len(track_x_list)):
            distance = ((track_x_list[i] - car_location[0])**2 + (track_y_list[i] - car_location[1])**2) ** 0.5
            # print("distance:", distance)
            if distance_min > distance:
                distance_min = distance
                distance_min_idx = i
        target_idx = (distance_min_idx + 3) % len(track_x_list)
        target_location = [target_location[0] + (track_x_list[target_idx] - target_location[0])*dt*10, target_location[1] + (track_y_list[target_idx] - target_location[1])*dt*10]
        # lookahead
        driver_lookahead_distance = int(driver_offset_lookahead + car_speed**2 * driver_gain_lookahead)
        radius_min = 1e12
        for i in range(distance_min_idx, distance_min_idx + driver_lookahead_distance):
            if track_r_list[i%len(track_r_list)] == 0:
                track_r_list[i%len(track_r_list)] = 1e12
            if radius_min > abs(track_r_list[i%len(track_r_list)]):
                radius_min = abs(track_r_list[i%len(track_r_list)])
        
        delta_location = np.array(target_location) - np.array(car_location)
        target_heading = math.atan2(delta_location[1], delta_location[0])
        delta_heading = (target_heading - car_heading)%(pi*2)
        target_speed = np.clip((radius_min * driver_corner_accel)**0.5, 1, 30)

        throttle = np.clip((target_speed - car_speed) * driver_gain_speed, -1, 1)
        if (delta_heading > pi):
            delta_heading -= pi*2
        driver_integral_direction = np.clip(driver_integral_direction + delta_heading*driver_gain_I_direction*dt, -1, 1)
        steering = np.clip(delta_heading * driver_gain_P_direction + driver_integral_direction + (delta_heading-driver_last_delta_direction)*driver_gain_D_direction/dt, -1, 1)
        driver_last_delta_direction = delta_heading

        # TODO: Car physics
        # calculate requested torque
        
        try:
            # calculate force on car
            acc_x = 0
            acc_y = 0
            # Normal Forces on tires
            Fz_fl = (0.5 * REAR_AXLE_TO_CG / WHEELBASE * M * g) - (ROLL_STIFFNESS_FRONT_RATIO * acc_y * M * CGH / TRACK_WIDTH) - (acc_x * M * CGH / WHEELBASE)
            Fz_fr = (0.5 * REAR_AXLE_TO_CG / WHEELBASE * M * g) + (ROLL_STIFFNESS_FRONT_RATIO * acc_y * M * CGH / TRACK_WIDTH) - (acc_x * M * CGH / WHEELBASE)
            Fz_rr = (0.5 * FRONT_AXLE_TO_CG / WHEELBASE * M * g) - (ROLL_STIFFNESS_REAR_RATIO * acc_y * M * CGH / TRACK_WIDTH) + (acc_x * M * CGH / WHEELBASE)
            Fz_rl = (0.5 * FRONT_AXLE_TO_CG / WHEELBASE * M * g) + (ROLL_STIFFNESS_REAR_RATIO * acc_y * M * CGH / TRACK_WIDTH) + (acc_x * M * CGH / WHEELBASE)
        except:
            pass

        ### very simple thing for testing
        # update car heading
        car_heading += steering * dt * pi * car_speed*0.1
        car_heading = car_heading%(2*pi)
        # car coordinates
        car_force = [0, 0] # Newtons
        car_force[0] += np.clip(throttle*80000 / car_speed, -2000, 2000)
        car_force[0] -= DRAG_COEFF*0.5*AIR_DENSITY*ACS_FRONT*car_speed**2
        car_slip_angle = (car_velocity_heading - car_heading)%(pi*2)
        if (car_slip_angle > pi):
            car_slip_angle -= pi*2
        # if (abs(car_slip_angle) > pi/2):
        #     car_slip_angle = pi - car_slip_angle
        car_force[1] += car_slip_angle * -20000 * np.clip((car_velocity[0]**2 + car_velocity[1]**2)**0.5, -0.1, 0.1)*10
        # convert to world coordinates
        car_force_world = [car_force[0] * math.cos(car_heading) - car_force[1] * math.sin(car_heading),
                           car_force[0] * math.sin(car_heading) + car_force[1] * math.cos(car_heading)]
        # calculate new velocity
        car_velocity += np.array(car_force_world) / M * dt
        #car_velocity *= 0.99

        # calculate new location
        car_location += car_velocity * dt
        
                
        # print(f"current time: {round(total_time, 3)}")
        # print("car_velocity:", car_velocity)
        # print("car_location:", car_location)
        
        print(round(total_time, 2), distance_min_idx, round(throttle, 2), round(steering, 2), round(car_speed, 2), round(target_speed, 2), round(car_heading, 3), round(car_slip_angle, 4))
        # print(total_time)
        
        total_time += dt
        car_location_array.append([car_location[0], car_location[1]])
        car_heading_array.append(car_heading)
        car_velocity_array.append([car_velocity[0], car_velocity[1]])
        car_speed_array.append(car_speed)
        driver_throttle_array.append(throttle*10)
        driver_steering_array.append(steering*10)

    car_location_array = np.array(car_location_array)
    driver_x = car_location_array[:, 0]
    driver_y = car_location_array[:, 1]
    driver_velocity = car_velocity_array
    driver_throttle = np.array(driver_throttle_array)
    driver_steering = np.array(driver_steering_array)
    driver_steering = np.array(driver_steering_array)

    # Create subplots
    fig = make_subplots(rows=3, cols=1, subplot_titles=("Track and Car Location", "Car Speed", "Driver Throttle and Steering"), shared_xaxes=True)

    # Normalize the speed
    car_speed_array = np.array(car_speed_array)
    normalized_speed = (car_speed_array - car_speed_array.min()) / (car_speed_array.max() - car_speed_array.min())

    # Track and Car Location with colormap of speed
    fig.add_trace(go.Scatter(x=track_x_list, y=track_y_list, mode='lines', name='Track'), row=1, col=1)
    fig.add_trace(go.Scatter(x=driver_x, y=driver_y, mode='lines', line=dict(color='blue'), name='Car Path'), row=1, col=1)
    fig.add_trace(go.Scatter(x=driver_x, y=driver_y, mode='markers', marker=dict(color=normalized_speed, colorscale='Viridis', size=5), name='Speed Colormap'), row=1, col=1)
    
    # Car Speed
    fig.add_trace(go.Scatter(x=np.linspace(0, total_time, len(car_speed_array)), y=car_speed_array, mode='lines', name='Speed'), row=2, col=1)

    # Driver Throttle and Steering
    fig.add_trace(go.Scatter(x=np.linspace(0, total_time, len(driver_throttle_array)), y=driver_throttle_array, mode='lines', name='Throttle'), row=3, col=1)
    fig.add_trace(go.Scatter(x=np.linspace(0, total_time, len(driver_steering_array)), y=driver_steering_array, mode='lines', name='Steering'), row=3, col=1)

    # Update layout
    fig.update_layout(height=900, width=1000, title_text="Simulation Results", showlegend=True)
    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig.update_xaxes(title_text="Time (s)", row=3, col=1)
    fig.update_yaxes(title_text="Speed (m/s)", row=2, col=1)
    fig.update_yaxes(title_text="Throttle / Steering", row=3, col=1)
    fig.update_yaxes(scaleanchor="x", scaleratio=1, row=1, col=1)

    fig.show()

if __name__ == "__main__":
    main()
