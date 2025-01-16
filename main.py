import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from math import pi
import math
import track_gen
from vehicle import *
from matplotlib import collections as mcoll
import dashboard

NUM_LAPS = 1
dt = 0.01 # seconds
TRACK_MODEL = "data/track.csv"
START_LINE = [0, 0]

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
    laps_completed = 0
    while laps_completed < NUM_LAPS:
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
        
        # Check if the car has crossed the start/finish line
        if (car_location[0] - START_LINE[0])**2 + (car_location[1] - START_LINE[1])**2 < 1:
            if total_time > 1:  # Ensure that the car has moved away from the start line before counting a new lap
                laps_completed += 1
                print(f"Lap {laps_completed} completed in {round(total_time, 2)} seconds")
                        
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
    
    
    
    dashboard.display(car_speed_array, driver_x, driver_y, track_x_list, track_y_list, driver_throttle_array, driver_steering_array, total_time)

    

if __name__ == "__main__":
    main()
