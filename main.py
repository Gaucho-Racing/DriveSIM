import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from math import pi
import math
import track_gen
from vehicle import *
import dashboard
import copy

NUM_LAPS = 2
dt = 0.01 # seconds
TRACK_MODEL = "data/track.csv"
START_LINE = [0, 0]

##############################
#       MAIN FUNCTION        #
##############################

dt = 0.01 # seconds

def main():
    track = pd.read_csv(TRACK_MODEL)
    d_track = track_gen.discretize_track(track, 1)
    track_xy = track_gen.generate_cartesian(d_track)
    
    lap_times = []
    vehicle_state_array = []
    total_time = 0 # seconds
    
    # current vehicle state
    vehicle_state = VehicleState(location=[0, 0],
                                 heading=-pi/2,
                                 velocity=[0, 0],
                                 speed=0,
                                 throttle=0,
                                 steering=0)


    vehicle_controller = VehicleController(driver_gain_speed=0.2,
                                           driver_gain_P_direction=5.0,
                                           driver_gain_I_direction=0.1,
                                           driver_gain_D_direction=0,
                                           driver_integral_direction=0,
                                           driver_last_delta_direction=0,
                                           driver_gain_lookahead=0.03,
                                           driver_offset_lookahead=1,
                                           driver_corner_accel=10)
    
    # Prepare other Vehicle Settings
    Prep_battery_kWH_array()

    target_location = [0, 0]
    laps_completed = 0
    last_lap_time = 0  # Track when the last lap was completed
    
    while laps_completed < NUM_LAPS:
        velocity_heading = math.atan2(vehicle_state.velocity[1], vehicle_state.velocity[0]) # velocity heading
        speed = (vehicle_state.velocity[0]**2 + vehicle_state.velocity[1]**2)**0.5 * math.cos(vehicle_state.heading - velocity_heading)
        target_location = vehicle_controller.compute_traj_target(vehicle_state=vehicle_state,track_xy=track_xy, target_location=target_location, dt=dt)
        throttle, steering = vehicle_controller.compute_driver_controls(vehicle_state=vehicle_state, track_xy=track_xy,speed=speed, target_location=target_location, dt=dt)
        
        vehicle_state = copy.deepcopy(vehicle_state)
        vehicle_state.speed = speed
        vehicle_state.throttle = throttle 
        vehicle_state.steering = steering  

        ### very simple thing for testing
        # update car heading
        vehicle_state.heading += steering * dt * pi * speed*0.1
        vehicle_state.heading = vehicle_state.heading%(2*pi)
        
        
        # car coordinates
        car_force = [0, 0] # Newtons
    
        car_force[0] += PedalMapping(throttle, speed)
        car_force[0] -= FDrag(speed)
        
        car_slip_angle = (velocity_heading - vehicle_state.heading)%(pi*2)
        if (car_slip_angle > pi):
            car_slip_angle -= pi*2
            
        car_force[1] += car_slip_angle * -20000 * np.clip((vehicle_state.velocity[0]**2 + vehicle_state.velocity[1]**2)**0.5, -0.1, 0.1)*10
        
        # convert to world coordinates
        car_force_world = [car_force[0] * math.cos(vehicle_state.heading) - car_force[1] * math.sin(vehicle_state.heading),
                           car_force[0] * math.sin(vehicle_state.heading) + car_force[1] * math.cos(vehicle_state.heading)]
        
        # calculate new velocity
        vehicle_state.velocity += np.array(car_force_world) / M * dt

        # calculate new location
        vehicle_state.location += vehicle_state.velocity * dt
        
        # Check if the car has crossed the start/finish line
        if (vehicle_state.location[0] - START_LINE[0])**2 + (vehicle_state.location[1] - START_LINE[1])**2 < 1:
            # Only count a new lap if at least 5 seconds have passed since the last lap
            if total_time > 1 and (total_time - last_lap_time) > 5:
                laps_completed += 1
                last_lap_time = total_time
                print(f"Lap {laps_completed} completed at {round(total_time, 2)} seconds")
        
        print(f"Simulating lap : {round(total_time, 3)}s at {dt}s per iteration")
        
        total_time += dt
        
        vehicle_state_array.append(vehicle_state)

    
    dashboard.display(vehicle_state_array, track_xy, total_time)

    

if __name__ == "__main__":
    main()
