from dataclasses import dataclass
import math
import numpy as np
from math import pi
import pandas as pd

##############################
#     VEHICLE PARAMETERS     #
##############################

# Aero
DRAG_COEFF = 1.09       # drag coefficient
LIFT_COEFF = -2.45      # coefficient of lift
AIR_DENSITY = 1.293     # density of air (kg/m^3)
ACS_FRONT = 0.6294092 # crosssectional area of car (m^2) from front
ACS_TOP = 1.913493  # cross sectional area of car (m^2) from top
DRAG_HEIGHT = 0.3        # height from the ground of resultant drag force, (m)

# Mass and Distribution
CGH = 0.2032   # center of gravity height (m)
M = 280.155    # mass of car (kg)
g = 9.81       # acceleration due to gravity (m/s^2)
W = M*g        # weight of car (N)
WHEELBASE = 1.525      # Wheelbase (meters)
FRONT_M_BIAS = 0.450     # Front Fmass bias
REAR_M_BIAS = 0.550     # Rear mass bias
FRONT_AXLE_TO_CG = FRONT_M_BIAS*WHEELBASE      # Front axle to CG horizontal distance (meters)
REAR_AXLE_TO_CG = WHEELBASE - FRONT_M_BIAS    # Rear axle to CG horizontal distance (meters)
INCLINE = 0      # road incline (radians)
TRACK_WIDTH = 1.194     # track width (m)

# Suspension
ROLL_STIFFNESS_FRONT = 1600 # Nm / deg
ROLL_STIFFNESS_REAR = 1600 #Nm / deg
ROLL_STIFFNESS_REAR_RATIO = ROLL_STIFFNESS_REAR / (ROLL_STIFFNESS_FRONT + ROLL_STIFFNESS_REAR)
ROLL_STIFFNESS_FRONT_RATIO = ROLL_STIFFNESS_FRONT / (ROLL_STIFFNESS_FRONT + ROLL_STIFFNESS_REAR)

# Tires
STATIC_FRICTION = 1.67      # coefficient of static friction for tires
KINETIC_FRICTION = STATIC_FRICTION*0.25   # coefficient of kinetic friction for tires
        #TTC TODO
WHEEL_RADIUS_INCH = 8     # radius of tires (in)
WHEEL_RADIUS = WHEEL_RADIUS_INCH*2.54/100     # radius of tire (m)
TIRE_CIRC = 3.1415 * 2 * WHEEL_RADIUS  # circumference of tire (m)
CRR = 0.014    # rolling resistance coefficient for each wheel

# Battery
Energy_battery_max = 5.927582577 # (kWh)
Temperature_battery_start = 20  # (degC)
Thermal_mass_battery = 11.11    # (kWh/degC)
Resistance_battery = 15e-3 / 3 * 140    # (Ohms)
CELL_CHGR_ARR_SIZE = 339
cell_kWH_tbl = [] # must be pre-computed by calling Prep_battery_kWH_array()

# Motor / Gearing
AMK_Eff_Drive = 0.85    # Efficiency with inverter->motor->gearbox (%)
AMK_Eff_Regen = 0.75    # Efficiency with gearbox->motor->inverter (%)
Emrax_Eff_Drive = 0.85  # Efficiency with inverter->motor->chain (%)
Emrax_Eff_Regen = 0.75  # Efficiency with chain->motor->inverter (%)
Max_Power = 80          # (kW)
Max_Regen_Power = 60    # (kW)
AMK_Gear_Ratio = 12.8   # 12.8:1
Emrax_Gear_Ratio = 3.4  # 3.4:1
AMK_Torque = [21 for rpm in range(16000)] + [(49.8 - 0.0018 * rpm) for rpm in range(16000, 20001)] # (N*m)
Emrax_Torque = [(0.02 * rpm + 220) for rpm in range(500)] + [230 for rpm in range(500, 5000)] + [(1190/3 - rpm/30) for rpm in range(5000, 6501)] # (N*m)
AMK_Max_RPM = 20000
Emrax_Max_RPM = 6500






def PedalMapping(throttle, speed):
    return np.clip(throttle*Max_Power*1e3 / speed, -3000, 3000)


def FDrag(velocity):
    return DRAG_COEFF * 0.5 * AIR_DENSITY * ACS_FRONT* velocity**2

def FLift(velocity):
    return 0.5 * LIFT_COEFF * AIR_DENSITY * ACS_TOP * velocity**2




@dataclass # to drive the vehicle
class VehicleController: # all static
    def __init__(self, 
                 driver_gain_speed, 
                 driver_gain_P_direction,
                 driver_gain_I_direction, 
                 driver_gain_D_direction, 
                 driver_integral_direction, 
                 driver_last_delta_direction, 
                 driver_gain_lookahead, 
                 driver_offset_lookahead, 
                 driver_corner_accel):
        self.driver_gain_speed = driver_gain_speed
        self.driver_gain_P_direction = driver_gain_P_direction
        self.driver_gain_I_direction = driver_gain_I_direction
        self.driver_gain_D_direction = driver_gain_D_direction
        self.driver_integral_direction = driver_integral_direction
        self.driver_last_delta_direction = driver_last_delta_direction
        self.driver_gain_lookahead = driver_gain_lookahead
        self.driver_offset_lookahead = driver_offset_lookahead
        self.driver_corner_accel = driver_corner_accel
        self.min_dist_idx = 0
        
    driver_gain_speed: float
    driver_gain_P_direction: float
    driver_gain_I_direction: float
    driver_gain_D_direction: float
    driver_integral_direction: float
    driver_last_delta_direction: float
    driver_gain_lookahead: float
    driver_offset_lookahead: float
    driver_corner_accel: float
    
    min_dist_idx: int # for tracking idx

    
    def compute_traj_target(self, vehicle_state, track_xy, target_location, dt, track_step_size):
        # load the raw lists of points
        track_x_array = np.array(track_xy['x'])
        track_y_array = np.array(track_xy['y'])

        distances = (track_x_array - vehicle_state.location[0])**2 + (track_y_array - vehicle_state.location[1])**2
        self.min_dist_idx = np.argmin(distances)
                
        target_idx = (self.min_dist_idx + int(np.clip(vehicle_state.speed/2, 3, 10)/track_step_size)) % len(track_x_array)
        target_location = [target_location[0] + (track_x_array[target_idx] - target_location[0])*dt*10, 
                            target_location[1] + (track_y_array[target_idx] - target_location[1])*dt*10]
        
        return target_location
        
        
        
    def compute_driver_controls(self, vehicle_state, track_xy, speed, target_location, dt, track_step_size):
        track_r_list = track_xy['radius']
        # lookahead
        driver_lookahead_distance = int((self.driver_offset_lookahead + speed**2 * self.driver_gain_lookahead)/track_step_size)
        radius_min = 1e12
        for i in range(self.min_dist_idx, self.min_dist_idx + driver_lookahead_distance):
            if track_r_list[i%len(track_r_list)] == 0:
                track_r_list[i%len(track_r_list)] = 1e12
            if radius_min > abs(track_r_list[i%len(track_r_list)]):
                radius_min = abs(track_r_list[i%len(track_r_list)])
        
        delta_location = np.array(target_location) - np.array(vehicle_state.location)
        target_heading = math.atan2(delta_location[1], delta_location[0])
        delta_heading = (target_heading - vehicle_state.heading)%(pi*2)
        target_speed = np.clip((radius_min * self.driver_corner_accel)**0.5, 1, 35)

        throttle = np.clip((target_speed - speed) * self.driver_gain_speed, -1, 1)
        if (delta_heading > pi):
            delta_heading -= pi*2
        self.driver_integral_direction = np.clip(self.driver_integral_direction 
                                                 + delta_heading*self.driver_gain_I_direction*dt, -1, 1)
        steering = np.clip(delta_heading * self.driver_gain_P_direction + self.driver_integral_direction 
                           + (delta_heading-self.driver_last_delta_direction)*self.driver_gain_D_direction/dt, -1, 1)
        
        self.driver_last_delta_direction = delta_heading
        
        return throttle, steering # (negative throttle is brakes)

# Struct to track vehicle state
@dataclass
class VehicleState:
    def __init__(self, location, heading, velocity, speed, throttle, steering):
        self.location = location
        self.heading = heading
        self.velocity = velocity
        self.speed = speed
        self.throttle = throttle
        self.steering = steering
        
    # def __str__(self):
        # pass
        
    def display(self):
        return (f"Speed: {self.speed} [m/s]<br>" 
                f"Throttle: {self.throttle}<br>"
                f"Steering: {self.steering}")
    location: list
    heading: float
    velocity: list
    speed: float
    throttle: float
    steering: float
    acc_x: float # longitudinal acceleration
    acc_y: float # lateral acceleration
    
    

    
    
    
    


# HELPER FUNCTIONS
def Battery_kWH_to_volts(kWH): #Cell voltage from energy (kWh) used
    cell_volts_tbl = pd.read_csv('data/cell_data.csv')['volts'].tolist()
    kWH /= 140 * 3
    left = 0
    right = CELL_CHGR_ARR_SIZE - 1
    mid = (left + right) // 2
    while right - left > 1:
        if kWH > cell_kWH_tbl[mid]:
            left = mid
        elif kWH < cell_kWH_tbl[mid]:
            right = mid
        else:
            return cell_volts_tbl[mid]
        mid = (left + right) // 2
    return cell_volts_tbl[left] + (cell_volts_tbl[right] - cell_volts_tbl[left]) * ((kWH - cell_kWH_tbl[left]) / (cell_kWH_tbl[right] - cell_kWH_tbl[left]))

def Prep_battery_kWH_array(): #Cell voltage from energy (kWh) used
    cell_charge_tbl = pd.read_csv('data/cell_data.csv')['charge'].tolist()
    cell_volts_tbl = pd.read_csv('data/cell_data.csv')['volts'].tolist()
    cell_kWH_count = 0
    for i in range(CELL_CHGR_ARR_SIZE - 1):
        cell_kWH_tbl.append(cell_kWH_count)
        cell_kWH_count += (cell_volts_tbl[i] + cell_volts_tbl[i+1])/2 * (cell_charge_tbl[CELL_CHGR_ARR_SIZE - 2 - i] - cell_charge_tbl[CELL_CHGR_ARR_SIZE - 1 - i]) * 1e-6
    cell_kWH_tbl.append(cell_kWH_count)
    


def Heat_Gen_Accumulator(I_DC): # Heat generated (watts) by accumulator calculated from accumulator current
    return Resistance_battery * I_DC**2

def Heat_Gen_Motors(I_AC): # Heat generated (watts) by TS systen calculated from AC current
    return 0 #TODO



