from dataclasses import dataclass
import math
import numpy as np
from math import pi

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
cell_charge_tbl = [4627.54,4625.92,4624.54,4622.87,4621.72,4619.81,4618.07,4617.07,4615.09,4613.42,4611.82,4609.86,4607.96,4605.25,4603.26,4602.59,4600.89,4598.53,4596.67,4595.12,4594.24,4592.32,4590.77,4587.59,4585.82,4584.2,4582.35,4581.52,4579.47,4578.09,4576.29,4575.26,4573.49,4571.63,4569.04,4566.68,4564.69,4562.33,4559.81,4557.66,4554.64,4552.49,4549.27,4548.14,4544.93,4542.8,4540.59,4538.9,4536.47,4534.02,4531.63,4528.95,4526.18,4524.42,4521.84,4517.77,4514.97,4512,4507.87,4505.07,4502.77,4499.3,4495.07,4491.43,4487.02,4484.23,4479.72,4476.8,4474.06,4471.92,4468.68,4463.54,4460.4,4456.37,4451.16,4447.1,4442.15,4438.49,4434.88,4433.17,4428.66,4425.35,4420.02,4415.89,4410.34,4404.8,4400.44,4394.9,4389,4383.45,4377.3,4372.22,4369.26,4366.32,4360.4,4354.34,4349.17,4342.36,4335.8,4332.84,4326.94,4319.25,4314.74,4306.69,4300.11,4293.77,4287.18,4279.64,4269.74,4263.75,4257.63,4252.46,4244.99,4237.17,4229.24,4221.43,4215.22,4206.2,4200.74,4190.76,4182.05,4168.75,4160.68,4157.14,4148.79,4139.86,4126.85,4121.08,4111.93,4102.47,4092.19,4082.73,4073.28,4066.18,4056.72,4049.31,4040.02,4031.06,4021.25,4012.52,4004.7,3995.53,3984.59,3977.14,3967.75,3955.26,3948.99,3944.32,3934.29,3925.43,3914.93,3906.42,3899.19,3890.84,3880.72,3863.73,3853.01,3843.41,3834.77,3825,3813.2,3800.69,3791.7,3781.13,3767.74,3755.12,3742.17,3734.3,3722.77,3710.72,3698.45,3684.64,3670.17,3655.25,3641.88,3633.24,3618.45,3608.95,3593.94,3572.7,3553.74,3538.82,3525.61,3513.72,3500.56,3485.51,3471.01,3460.58,3445.57,3431.73,3417.37,3394.98,3374.76,3356.35,3337.21,3305.94,3276.68,3249.46,3206.92,3173.72,3157.99,3128.49,3107.51,3088.68,3073.79,3059.66,3041.47,3019.46,3001.45,2981.72,2962.89,2942.42,2929.78,2907.75,2886.4,2867.57,2846.13,2823.46,2800.63,2780.89,2758.59,2736.76,2719.79,2697.4,2673.67,2651.65,2631.19,2601.09,2583.81,2564.08,2534.88,2517.89,2494.68,2464.21,2447.3,2425.29,2396.08,2379.55,2357.53,2335.24,2319.79,2284.45,2269.25,2245.64,2219.57,2204.49,2182.68,2152.23,2128.34,2103.06,2081.86,2052.75,2030.65,2009.46,1987.45,1956.71,1920.29,1895.96,1869.63,1849.59,1825.16,1801.06,1769.25,1728.15,1700.51,1676.13,1642.28,1616.11,1586.92,1560,1532.11,1503.54,1474.7,1443.06,1423.32,1402.86,1378.55,1357.84,1336.85,1316.76,1294.86,1271.84,1252.9,1233.73,1215.88,1196.98,1177.18,1162.4,1138.75,1120.65,1108.18,1093.26,1077.7,1053.41,1036.94,1021.58,1001.07,982.52,968.03,944.32,923,900.06,880.41,863.43,834.11,813.08,794.26,761.75,729.25,676.58,616.79,557,487.74,418.47,369.77,321.07,272.38,244.14,215.9,187.67,169.83,151.99,134.15,119.55,104.94,90.34,75.74,66,56.26,46.53,40.04,33.55,27.06,20.57,14.08,10.56,7.04,3.52,0] # mAH
cell_volts_tbl = [2.5,2.505,2.51,2.515,2.52,2.525,2.53,2.535,2.54,2.545,2.55,2.555,2.56,2.565,2.57,2.575,2.58,2.585,2.59,2.595,2.6,2.605,2.61,2.615,2.62,2.625,2.63,2.635,2.64,2.645,2.65,2.655,2.66,2.665,2.67,2.675,2.68,2.685,2.69,2.695,2.7,2.705,2.71,2.715,2.72,2.725,2.73,2.735,2.74,2.745,2.75,2.755,2.76,2.765,2.77,2.775,2.78,2.785,2.79,2.795,2.8,2.805,2.81,2.815,2.82,2.825,2.83,2.835,2.84,2.845,2.85,2.855,2.86,2.865,2.87,2.875,2.88,2.885,2.89,2.895,2.9,2.905,2.91,2.915,2.92,2.925,2.93,2.935,2.94,2.945,2.95,2.955,2.96,2.965,2.97,2.975,2.98,2.985,2.99,2.995,3,3.005,3.01,3.015,3.02,3.025,3.03,3.035,3.04,3.045,3.05,3.055,3.06,3.065,3.07,3.075,3.08,3.085,3.09,3.095,3.1,3.105,3.11,3.115,3.12,3.125,3.13,3.135,3.14,3.145,3.15,3.155,3.16,3.165,3.17,3.175,3.18,3.185,3.19,3.195,3.2,3.205,3.21,3.215,3.22,3.225,3.23,3.235,3.24,3.245,3.25,3.255,3.26,3.265,3.27,3.275,3.28,3.285,3.29,3.295,3.3,3.305,3.31,3.315,3.32,3.325,3.33,3.335,3.34,3.345,3.35,3.355,3.36,3.365,3.37,3.375,3.38,3.385,3.39,3.395,3.4,3.405,3.41,3.415,3.42,3.425,3.43,3.435,3.44,3.445,3.45,3.455,3.46,3.465,3.47,3.475,3.48,3.485,3.49,3.495,3.5,3.505,3.51,3.515,3.52,3.525,3.53,3.535,3.54,3.545,3.55,3.555,3.56,3.565,3.57,3.575,3.58,3.585,3.59,3.595,3.6,3.605,3.61,3.615,3.62,3.625,3.63,3.635,3.64,3.645,3.65,3.655,3.66,3.665,3.67,3.675,3.68,3.685,3.69,3.695,3.7,3.705,3.71,3.715,3.72,3.725,3.73,3.735,3.74,3.745,3.75,3.755,3.76,3.765,3.77,3.775,3.78,3.785,3.79,3.795,3.8,3.805,3.81,3.815,3.82,3.825,3.83,3.835,3.84,3.845,3.85,3.855,3.86,3.865,3.87,3.875,3.88,3.885,3.89,3.895,3.9,3.905,3.91,3.915,3.92,3.925,3.93,3.935,3.94,3.945,3.95,3.955,3.96,3.965,3.97,3.975,3.98,3.985,3.99,3.995,4,4.005,4.01,4.015,4.02,4.025,4.03,4.035,4.04,4.045,4.05,4.055,4.06,4.065,4.07,4.075,4.08,4.085,4.09,4.095,4.1,4.105,4.11,4.115,4.12,4.125,4.13,4.135,4.14,4.145,4.15,4.155,4.16,4.165,4.17,4.175,4.18,4.185,4.19] # Volts
cell_kWH_tbl = []

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
    return np.clip(throttle*80000 / speed, -2000, 2000) # TODO: FIX


def FDrag(velocity):
    return DRAG_COEFF * 0.5 * AIR_DENSITY * ACS_FRONT* velocity**2

def FLift(velocity):
    return 0.5 * LIFT_COEFF * AIR_DENSITY * ACS_TOP * velocity**2




@dataclass # to drive the vehicle
class VehicleController: # all static
    def __init__(self, driver_gain_speed, driver_gain_P_direction, driver_gain_I_direction, driver_gain_D_direction, driver_integral_direction, driver_last_delta_direction, driver_gain_lookahead, driver_offset_lookahead, driver_corner_accel):
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

    
    def compute_traj_target(self, vehicle_state, track_xy, target_location, dt):
        # load the raw lists of points
        track_x_list = track_xy['x']
        track_y_list = track_xy['y']
        min_dist = 1e12
        self.min_dist_idx = 0
        for i in range(len(track_x_list)):
            dist =  ((track_x_list[i] - vehicle_state.location[0])**2 + 
                      (track_y_list[i] - vehicle_state.location[1])**2) ** 0.5
            if min_dist > dist:
                min_dist = dist
                self.min_dist_idx = i
                
        target_idx = (self.min_dist_idx + 3) % len(track_x_list)
        target_location = [target_location[0] + (track_x_list[target_idx] - target_location[0])*dt*10, 
                            target_location[1] + (track_y_list[target_idx] - target_location[1])*dt*10]
        
        return target_location
        
        
        
    def compute_driver_controls(self, vehicle_state, track_xy, speed, target_location, dt):
        track_r_list = track_xy['radius']
        # lookahead
        driver_lookahead_distance = int(self.driver_offset_lookahead + speed**2 * self.driver_gain_lookahead)
        radius_min = 1e12
        for i in range(self.min_dist_idx, self.min_dist_idx + driver_lookahead_distance):
            if track_r_list[i%len(track_r_list)] == 0:
                track_r_list[i%len(track_r_list)] = 1e12
            if radius_min > abs(track_r_list[i%len(track_r_list)]):
                radius_min = abs(track_r_list[i%len(track_r_list)])
        
        delta_location = np.array(target_location) - np.array(vehicle_state.location)
        target_heading = math.atan2(delta_location[1], delta_location[0])
        delta_heading = (target_heading - vehicle_state.heading)%(pi*2)
        target_speed = np.clip((radius_min * self.driver_corner_accel)**0.5, 1, 30)

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
        
    def __str__(self):
        return (f"Location: {self.location} [m]<br>"
                f"Heading: {self.heading} [rad]<br>"
                f"Velocity: {self.velocity}<br>"
                f"Speed: {self.speed} [m/s]<br>" 
                f"Throttle: {self.throttle}<br>"
                f"Steering: {self.steering}")
    location: list
    heading = float
    velocity = list
    speed = float
    throttle = float
    steering = float

    


# HELPER FUNCTIONS
def Battery_kWH_to_volts(kWH): #Cell voltage from energy (kWh) used
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
    cell_kWH_count = 0
    for i in range(CELL_CHGR_ARR_SIZE - 1):
        cell_kWH_tbl.append(cell_kWH_count)
        cell_kWH_count += (cell_volts_tbl[i] + cell_volts_tbl[i+1])/2 * (cell_charge_tbl[CELL_CHGR_ARR_SIZE - 2 - i] - cell_charge_tbl[CELL_CHGR_ARR_SIZE - 1 - i]) * 1e-6
    cell_kWH_tbl.append(cell_kWH_count)

def Heat_Gen_Accumulator(I_DC): # Heat generated (watts) by accumulator calculated from accumulator current
    return Resistance_battery * I_DC**2

def Heat_Gen_Motors(I_AC): # Heat generated (watts) by TS systen calculated from AC current
    return 0 #TODO



