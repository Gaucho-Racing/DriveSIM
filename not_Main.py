import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from math import pi
import math
from dataclasses import dataclass
from vehicle import *
import track_gen
##############################
#       SIM PARAMETERS       #
##############################

NUM_LAPS = 22


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

def Drivetrain_Model(commandedPower, acc_x, acc_y, velocity):
    Fx_Drag = DRAG_COEFF*1/2*AIR_DENSITY*ACS_FRONT*velocity**2
    Fz_Downforce = -LIFT_COEFF*1/2*AIR_DENSITY*ACS_TOP*velocity**2
    aa = (0.5 * FRONT_M_BIAS * M * g)
    bb=(0.5 * REAR_M_BIAS * M * g)
    cc=(ROLL_STIFFNESS_FRONT * acc_y * M * CGH / TRACK_WIDTH)
    dd = (ROLL_STIFFNESS_REAR * acc_y * M * CGH / TRACK_WIDTH)
    ee=(acc_x * M * CGH / WHEELBASE/2)
    ff=Fz_Downforce*.4*.5
    gg=Fx_Drag * DRAG_HEIGHT/WHEELBASE/2
    Fz_fr = (0.5 * FRONT_M_BIAS * M * g) + (ROLL_STIFFNESS_FRONT * acc_y * M * CGH / TRACK_WIDTH) - (acc_x * M * CGH / WHEELBASE/2) + Fz_Downforce*.4*.5 - Fx_Drag * DRAG_HEIGHT/WHEELBASE/2
    Fz_fl = (0.5 * FRONT_M_BIAS * M * g) - (ROLL_STIFFNESS_FRONT * acc_y * M * CGH / TRACK_WIDTH) - (acc_x * M * CGH / WHEELBASE/2) + Fz_Downforce*.4*.5 - Fx_Drag * DRAG_HEIGHT/WHEELBASE/2
    Fz_rr = (0.5 * REAR_M_BIAS * M * g) + (ROLL_STIFFNESS_REAR * acc_y * M * CGH / TRACK_WIDTH) + (acc_x * M * CGH / WHEELBASE/2) + Fz_Downforce*.6*.5 + Fx_Drag * DRAG_HEIGHT/WHEELBASE/2
    Fz_rl = (0.5 * REAR_M_BIAS * M * g) - (ROLL_STIFFNESS_REAR * acc_y * M * CGH / TRACK_WIDTH) + (acc_x * M * CGH / WHEELBASE/2) + Fz_Downforce*.6*.5 + Fx_Drag * DRAG_HEIGHT/WHEELBASE/2

    Fz_tot = Fz_fr + Fz_fl + Fz_rr + Fz_rl

    Ffr_fr = STATIC_FRICTION * Fz_fr
    Ffr_fl = STATIC_FRICTION * Fz_fl
    Ffr_rr = STATIC_FRICTION * Fz_rr
    Ffr_rl = STATIC_FRICTION * Fz_rl

    if commandedPower < 0:
        return 0, 0, 0, 0,0,0,-5, -(Ffr_fr+Ffr_fl+Ffr_rr+Ffr_rl)/M, Fz_fr, Fz_fl, Fz_rr, Fz_rl

    if (M*acc_y*Fz_fr/Fz_tot)**2 > Ffr_fr**2:
        Fx_fr = 0
        print("fuck")
    else:
        Fx_fr = np.sqrt(Ffr_fr**2 - (M*acc_y*Fz_fr/Fz_tot)**2)
    if (M*acc_y*Fz_fl/Fz_tot)**2 > Ffr_fl**2:
        Fx_fl = 0
        print("fuck")
    else:
        Fx_fl = np.sqrt(Ffr_fl**2 - (M*acc_y*Fz_fl/Fz_tot)**2)
    if (M*acc_y*Fz_rr/Fz_tot)**2 > Ffr_rr**2:
        Fx_rr = 0
        print("fuck")
    else:
        Fx_rr = np.sqrt(Ffr_rr**2 - (M*acc_y*Fz_rr/Fz_tot)**2)
    if (M*acc_y*Fz_rl/Fz_tot)**2 > Ffr_rl**2:
        Fx_rl = 0
        print("fuck")
    else:
        Fx_rl = np.sqrt(Ffr_rl**2 - (M*acc_y*Fz_rl/Fz_tot)**2)

    t_fr = Fx_fr * WHEEL_RADIUS
    t_fl = Fx_fl * WHEEL_RADIUS
    t_rr = Fx_rr * WHEEL_RADIUS
    t_rl = Fx_rl * WHEEL_RADIUS

    AMK_rpm = int(velocity/TIRE_CIRC*AMK_Gear_Ratio*60)
    Emrax_rpm = int(velocity/TIRE_CIRC*Emrax_Gear_Ratio*60)

    t_AMK_fr = t_fr / AMK_Gear_Ratio
    t_AMK_fl = t_fl / AMK_Gear_Ratio
    t_Emrax_r = (t_rr + t_rl) / Emrax_Gear_Ratio


    
    AMK_fl_torque = min(t_AMK_fl, AMK_Torque[AMK_rpm])
    Emrax_r_torque = min(t_Emrax_r, Emrax_Torque[Emrax_rpm], commandedPower*Emrax_Eff_Drive*9549/(Emrax_rpm+.0001))
    Emrax_power = (Emrax_r_torque * Emrax_rpm / 9549)/Emrax_Eff_Drive

    if t_AMK_fr < t_AMK_fl:
        AMK_fr_torque = min(t_AMK_fr, AMK_Torque[AMK_rpm], (commandedPower-Emrax_power)/2*AMK_Eff_Drive*9549/(AMK_rpm+.0001))
        AMK_fr_power = (AMK_fr_torque * AMK_rpm / 9549)/AMK_Eff_Drive
        AMK_fl_torque = min(t_AMK_fl, AMK_Torque[AMK_rpm], (commandedPower-Emrax_power-AMK_fr_power)*AMK_Eff_Drive*9549/(AMK_rpm+.0001))
        AMK_fl_power = (AMK_fl_torque * AMK_rpm / 9549)/AMK_Eff_Drive
    else:
        AMK_fl_torque = min(t_AMK_fl, AMK_Torque[AMK_rpm], (commandedPower-Emrax_power)/2*AMK_Eff_Drive*9549/(AMK_rpm+.0001))
        AMK_fl_power = (AMK_fl_torque * AMK_rpm / 9549)/AMK_Eff_Drive
        AMK_fr_torque = min(t_AMK_fr, AMK_Torque[AMK_rpm], (commandedPower-Emrax_power-AMK_fl_power)*AMK_Eff_Drive*9549/(AMK_rpm+.0001))
        AMK_fr_power = (AMK_fr_torque * AMK_rpm / 9549)/AMK_Eff_Drive

    
    total_power = Emrax_power + AMK_fr_power + AMK_fl_power

    acc_x = ((AMK_fr_torque*AMK_Gear_Ratio + AMK_fl_torque*AMK_Gear_Ratio + Emrax_r_torque*Emrax_Gear_Ratio) / WHEEL_RADIUS - Fx_Drag) / M
    
    return Emrax_r_torque, AMK_fr_torque, AMK_fl_torque, Emrax_power, AMK_fr_power, AMK_fl_power, total_power, acc_x, Fz_fr, Fz_fl, Fz_rr, Fz_rl

def distanceToBraking(acc_y, next_radius, velocity, car_position, next_curve_position):
    Fx_Drag = -DRAG_COEFF*1/2*AIR_DENSITY*ACS_FRONT*velocity**2
    Fz_Downforce = -LIFT_COEFF*1/2*AIR_DENSITY*ACS_TOP*velocity**2
    Fz_fr = (0.5 * REAR_AXLE_TO_CG / WHEELBASE * M * g) + Fz_Downforce*.4*.5 - Fx_Drag * DRAG_HEIGHT/WHEELBASE
    Fz_fl = (0.5 * REAR_AXLE_TO_CG / WHEELBASE * M * g) + Fz_Downforce*.4*.5 - Fx_Drag * DRAG_HEIGHT/WHEELBASE
    Fz_rr = (0.5 * FRONT_AXLE_TO_CG / WHEELBASE * M * g) + Fz_Downforce*.6*.5 + Fx_Drag * DRAG_HEIGHT/WHEELBASE
    Fz_rl = (0.5 * FRONT_AXLE_TO_CG / WHEELBASE * M * g) + Fz_Downforce*.6*.5 + Fx_Drag * DRAG_HEIGHT/WHEELBASE

    Fz_tot = Fz_fr + Fz_fl + Fz_rr + Fz_rl
    if (M*acc_y)**2 > (Fz_tot * STATIC_FRICTION)**2:
        Fx = 0
    else:
        Fx = np.sqrt((Fz_tot * STATIC_FRICTION)**2 - (M*acc_y)**2)
    Ax_max = Fx / M

    distance = next_curve_position - car_position
    next_velocity = np.sqrt(g*STATIC_FRICTION/next_radius)

    return distance - (velocity**2 - next_velocity**2)/(2*Ax_max)



@dataclass
class VehicleState:
    #velocity, accel x, accel y, power, emrax, amk_fr, amk_fl
    velocity: float # m/s
    a_x: float # m/s^2
    a_y: float # m/s^2
    Fz_fr: float # N
    Fz_fl: float # N
    Fz_rr: float # N
    Fz_rl: float # N    
    emrax_power: float # kW
    amk_fr_power: float # kW
    amk_fl_power: float # kW
    tot_power: float # kW
    used_energy: float # kWh
    recovered_energy: float # kWh
    net_energy: float # kWh
    emrax_tq: float # Nm
    amk_fr_tq: float # Nm
    amk_fl_tq:float #Nm
    time: float #sec
    dist: float #distance
    radius: float #radius
    
    def __str__(self):
        return (f"Vehicle State:\n"
                f"  Velocity:       {self.velocity:.2f} m/s\n"
                f"  Acceleration X: {self.a_x / g:.2f}  Gs\n"
                f"  Acceleration Y: {self.a_y / g:.2f} Gs\n"
                f"  Fz FR:          {self.Fz_fr:.2f} N\n"
                f"  Fz FL:          {self.Fz_fl:.2f} N\n"
                f"  Fz RR:          {self.Fz_rr:.2f} N\n"
                f"  Fz RL:          {self.Fz_rl:.2f} N\n"
                f"  Emrax Power:    {self.emrax_power:.2f} kW\n"
                f"  AMK FR Power:   {self.amk_fr_power:.2f} kW\n"
                f"  AMK FL Power:   {self.amk_fl_power:.2f} kW\n"
                f"  Total Power:    {self.tot_power:.2f} kW\n"
                f"  Used Energy:    {self.used_energy:.2f} kWh\n"
                f"  Recov Energy:   {self.recovered_energy:.2f} kWh\n"
                f"  Net Energy:     {self.net_energy:.2f} kWh\n"
                f"  Emrax Torque:   {self.emrax_tq:.2f} Nm\n"
                f"  AMK FR Torque:  {self.amk_fr_tq:.2f} Nm\n"
                f"  AMK FL Torque:  {self.amk_fl_tq:.2f} Nm\n"
                f"  Time:           {self.time:.2f} sec\n"
                f"  Distance:       {self.dist:.2f} m\n"
                f"  Radius:         {self.radius:.2f} m")

##############################
#       MAIN FUNCTION        #
##############################
TRACK_MODEL = 'track.csv'
# TRACK_MODEL = 'Accel.csv'
dt = 0.01 # seconds
sim_time = 120
    


def main():
    Prep_battery_kWH_array()

    car_properties = [VehicleState(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)] 

    track = pd.read_csv(TRACK_MODEL)
    d_track = track_gen.discretize_track(track, 1)
    track_xy = track_gen.generate_cartesian(d_track)

    radius = track['radius'].values
    distance = track['length'].cumsum().values

    i=0
    while i < sim_time/dt:
        print(car_properties[i])

        Lookahead_Distance  = max(car_properties[i].velocity**2/(2*g*STATIC_FRICTION), 10)
        #find start of any future corners in the lookahead distance, calc their minimum speeds, target those speeds with power or braking
        
        for j in range(len(distance)):
            if ((car_properties[i].dist % max(distance)) <= distance[j]):
                # print(j)
                min_radius = radius[j]
                if j < len(distance)-1:
                    next_radius = radius[j+1]
                else:
                    next_radius = min_radius
                k=j
                break
        
        acc_y = car_properties[i].velocity**2 / min_radius
        if TRACK_MODEL != 'Accel.csv':
            if distanceToBraking(acc_y, next_radius, car_properties[i].velocity, car_properties[i].dist, distance[k]) <= car_properties[i].velocity * dt:
                power_request = -200
            else:
                power_request = 10
        else:
            power_request = 80
        dist = car_properties[i].dist + dt * car_properties[i].velocity
        if dist >= max(distance):
            break
        
        
        
        Emrax_r_torque, AMK_fr_torque, AMK_fl_torque, Emrax_power, AMK_fr_power, AMK_fl_power, total_power, acc_x, Fn_fr, Fn_fl, Fn_rr, Fn_rl  = Drivetrain_Model(power_request, car_properties[i].a_x, abs(car_properties[i].a_y), car_properties[i].velocity)
        
        if car_properties[i].velocity > 33 and acc_x > 0:
            Emrax_r_torque = 0
            AMK_fr_torque = 0
            AMK_fl_torque = 0
            Emrax_power = 0
            AMK_fr_power = 0
            AMK_fl_power = 0
            total_power = 0
            acc_x = 0

        used_energy = car_properties[i].used_energy - total_power * dt / 3600
        recovered_energy = car_properties[i].recovered_energy + 0
        net_energy = car_properties[i].net_energy - used_energy + recovered_energy

        car_properties.append(VehicleState(car_properties[i].velocity + acc_x * dt, acc_x, acc_y, Fn_fr, Fn_fl, Fn_rr, Fn_rl, Emrax_power, AMK_fr_power, AMK_fl_power, total_power, used_energy, recovered_energy, net_energy, Emrax_r_torque, AMK_fr_torque, AMK_fl_torque, i*dt, dist, min_radius))
        i += 1

    time = [state.time for state in car_properties]
    velocities = [state.velocity for state in car_properties]

    # Create a plot
    plt.figure(figsize=(8, 6))
    plt.plot(time, velocities, label='Velocity', color='b')

    # Add labels and title
    plt.title('Velocity vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.grid(True)

    # Show the plot
    plt.legend()
    plt.show()
    
    

if __name__ == "__main__":
    main()
