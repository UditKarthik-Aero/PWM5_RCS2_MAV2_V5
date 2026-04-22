import numpy as np
import matplotlib.pyplot as plt

#  MISSION CONSTANTS 
F_MAX = 7607000     # Peak Thrust (N) - Merlin 1D Class
G = 9.81            
ISP = 282           # Specific Impulse (s)
DRY_MASS = 25600    # kg (F1st Stage)
ROCKET_L = 47.0     # meters

def run_simulation(mode="Pulsed"):
    state = [6000.0, -280.0, 0.0, 0.0, 45000.0]
    dt = 0.01 
    time = 0.0
    fuel_spent = 0.0
    
    # LOGS 
    duty_log, time_log, alt_log, vel_log = [], [], [], []
    
    while state[0] > 0:
        y, vy, theta, omega, mass = state
        thrust_ratio = 0.0
        gimbal = 0.0
        rcs_torque = 0.0
        current_duty = 0.0

        if mode == "SpaceX":
            #  SPACEX GNC (Continuous Hoverslam) 
            req_a = (vy**2) / (2 * y + 1e-6)
            thrust_ratio = np.clip((mass * (req_a + G)) / F_MAX, 0, 1)
            # F9 cannot throttle below 40%
            if thrust_ratio < 0.40 and y > 500: 
                thrust_ratio = 0.0
            current_duty = thrust_ratio
        
        else:
            #  PULSED MAV GNC (Aero-Tilt + Pulsing) 
            if y > 1500: target_theta = np.radians(15)
            elif y > 300: target_theta = np.radians(5)
            else: target_theta = 0.0
            
            if y < 50: rcs_torque = ((-theta) * 10000000) - (omega * 3000000)
            else: rcs_torque = ((target_theta - theta) * 5000000) - (omega * 1500000)
            
            v_thrust_eff = F_MAX * np.cos(theta)
            req_a = (vy**2) / (2 * y + 1e-6)
            duty_cycle = np.clip((mass * (req_a + G)) / (v_thrust_eff + 1), 0, 1)
            current_duty = duty_cycle 
            
            h_stop_trigger = ((vy**2) / (2 * ((F_MAX/mass)-G)) * 1.2)
            if y < h_stop_trigger:
                thrust_ratio = 1.0 if (time % 0.01) < (duty_cycle * 0.01) else 0.0
            
            if thrust_ratio > 0:
                gimbal = np.clip((-theta) * 0.8, -0.4, 0.4)

        # PHYSICS CORE 
        eff_area = (10.7 * np.abs(np.cos(theta))) + (170.0 * np.abs(np.sin(theta)))
        rho = 1.225 * np.exp(-y / 8500) 
        f_drag = 0.5 * rho * (vy**2) * eff_area * 0.8
        
        # Fuel Logic
        if mass > DRY_MASS:
            f_t = thrust_ratio * F_MAX
            fuel_step = (f_t / (ISP * G)) * dt
            if fuel_spent + fuel_step > (45000.0 - DRY_MASS):
                f_t, fuel_step = 0, 0
        else:
            f_t, fuel_step = 0, 0

        fuel_spent += fuel_step
        mass -= fuel_step

        # Kinematics
        ay = (f_t * np.cos(theta + gimbal) + f_drag - (mass * G)) / mass
        vy += ay * dt
        y += vy * dt
        
        moi = (1/12) * mass * (ROCKET_L**2)
        total_torque = (f_t * np.sin(gimbal) * 23.5) + rcs_torque
        omega += (total_torque / moi) * dt
        theta += omega * dt
        
        # LOGGING
        time_log.append(time)
        duty_log.append(current_duty)
        alt_log.append(y)
        vel_log.append(vy)
        time += dt
        state = [y, vy, theta, omega, mass]
        if time > 200: break

    status = "SUCCESS" if (vy > -6.5 and abs(np.degrees(theta)) < 5.0) else "CRASH"
    
    return {
        "status": status,
        "velocity": vy,
        "fuel": fuel_spent,
        "time": time,
        "tilt": np.degrees(theta),
        "logs": (time_log, duty_log, alt_log, vel_log)
    }

# EXECUTION 
spacex_data = run_simulation("SpaceX")
pulsed_mav_data = run_simulation("Pulsed")

# MISSION REPORT)
print("\n" + "="*65)
print(f"{'FINAL MISSION REPORT':^65}")
print("="*65)
print(f"{'METRIC':<25} | {'SPACEX FALCON 9':<18} | {'PULSED MAV':<18}")
print("-" * 65)
print(f"{'Landing Status':<25} | {spacex_data['status']:<18} | {pulsed_mav_data['status']:<18}")
print(f"{'Fuel Consumed (kg)':<25} | {spacex_data['fuel']:>15.1f} kg | {pulsed_mav_data['fuel']:>15.1f} kg")
print(f"{'Touchdown Velocity':<25} | {spacex_data['velocity']:>15.2f} m/s | {pulsed_mav_data['velocity']:>15.2f} m/s")
print(f"{'Flight Duration (s)':<25} | {spacex_data['time']:>15.2f} s | {pulsed_mav_data['time']:>15.2f} s")
print(f"{'Final Tilt Angle':<25} | {spacex_data['tilt']:>15.2f}° | {pulsed_mav_data['tilt']:>15.2f}°")
print("="*65)

# GRAPH 
t_s, d_s, a_s, v_s = spacex_data["logs"]
t_p, d_p, a_p, v_p = pulsed_mav_data["logs"]

plt.figure(figsize=(10, 8))
plt.subplot(2, 1, 1)
plt.plot(t_s, v_s, label="SpaceX Velocity", linestyle="--", color="gray")
plt.plot(t_p, v_p, label="Pulsed MAV Velocity", color="red", linewidth=2)
plt.title(f"Simulation Profile: SpaceX vs Pulsed MAV")
plt.ylabel("Velocity (m/s)")
plt.legend(); plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(t_s, d_s, label="SpaceX Continuous Throttle", linestyle="--", color="gray")
plt.step(t_p, d_p, label="Pulsed MAV Duty Cycle", color="blue")
plt.ylabel("Duty Cycle (D)")
plt.xlabel("Time (s)")
plt.legend(); plt.grid(True)

plt.tight_layout()
plt.show()
# Unpack the logs first
t_p, d_p, a_p, v_p = pulsed_mav_data["logs"]

# This creates the file on your computer
np.savetxt("flight_data.csv", np.column_stack((t_p, a_p, v_p, d_p)), 
           delimiter=",", header="Time,Altitude,Velocity,DutyCycle", comments='')
