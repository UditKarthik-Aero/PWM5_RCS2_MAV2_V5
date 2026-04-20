import numpy as np

# --- MISSION CONSTANTS ---
F_MAX = 7607000     # Peak Thrust (N) - Merlin 1D Class
G = 9.81            # Gravity (m/s^2)
ISP = 282           # Specific Impulse (s)
DRY_MASS = 25600    # kg (F1st Stage)
ROCKET_L = 47.0     # meters

def run_simulation(mode="Yours"):
    # State Vector: [Altitude(m), Velocity(m/s), Angle(rad), Angular_Velo(rad/s), Mass(kg)]
    state = [6000.0, -280.0, 0.0, 0.0, 45000.0]
    dt = 0.01 
    time = 0.0
    fuel_spent = 0.0
    
    # Simulation Loop
    while state[0] > 0:
        y, vy, theta, omega, mass = state
        thrust_ratio = 0.0
        gimbal = 0.0
        rcs_torque = 0.0

        if mode == "SpaceX":
            # --- SPACEX GNC (Continuous Hoverslam) ---
            target_theta = 0.0
            # Calculate required acceleration to stop at y=0
            req_a = (vy**2) / (2 * y + 1e-6)
            thrust_ratio = np.clip((mass * (req_a + G)) / F_MAX, 0, 1)
            
            # Real-world constraint: F9 cannot throttle below 40%
            # If required thrust is < 40%, engine stays OFF (Freefall phase)
            if thrust_ratio < 0.40 and y > 500: 
                thrust_ratio = 0.0
        
        else:
            # --- YOUR PULSED MAV GNC (Aero-Tilt + Pulsing) ---
            # 1. Orientation Logic (The Aero-Brake)
            if y > 1500:
                target_theta = np.radians(15) # Optimized angle for drag/recovery
            elif y > 300:
                target_theta = np.radians(5)  # Alignment glide
            else:
                target_theta = 0.0            # Final landing verticality
            
            # 2. Leveraged RCS (Cold Gas at Nose)
            # High damping (omega) to kill momentum for MIT-level realism
            if y < 50:
                rcs_torque = ((-theta) * 10000000) - (omega * 3000000)
            else:
                rcs_torque = ((target_theta - theta) * 5000000) - (omega * 1500000)
            
            # 3. Adaptive Pulsed Engine Controller
            # Adjusts for the loss of vertical thrust due to tilt (cos theta)
            v_thrust_eff = F_MAX * np.cos(theta)
            req_a = (vy**2) / (2 * y + 1e-6)
            duty_cycle = np.clip((mass * (req_a + G)) / (v_thrust_eff + 1), 0, 1)
            
            # Stop-distance trigger with 20% safety margin for pulse lag
            h_stop_trigger = ((vy**2) / (2 * ((F_MAX/mass)-G)) * 1.2)
            if y < h_stop_trigger:
                # Digital Pulsing at 100Hz
                thrust_ratio = 1.0 if (time % 0.01) < (duty_cycle * 0.01) else 0.0
            
            # 4. Engine Gimbal Assist
            if thrust_ratio > 0:
                gimbal = np.clip((-theta) * 0.8, -0.4, 0.4)

        # --- PHYSICS CORE ---
        # Atmospheric Drag (Nose area 10.7m^2 vs Side area 170m^2)
        eff_area = (10.7 * np.abs(np.cos(theta))) + (170.0 * np.abs(np.sin(theta)))
        rho = 1.225 * np.exp(-y / 8500) # Simple barometric model
        f_drag = 0.5 * rho * (vy**2) * eff_area * 0.8
        
        # Thrust and Fuel Consumption
        f_t = thrust_ratio * F_MAX if mass > DRY_MASS else 0
        fuel_step = (f_t / (ISP * G)) * dt
        fuel_spent += fuel_step
        mass -= fuel_step

        # Linear Kinematics (Y-Axis)
        ay = (f_t * np.cos(theta + gimbal) + f_drag - (mass * G)) / mass
        vy += ay * dt
        y += vy * dt
        
        # Rotational Kinematics (Momentum Tracking)
        moi = (1/12) * mass * (ROCKET_L**2)
        # Total Torque = (Gimbaled Engine Force) + (RCS Cold Gas)
        total_torque = (f_t * np.sin(gimbal) * 23.5) + rcs_torque
        alpha = total_torque / moi
        omega += alpha * dt
        theta += omega * dt
        
        time += dt
        state = [y, vy, theta, omega, mass]
        
        # Emergency Cutoff
        if time > 200: break

    # Final Mission Assessment
    is_soft = vy > -6.0
    is_upright = abs(np.degrees(theta)) < 3.0
    status = "SUCCESS" if is_soft and is_upright else "CRASH"
    
    return {
        "status": status,
        "velocity": vy,
        "fuel": fuel_spent,
        "time": time,
        "tilt": np.degrees(theta)
    }

# --- EXECUTION ---
spacex_data = run_simulation("SpaceX")
your_data = run_simulation("Yours")

print("-" * 50)
print(f"{'METRIC':<20} | {'SPACEX F9':<12} | {'YOUR MAV':<12}")
print("-" * 50)
print(f"{'Mission Status':<20} | {spacex_data['status']:<12} | {your_data['status']:<12}")
print(f"{'Landing Velocity':<20} | {spacex_data['velocity']:>9.2f} m/s | {your_data['velocity']:>9.2f} m/s")
print(f"{'Fuel Consumed':<20} | {spacex_data['fuel']:>9.1f} kg  | {your_data['fuel']:>9.1f} kg")
print(f"{'Flight Time':<20} | {spacex_data['time']:>9.2f} s   | {your_data['time']:>9.2f} s")
print(f"{'Final Tilt':<20} | {spacex_data['tilt']:>9.2f}°   | {your_data['tilt']:>9.2f}°")
print("-" * 50)

import csv

# Save the results to a file named 'telemetry.csv'
with open('telemetry.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['altitude', 'velocity', 'fuel_mav', 'fuel_spacex']) # Headers
    # Zip your logs together and save them
    writer.writerows(zip(h_log, v_log, fuel_log_mav, fuel_log_spacex))

print("Telemetry saved to telemetry.csv")
