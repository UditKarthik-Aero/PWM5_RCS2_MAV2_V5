import numpy as np
import matplotlib.pyplot as plt

# --- Constants ---
F_MAX = 7607000
G = 9.81
ISP = 282
MAX_V_SAFE = -5.0

def run_simulation(mode="SpaceX"):
    # [y, vy, theta, omega, mass]
    state = [6000.0, -280.0, 0.0, 0.0, 45000.0]
    dt = 0.01 
    fuel_spent = 0
    time = 0
    
    # Data containers for the graph
    alt_history = []
    vel_history = []
    fuel_history = []
    
    while state[0] > 0:
        y, vy, theta, omega, mass = state
        thrust_ratio = 0.0
        gimbal = 0.0
        rcs_torque = 0.0

        if mode == "SpaceX":
            target_theta = 0.0
            req_a = (vy**2) / (2 * y + 1e-6)
            thrust_ratio = np.clip((mass * (req_a + G)) / F_MAX, 0, 1)
            if thrust_ratio < 0.40 and y > 500: thrust_ratio = 0
        
        else:
            if y > 1500:
                target_theta = np.radians(15) 
            elif y > 300:
                target_theta = np.radians(5)  
            else:
                target_theta = 0.0            
            
            if y < 50:
                rcs_torque = ((-theta) * 10000000) - (omega * 3000000)
            else:
                rcs_torque = ((target_theta - theta) * 5000000) - (omega * 1500000)
            
            v_thrust_eff = F_MAX * np.cos(theta)
            req_a = (vy**2) / (2 * y + 1e-6)
            duty_cycle = np.clip((mass * (req_a + G)) / (v_thrust_eff + 1), 0, 1)
            
            if y < ((vy**2) / (2 * ((F_MAX/mass)-G)) * 1.2):
                thrust_ratio = 1.0 if (time % 0.01) < (duty_cycle * 0.01) else 0.0
            
            if thrust_ratio > 0:
                gimbal = np.clip((-theta) * 0.8, -0.4, 0.4)

        # Physics
        eff_area = (10.7 * np.abs(np.cos(theta))) + (170.0 * np.abs(np.sin(theta)))
        rho = 1.225 * np.exp(-y / 8500)
        f_drag = 0.5 * rho * (vy**2) * eff_area * 0.8
        
        f_t = thrust_ratio * F_MAX if mass > 25600 else 0
        fuel_step = (f_t / (ISP * G)) * dt
        fuel_spent += fuel_step
        mass -= fuel_step

        ay = (f_t * np.cos(theta + gimbal) + f_drag - (mass * G)) / mass
        vy += ay * dt
        y += vy * dt
        
        moi = (1/12) * mass * (47**2)
        total_torque = (f_t * np.sin(gimbal) * 23.5) + rcs_torque
        alpha = total_torque / moi
        omega += alpha * dt
        theta += omega * dt
        
        # Save data
        alt_history.append(y)
        vel_history.append(vy)
        fuel_history.append(fuel_spent)
        
        time += dt
        state = [y, vy, theta, omega, mass]
        if time > 150: break

    return alt_history, vel_history, fuel_spent

# --- Run Sims ---
s_alt, s_vel, s_fuel_total = run_simulation("SpaceX")
y_alt, y_vel, y_fuel_total = run_simulation("Yours")

# --- Plotting ---
plt.style.use('seaborn-v0_8-darkgrid') # Gives it a clean, academic look
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

# Plot 1: Velocity vs Altitude
ax1.plot(s_alt, s_vel, label='SpaceX Falcon 9 (95/100 Baseline)', color='red', linestyle='--')
ax1.plot(y_alt, y_vel, label='Your MAV (Pulsed + Aero-Tilt)', color='blue', linewidth=2)
ax1.set_xlabel('Altitude (m)')
ax1.set_ylabel('Vertical Velocity (m/s)')
ax1.set_title('Deceleration Profile Comparison')
ax1.invert_xaxis() # Altitude goes from 6000 -> 0
ax1.legend()

# Plot 2: Total Fuel Consumption
bars = ax2.bar(['SpaceX F9', 'Your MAV'], [s_fuel_total, y_fuel_total], color=['red', 'blue'])
ax2.set_ylabel('Fuel Consumed (kg)')
ax2.set_title('Total Propellant Expenditure')

# Add values on top of bars
for bar in bars:
    height = bar.get_height()
    ax2.text(bar.get_x() + bar.get_width()/2., height + 50,
             f'{int(height)} kg', ha='center', va='bottom', fontweight='bold')

plt.tight_layout()
plt.show()
