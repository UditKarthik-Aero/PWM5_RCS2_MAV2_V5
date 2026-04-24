import numpy as np
import matplotlib.pyplot as plt

# --- MISSION CONSTANTS ---
F_MAX = 7607000     
G = 9.81            
ISP = 282           
DRY_MASS = 25600    
ROCKET_L = 47.0     
AGGRESSION_LEVEL = 0.1 

def run_simulation(mode="Pulsed"):
    state = [6000.0, -280.0, 0.0, 0.0, 45000.0]
    dt = 0.01 
    time = 0.0
    fuel_spent = 0.0
    max_stress = 0.0
    
    logs = {"t": [], "d": [], "a": [], "v": [], "s": []}
    
    while state[0] > 0:
        y, vy, theta, omega, mass = state
        thrust_ratio = 0.0
        
        # Trigger Calculation
        max_decel = (F_MAX / mass) - G
        h_trigger = ((vy**2) / (2 * max_decel)) * (1.0 + AGGRESSION_LEVEL)

        if mode == "SpaceX":
            target_theta = np.radians(1.5) 
            if y < h_trigger:
                req_a = (vy**2) / (2 * y + 1e-6)
                thrust_ratio = np.clip((mass * (req_a + G)) / F_MAX, 0.4, 1.0)
            current_duty = thrust_ratio
        else:
            target_theta = np.radians(15) if y > 1500 else 0.0
            if y < h_trigger:
                v_eff = F_MAX * np.cos(theta)
                req_a = (vy**2) / (2 * y + 1e-6)
                duty_cycle = np.clip((mass * (req_a + G)) / (v_eff + 1), 0, 1)
                thrust_ratio = 1.0 if (time % 0.01) < (duty_cycle * 0.01) else 0.0
                current_duty = duty_cycle 
            else:
                current_duty = 0.0

        # Physics & Stress
        rho = 1.225 * np.exp(-y / 8500) 
        q = 0.5 * rho * (vy**2)
        stress = q * np.abs(np.sin(theta)) * 10
        if stress > max_stress: max_stress = stress

        eff_area = (10.7 * np.abs(np.cos(theta))) + (170.0 * np.abs(np.sin(theta)))
        f_drag = q * eff_area * 0.8
        
        if mass > DRY_MASS:
            f_t = thrust_ratio * F_MAX
            fuel_step = (f_t / (ISP * G)) * dt
            fuel_spent += fuel_step
            mass -= fuel_step
        else: f_t = 0

        ay = (f_t * np.cos(theta) + f_drag - (mass * G)) / mass
        vy += ay * dt
        y += vy * dt
        
        rcs_torque = ((target_theta - theta) * 5000000) - (omega * 1500000)
        moi = (1/12) * mass * (ROCKET_L**2)
        omega += (rcs_torque / moi) * dt
        theta += omega * dt
        
        for k, v in zip(logs.keys(), [time, current_duty, y, vy, stress]):
            logs[k].append(v)
            
        time += dt
        state = [y, vy, theta, omega, mass]
        if time > 200: break

    status = "SUCCESS" if (vy > -7.5) else "CRASH"
    return {"status": status, "v": vy, "f": fuel_spent, "s": max_stress, "logs": logs}

# --- EXECUTION ---
spacex = run_simulation("SpaceX")
pulsed = run_simulation("Pulsed")

# --- SHELL REPORT ---
print("\n" + "="*75)
print(f"{'MISSION ANALYSIS: STRESS & FUEL COMPARISON':^75}")
print("="*75)
print(f"{'METRIC':<25} | {'SPACEX (1.5° TILT)':<22} | {'PULSED MAV (15° TILT)':<22}")
print("-" * 75)
print(f"{'Landing Status':<25} | {spacex['status']:<22} | {pulsed['status']:<22}")
print(f"{'Fuel Consumed':<25} | {spacex['f']:>15.1f} kg | {pulsed['f']:>15.1f} kg")
print(f"{'Touchdown Velocity':<25} | {spacex['v']:>15.2f} m/s | {pulsed['v']:>15.2f} m/s")
print(f"{'Max Bending Stress':<25} | {spacex['s']:>15.0f} Pa | {pulsed['s']:>15.0f} Pa")
print("="*75)

# --- GRAPHING BLOCK ---
plt.figure(figsize=(12, 10))

# 1. Velocity Plot
plt.subplot(3, 1, 1)
plt.plot(spacex['logs']['t'], spacex['logs']['v'], label="SpaceX (Baseline)", color='black', alpha=0.5, linestyle='--')
plt.plot(pulsed['logs']['t'], pulsed['logs']['v'], label="Pulsed MAV", color='red', linewidth=2)
plt.title("Landing Physics Comparison")
plt.ylabel("Velocity (m/s)")
plt.legend(); plt.grid(True, alpha=0.3)

# 2. Bending Stress Plot
plt.subplot(3, 1, 2)
plt.fill_between(spacex['logs']['t'], spacex['logs']['s'], color='gray', alpha=0.2, label="SpaceX Stress")
plt.plot(pulsed['logs']['t'], pulsed['logs']['s'], label="Pulsed MAV Stress", color='darkorange', linewidth=2)
plt.ylabel("Bending Stress (Pa)")
plt.legend(); plt.grid(True, alpha=0.3)

# 3. Duty Cycle / Thrust Plot
plt.subplot(3, 1, 3)
plt.plot(spacex['logs']['t'], spacex['logs']['d'], label="SpaceX Continuous", color='blue', alpha=0.4)
plt.step(pulsed['logs']['t'], pulsed['logs']['d'], label="Pulsed Duty Cycle", color='blue', where='post')
plt.xlabel("Mission Time (s)")
plt.ylabel("Thrust Ratio (D)")
plt.legend(); plt.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()
