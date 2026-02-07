import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from vehicle import Vehicle
from controller import ACC_Controller
from safety_monitor import SafetyMonitor
import random

# Import functions from visualization.py directly if in the same dir
from visualization import plot_results, animate_simulation

def main():
    # Simulation Parameters
    T = 60.0  # Total time (s)
    dt_sim = 0.01  # Physics time step (s)
    dt_sample = 0.1  # Controller sampling time (s)

    # Initialize Vehicles
    # Lead starts ahead
    lead_vehicle = Vehicle(position=50.0, velocity=20.0, acceleration=0.0)
    ego_vehicle = Vehicle(position=0.0, velocity=20.0, acceleration=0.0)

    # Initialize Controller
    # Delay: 2 steps delay (2 * 0.1s = 0.2s delay)
    controller = ACC_Controller(desired_speed=25.0, safe_distance=15.0, time_gap=1.5, delay_steps=2)

    # Initialize Safety Monitor
    safety_monitor = SafetyMonitor(safe_distance=10.0)

    # Simulation Loop State
    next_sample_time = 0.0
    current_control_action = 0.0

    # History Storage
    history = {
        't': [],
        'x_ego': [],
        'v_ego': [],
        'a_ego': [],
        'x_lead': [],
        'v_lead': [],
        'distance': [],
        'state': [],
        'violations': []
    }

    # Run Simulation
    # Calculate number of steps
    steps = int(T / dt_sim)

    for step in range(steps):
        t = step * dt_sim

        # 1. Determine Lead Vehicle Acceleration (Scenario)
        if t < 10:
            lead_acc = 0.0
        elif t < 20:
            lead_acc = -1.0 # Slow down
        elif t < 30:
            lead_acc = 0.0 # Constant speed (slower)
        elif t < 40:
            lead_acc = 1.0 # Speed up
        else:
            lead_acc = 0.0 # Constant speed

        # 2. Controller Update (with Jitter)
        if t >= next_sample_time:
            # Measure (using current state)
            distance = lead_vehicle.x - ego_vehicle.x

            # Get Action
            current_control_action = controller.get_control_action(
                ego_vehicle.v, lead_vehicle.v, distance
            )

            # Simulate jitter: varying sampling interval
            jitter = random.uniform(-0.01, 0.01) # Small jitter
            next_sample_time += dt_sample + jitter

        # 3. Physics Update (Both)
        lead_vehicle.update(dt_sim, lead_acc)
        ego_vehicle.update(dt_sim, current_control_action)

        # 4. Safety Monitor
        current_distance = lead_vehicle.x - ego_vehicle.x
        is_safe = safety_monitor.check_safety(current_distance, t + dt_sim)

        # 5. Store History
        history['t'].append(t + dt_sim)
        history['x_ego'].append(ego_vehicle.x)
        history['v_ego'].append(ego_vehicle.v)
        history['a_ego'].append(ego_vehicle.a)
        history['x_lead'].append(lead_vehicle.x)
        history['v_lead'].append(lead_vehicle.v)
        history['distance'].append(current_distance)
        history['state'].append(controller.state)

        if not is_safe:
            history['violations'].append((t + dt_sim, current_distance))

    print(f"Simulation completed. Total steps: {steps}")
    print(f"Safety Violations: {len(history['violations'])}")

    # Visualization
    print("Generating plots...")
    plot_results(history, filename="simulation_results.png")

    print("Generating animation...")
    # Animation might take time, uncomment if needed or run separately
    animate_simulation(history, filename="simulation.gif")
    print("Done.")

if __name__ == '__main__':
    main()
