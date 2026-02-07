import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

def plot_results(history, filename="simulation_results.png"):
    """
    Plots the simulation results.

    Args:
        history (dict): Dictionary containing simulation data.
            Keys: 't', 'v_ego', 'v_lead', 'distance', 'state', 'violations'
        filename (str): Output filename for the plot.
    """
    t = history['t']
    v_ego = history['v_ego']
    v_lead = history['v_lead']
    distance = history['distance']
    state = history['state']
    violations = history.get('violations', [])

    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

    # 1. Velocity
    axs[0].plot(t, v_ego, label='Ego Vehicle')
    axs[0].plot(t, v_lead, label='Lead Vehicle', linestyle='--')
    axs[0].set_ylabel('Velocity (m/s)')
    axs[0].set_title('Velocity vs Time')
    axs[0].legend()
    axs[0].grid(True)

    # 2. Distance
    axs[1].plot(t, distance, label='Distance', color='green')
    # Plot safety threshold line if available, but it's dynamic or fixed?
    # Let's assume a fixed safe distance for visualization if possible, or just plot violations.
    # Highlight violations
    if violations:
        violation_times = [v[0] for v in violations]
        violation_dists = [v[1] for v in violations]
        axs[1].scatter(violation_times, violation_dists, color='red', marker='x', label='Safety Violation')

    axs[1].set_ylabel('Distance (m)')
    axs[1].set_title('Distance vs Time')
    axs[1].legend()
    axs[1].grid(True)

    # 3. Controller State
    # Map states to integers for plotting
    state_map = {"CRUISE": 0, "FOLLOW": 1, "EMERGENCY_BRAKE": 2}
    numeric_states = [state_map.get(s, -1) for s in state]

    axs[2].step(t, numeric_states, where='post', color='purple')
    axs[2].set_yticks([0, 1, 2])
    axs[2].set_yticklabels(['CRUISE', 'FOLLOW', 'EMERGENCY'])
    axs[2].set_ylabel('Controller State')
    axs[2].set_title('Controller State vs Time')
    axs[2].set_xlabel('Time (s)')
    axs[2].grid(True)

    plt.tight_layout()
    plt.savefig(filename)
    plt.close()

def animate_simulation(history, filename="simulation.gif"):
    """
    Creates an animation of the simulation.

    Args:
        history (dict): Simulation data.
        filename (str): Output filename.
    """
    t = history['t']
    # x_ego and x_lead are needed. We only have distance.
    # We can integrate velocity or if we stored positions, use them.
    # Let's assume history has 'x_ego' and 'x_lead'. If not, we can reconstruct or just animate distance.
    # The prompt says "Store history of states (position...)" so we should have x_ego.
    # But we need x_lead too.

    x_ego = history.get('x_ego')
    x_lead = history.get('x_lead')

    if x_ego is None or x_lead is None:
        print("Position data missing for animation.")
        return

    fig, ax = plt.subplots(figsize=(10, 2))
    ax.set_xlim(min(min(x_ego), min(x_lead)) - 10, max(max(x_ego), max(x_lead)) + 10)
    ax.set_ylim(-2, 2)
    ax.set_xlabel('Position (m)')
    ax.set_yticks([])
    ax.set_title('Vehicle Simulation')

    ego_car, = ax.plot([], [], 'bo', markersize=10, label='Ego')
    lead_car, = ax.plot([], [], 'ro', markersize=10, label='Lead')
    time_text = ax.text(0.02, 0.9, '', transform=ax.transAxes)

    def init():
        ego_car.set_data([], [])
        lead_car.set_data([], [])
        time_text.set_text('')
        return ego_car, lead_car, time_text

    def update(frame):
        # Update limits to follow cars
        current_x_ego = x_ego[frame]
        current_x_lead = x_lead[frame]

        center = (current_x_ego + current_x_lead) / 2
        ax.set_xlim(center - 50, center + 50)

        ego_car.set_data([current_x_ego], [0])
        lead_car.set_data([current_x_lead], [0])
        time_text.set_text(f'Time: {t[frame]:.1f}s')
        return ego_car, lead_car, time_text

    # Downsample for animation speed (e.g., every 10th frame)
    step = max(1, len(t) // 200) # Limit to ~200 frames
    frames = range(0, len(t), step)

    ani = animation.FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, interval=50)
    ani.save(filename, writer='pillow', fps=20)
    plt.close()
