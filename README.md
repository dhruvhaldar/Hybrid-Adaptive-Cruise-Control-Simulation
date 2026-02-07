# Hybrid Adaptive Cruise Control (ACC) Simulation

This project implements a Hybrid Adaptive Cruise Control (ACC) system for the KTH course EL2450 (Hybrid and Embedded Control Systems). It models the continuous dynamics of two vehicles (Lead and Ego) and a discrete hybrid controller that switches between different modes of operation.

## Features

- **Hybrid Automata Controller**: Switches between `CRUISE`, `FOLLOW`, and `EMERGENCY_BRAKE` modes based on distance and relative velocity.
- **Sampled-Data Simulation**: Simulates digital controller behavior with fixed sampling time ($dt_{sample}$) and jitter/delay.
- **Quantization**: Simulates sensor quantization (rounding) for realistic measurement noise.
- **Safety Monitor**: Runtime verification of the safety invariant ($Distance > Safe\_Distance$).
- **Visualization**: Generates static plots and animations of the simulation.

## Hybrid Automata Model

The controller is modeled as a Hybrid Automaton with the following states:

1.  **CRUISE**:
    -   **Condition**: Distance to lead vehicle is large.
    -   **Control**: Regulate speed to `desired_speed`.
2.  **FOLLOW**:
    -   **Condition**: Distance is within following range.
    -   **Control**: Maintain safe distance and match lead vehicle speed.
3.  **EMERGENCY_BRAKE**:
    -   **Condition**: Distance is critically low ($< 0.8 \times Safe\_Distance$).
    -   **Control**: Apply maximum braking.

**Transitions**:
-   `CRUISE` $\to$ `FOLLOW`: Distance decreases below safe following distance.
-   `FOLLOW` $\to$ `CRUISE`: Distance increases significantly (hysteresis).
-   `*` $\to$ `EMERGENCY_BRAKE`: Distance drops below critical threshold.
-   `EMERGENCY_BRAKE` $\to$ `FOLLOW`/`CRUISE`: Distance recovers.

## Control Logic & Implementation Details

### Physics Model (`vehicle.py`)
-   Point-mass model: $\dot{p} = v, \dot{v} = a$.
-   Explicit Euler integration.

### Controller (`controller.py`)
-   **Quantization**: Measurements are rounded to the nearest `sensor_resolution`.
-   **Delay/Jitter**: A buffer simulates processing delay. Jitter is simulated in the main loop by varying sampling intervals.
-   **Control Law**: P-control for Cruise, PD-like control for Follow.

### Safety Verification (`safety_monitor.py`)
-   Checks $d(t) > d_{safe}$ at every simulation step.
-   Logs violations for analysis.

## Relation to Syllabus

This project demonstrates key concepts from EL2450:
-   **Hybrid Control**: Interaction between continuous vehicle dynamics and discrete control logic.
-   **Sampling & Hold**: The controller runs at a lower frequency than the physics simulation (ZOH).
-   **Quantization**: Effects of limited sensor precision.
-   **Verification**: Runtime monitoring of safety specifications.

## Running the Simulation

1.  **Install Dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

2.  **Run Simulation**:
    ```bash
    python main.py
    ```

3.  **Run Tests**:
    ```bash
    python -m unittest discover tests
    ```

## Results

### Static Plots
The following plot shows the velocity profile, inter-vehicle distance, and controller state transitions over time.

![Simulation Results](simulation_results.png)

### Animation
An animation of the vehicles is also generated: [simulation.gif](simulation.gif)

## Source Code

-   [main.py](main.py): Simulation orchestration.
-   [controller.py](controller.py): Hybrid ACC controller logic.
-   [vehicle.py](vehicle.py): Vehicle physics model.
-   [safety_monitor.py](safety_monitor.py): Safety invariant checker.
-   [visualization.py](visualization.py): Plotting and animation functions.
