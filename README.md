# HybridACC : EL2450 Hybrid Adaptive Cruise Control (ACC) system

This project implements a Hybrid Adaptive Cruise Control (ACC) system for [KTH Royal Institute of Technology : EL2450 Hybrid and Embedded Control Systems](https://www.kth.se/student/kurser/kurs/EL2450). It models the continuous dynamics of two vehicles (Lead and Ego) and a discrete hybrid controller that switches between different modes of operation.

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

1.  **Create and Activate Virtual Environment**:
    
    *   Create the environment:
        ```bash
        python -m venv venv
        ```
    *   Activate the environment:
        *   Windows: `.\venv\Scripts\activate`
        *   macOS/Linux: `source venv/bin/activate`

2.  **Install Dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

3.  **Run Simulation**:
    ```bash
    python main.py
    ```

4.  **Run Tests**:
    ```bash
    python -m unittest discover tests
    ```

## Results

### Static Plots
The following plots analyze the system's behavior over the simulation duration:

1.  **Velocity vs Time**:
    -   **Blue Line**: Velocity of the Ego vehicle.
    -   **Orange Dashed Line**: Velocity of the Lead vehicle.
    -   Shows how the Ego vehicle adjusts its speed to match the Lead vehicle or maintain its set cruise speed.

2.  **Distance vs Time**:
    -   **Green Line**: Distance between the two vehicles.
    -   **Red 'x' Markers**: Indicate safety violations where the distance fell below the safe threshold.

3.  **Controller State vs Time**:
    -   **Purple Step Line**: The active state of the Hybrid Automaton (`CRUISE`, `FOLLOW`, `EMERGENCY_BRAKE`).
    -   Demonstrates the discrete switching logic in response to changing distance and velocity conditions.

![Simulation Results](simulation_results.png)

### Animation
An animation of the vehicles is also generated to visualize the spatial dynamics:
-   **Blue Dot**: Ego Vehicle.
-   **Red Dot**: Lead Vehicle.
-   The view pans to keep both vehicles centered as they travel.

![Simulation Animation](simulation.gif)

## Source Code

-   [main.py](main.py): Simulation orchestration.
-   [controller.py](controller.py): Hybrid ACC controller logic.
-   [vehicle.py](vehicle.py): Vehicle physics model.
-   [safety_monitor.py](safety_monitor.py): Safety invariant checker.
-   [visualization.py](visualization.py): Plotting and animation functions.

## License

MIT License
