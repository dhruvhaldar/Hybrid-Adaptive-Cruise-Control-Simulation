class Vehicle:
    def __init__(self, position=0.0, velocity=0.0, acceleration=0.0):
        self.x = position
        self.v = velocity
        self.a = acceleration

    def update(self, dt, control_input):
        """
        Updates the vehicle state using explicit Euler integration.

        Args:
            dt (float): Time step in seconds.
            control_input (float): The control input (acceleration) for this step.
        """
        # Update acceleration
        self.a = control_input

        # Update position based on current velocity
        self.x += self.v * dt

        # Update velocity based on current acceleration
        self.v += self.a * dt
