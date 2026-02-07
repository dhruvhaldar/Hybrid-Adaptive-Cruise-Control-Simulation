class SafetyMonitor:
    def __init__(self, safe_distance=10.0):
        """
        Initializes the Safety Monitor.

        Args:
            safe_distance (float): The minimum safe distance.
        """
        self.safe_distance = safe_distance
        self.violations = []

    def check_safety(self, distance, time):
        """
        Checks if the safety invariant holds.

        Args:
            distance (float): Current distance to the lead vehicle.
            time (float): Current simulation time.

        Returns:
            bool: True if safe, False if violation.
        """
        if distance < self.safe_distance:
            self.violations.append((time, distance))
            return False
        return True
