import collections

class ACC_Controller:
    # States
    CRUISE = "CRUISE"
    FOLLOW = "FOLLOW"
    EMERGENCY_BRAKE = "EMERGENCY_BRAKE"

    def __init__(self, desired_speed=30.0, safe_distance=20.0, time_gap=1.5,
                 sensor_resolution=0.1, max_acceleration=2.0, max_braking=5.0,
                 delay_steps=0):
        self.desired_speed = desired_speed
        self.safe_distance = safe_distance
        self.time_gap = time_gap
        self.sensor_resolution = sensor_resolution
        self.max_acceleration = max_acceleration
        self.max_braking = max_braking

        self.state = self.CRUISE

        # Delay mechanism
        self.delay_steps = delay_steps
        self.measurement_buffer = collections.deque(maxlen=delay_steps + 1)

    def quantize_measurement(self, value):
        if self.sensor_resolution <= 1e-6:
            return value
        return round(value / self.sensor_resolution) * self.sensor_resolution

    def get_control_action(self, ego_v, lead_v, distance):
        # Quantize measurements
        q_ego_v = self.quantize_measurement(ego_v)
        q_lead_v = self.quantize_measurement(lead_v)
        q_distance = self.quantize_measurement(distance)

        # Store in buffer
        self.measurement_buffer.append((q_ego_v, q_lead_v, q_distance))

        # Use oldest available measurement (simulates delay)
        curr_ego_v, curr_lead_v, curr_distance = self.measurement_buffer[0]

        return self._compute_action(curr_ego_v, curr_lead_v, curr_distance)

    def _compute_action(self, ego_v, lead_v, distance):
        desired_following_distance = self.safe_distance + self.time_gap * ego_v

        # State Transitions
        # Priority: Emergency -> Follow -> Cruise

        if distance < self.safe_distance * 0.8:
            self.state = self.EMERGENCY_BRAKE
        elif distance < desired_following_distance:
            self.state = self.FOLLOW
        else:
            # Hysteresis for returning to Cruise
            if self.state == self.FOLLOW and distance > desired_following_distance * 1.1:
                self.state = self.CRUISE
            elif self.state == self.EMERGENCY_BRAKE and distance > self.safe_distance * 1.0:
                 # Recovery from emergency brake usually goes to follow first if close enough
                 # or cruise if far enough.
                 # Let's say if we are safe enough, we go to Follow to re-evaluate or Cruise.
                 if distance > desired_following_distance:
                     self.state = self.CRUISE
                 else:
                     self.state = self.FOLLOW
            elif self.state == self.EMERGENCY_BRAKE:
                 pass
            elif self.state != self.FOLLOW:
                 self.state = self.CRUISE

        # Control Law
        if self.state == self.EMERGENCY_BRAKE:
            return -self.max_braking

        elif self.state == self.CRUISE:
            kp = 0.5
            error = self.desired_speed - ego_v
            acc = kp * error
            return max(min(acc, self.max_acceleration), -self.max_braking)

        elif self.state == self.FOLLOW:
            kp_dist = 0.2
            kp_vel = 0.5

            dist_error = distance - desired_following_distance
            vel_error = lead_v - ego_v

            acc = kp_dist * dist_error + kp_vel * vel_error
            return max(min(acc, self.max_acceleration), -self.max_braking)

        return 0.0
