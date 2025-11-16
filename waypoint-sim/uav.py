"""
Defines the UAV and PIDController classes.

This module contains the core physics and control logic for the simulation:
1.  PIDController: A general-purpose PID controller.
2.  UAV: A kinematic model of a drone, which uses three PID controllers
    to manage its flight (yaw, speed, and altitude).
"""

import math
import numpy as np

class PIDController:
    """
    Implements a standard PID (Proportional-Integral-Derivative) controller.
    
    A PID controller attempts to minimize the error between a measured
    process variable and a desired setpoint by adjusting a control output.
    
    Attributes:
        kp (float): Proportional gain.
        ki (float): Integral gain.
        kd (float): Derivative gain.
        integral (float): The accumulated integral of the error over time.
        prev_error (float): The error from the previous update step.
        min_out (float): The minimum value the output can be clamped to.
        max_out (float): The maximum value the output can be clamped to.
    """

    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        """
        Initializes the PID controller.

        Args:
            kp (float): The Proportional gain.
            ki (float): The Integral gain.
            kd (float): The Derivative gain.
            output_limits (tuple): A (min, max) tuple to clamp the output.
                                   Use None for no limit (e.g., (None, 5.0)).
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # State variables
        self.integral = 0
        self.prev_error = 0

        # Output clamping limits
        self.min_out, self.max_out = output_limits

    def update(self, error, dt):
        """
        Calculates the controller output for a given error and time step.

        The PID formula is:
        output = (kp * error) + (ki * integral) + (kd * derivative)

        Args:
            error (float): The current error (target - actual).
            dt (float): The time step (delta-time) in seconds. This must
                        be non-zero to avoid a ZeroDivisionError.

        Returns:
            float: The calculated control output, clamped to the
                   output_limits.
        """

        # --- Integral Term ---
        # Accumulates error over time. Helps correct steady-state error.
        self.integral += error * dt

        # --- Derivative Term ---
        # Responds to the rate of change of the error. Helps dampen
        # oscillations and predict future error.
        # Handle dt=0 case to prevent division by zero.
        derivative = (error - self.prev_error) / dt
        
        # --- PID Calculation ---
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        
        # Update state for next iteration
        self.prev_error = error

        # --- Output Clamping ---
        # Enforce the output limits
        if self.min_out is not None:
            output = max(self.min_out, output)
        if self.max_out is not None:
            output = min(self.max_out, output)
        return output

class UAV:
    """
    Represents the simulated Unmanned Aerial Vehicle (UAV).

    This class models the UAV as a point mass with a kinematic control
    system. It does not use a full rigid-body dynamics model but rather
    uses PID controllers to directly manage its yaw rate, horizontal
    speed, and vertical velocity.

    Attributes:
        x, y, z (float): Current position in meters.
        vx, vy, vz (float): Current velocity in m/s.
        yaw (float): Current heading in radians.
        max_speed (float): The maximum allowed horizontal speed in m/s.
        max_acceleration (float): Max horizontal acceleration in m/s^2.
        drag (float): A simple linear drag coefficient.
        
        yaw_pid (PIDController): Controls yaw rate to face the target.
        alt_pid (PIDController): Controls vertical velocity (vz) to
                                 match target altitude.
        speed_pid (PIDController): Controls desired horizontal speed based
                                   on distance to target.
    """

    def __init__(self, x=0, y=0, z=0, max_speed=15.0, max_acceleration=5.0, drag=0.1):
        """
        Initializes the UAV's state and its PID controllers.

        Args:
            x, y, z (float, optional): Initial position. Defaults to (0,0,0).
            max_speed (float, optional): Max horizontal speed (m/s).
            max_acceleration (float, optional): Max horizontal acceleration (m/s^2).
            drag (float, optional): Linear drag coefficient.
        """

        # --- State Variables ---
        self.x = x
        self.y = y
        self.z = z
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.yaw = 0     # Heading in radians

        # --- Physics / Model Parameters ---
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.drag = drag

        # --- Control Systems ---
        # Yaw PID: Output is yaw *rate* (rad/s)
        self.yaw_pid = PIDController(2.0, 0.0, 0.3, (-math.radians(30), math.radians(30)))
        
        # Altitude PID: Output is vertical *velocity* (m/s)
        # This is a kinematic, not dynamic, controller.
        self.alt_pid = PIDController(1.5, 0.0, 0.2, (-5.0, 5.0))
        
        # Speed PID: Output is desired horizontal *speed* (m/s)
        self.speed_pid = PIDController(1.0, 0.0, 0.1, (0, self.max_speed))

    def update(self, target_pos, dt, wind=(0,0,0), noise_std=0.0):
        """
        Updates the UAV's state for one time step.

        This method performs the full control and physics loop:
        1.  Simulates sensor noise on its position measurement.
        2.  Calculates errors to the target position.
        3.  Runs PID controllers for yaw, speed, and altitude.
        4.  Applies physics (acceleration, drag, wind).
        5.  Updates its position using Euler integration.

        Args:
            target_pos (tuple): The (x, y, z) target position for this step.
            dt (float): The time step in seconds.
            wind (tuple, optional): The wind vector (wx, wy, wz) in m/s.
            noise_std (float, optional): Standard deviation of Gaussian noise
                                         to add to position measurement,
                                         simulating sensor error.
        """

        tx, ty, tz = target_pos

        # --- 1. Simulate Sensor Noise ---
        # The UAV calculates its control based on a *noisy* measurement
        # of its current position.
        noisy_x = self.x + np.random.normal(0, noise_std)
        noisy_y = self.y + np.random.normal(0, noise_std)
        noisy_z = self.z + np.random.normal(0, noise_std)

        # --- 2. Calculate Errors ---
        # Errors are calculated relative to the noisy position
        dx = tx - noisy_x
        dy = ty - noisy_y
        dz = tz - noisy_z

        # --- 3. Run Control Systems ---
        
        # --- Yaw Control ---
        # Calculate desired heading (yaw) to point at the target
        target_yaw = math.atan2(dy, dx)
        # Calculate the shortest angle to the target yaw
        yaw_error = target_yaw - self.yaw
        # Get desired yaw *rate* from the PID controller
        yaw_rate = self.yaw_pid.update(yaw_error, dt)
        # Update the UAV's yaw
        self.yaw += yaw_rate * dt

        # --- Horizontal Speed Control ---
        # Get desired speed based on 2D distance to target
        distance_xy = math.hypot(dx, dy)
        desired_speed = self.speed_pid.update(distance_xy, dt)
        
        # Clamp speed to the UAV's maximum
        desired_speed = min(self.max_speed, desired_speed)
        
        # Calculate the desired velocity components based on our
        # *current* yaw and *desired* speed
        vx_desired = math.cos(self.yaw) * desired_speed
        vy_desired = math.sin(self.yaw) * desired_speed

        # --- 4. Apply Horizontal Physics ---
        
        # Calculate acceleration needed to reach desired horizontal velocity
        # Clamp acceleration to the UAV's maximum
        ax = max(-self.max_acceleration, min(self.max_acceleration, vx_desired - self.vx))
        ay = max(-self.max_acceleration, min(self.max_acceleration, vy_desired - self.vy))
        
        # Update velocity using Euler integration (v = v_0 + a*t)
        self.vx += ax * dt
        self.vy += ay * dt

        # Apply linear drag
        # This reduces velocity by a small fraction each time step
        self.vx *= (1 - self.drag*dt)
        self.vy *= (1 - self.drag*dt)

        # --- Altitude Control (Kinematic) ---
        # The PID controller directly sets the vertical velocity.
        # This is not physically realistic (it ignores mass/thrust/gravity)
        # but is simple and effective for this simulation.
        self.vz = self.alt_pid.update(dz, dt)
        
        # --- 5. Update Position (Euler Integration) ---
        # The final position update includes the effects of the
        # UAV's own velocity *and* the external wind.
        self.x += (self.vx + wind[0]) * dt
        self.y += (self.vy + wind[1]) * dt
        self.z += (self.vz + wind[2]) * dt
