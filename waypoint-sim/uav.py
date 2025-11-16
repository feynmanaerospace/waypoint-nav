import math
import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.min_out, self.max_out = output_limits

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.prev_error = error
        if self.min_out is not None:
            output = max(self.min_out, output)
        if self.max_out is not None:
            output = min(self.max_out, output)
        return output

class UAV:
    def __init__(self, x=0, y=0, z=0, max_speed=15.0, max_acceleration=5.0, drag=0.1):
        self.x = x
        self.y = y
        self.z = z
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.yaw = 0
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.drag = drag

        self.yaw_pid = PIDController(2.0, 0.0, 0.3, (-math.radians(30), math.radians(30)))
        self.alt_pid = PIDController(1.5, 0.0, 0.2, (-5.0, 5.0))
        self.speed_pid = PIDController(1.0, 0.0, 0.1, (0, self.max_speed))

    def update(self, target_pos, dt, wind=(0,0,0), noise_std=0.0):
        tx, ty, tz = target_pos

        noisy_x = self.x + np.random.normal(0, noise_std)
        noisy_y = self.y + np.random.normal(0, noise_std)
        noisy_z = self.z + np.random.normal(0, noise_std)

        dx = tx - noisy_x
        dy = ty - noisy_y
        dz = tz - noisy_z

        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self.yaw
        yaw_rate = self.yaw_pid.update(yaw_error, dt)
        self.yaw += yaw_rate * dt

        distance_xy = math.hypot(dx, dy)
        desired_speed = self.speed_pid.update(distance_xy, dt)
        desired_speed = min(self.max_speed, desired_speed)
        vx_desired = math.cos(self.yaw) * desired_speed
        vy_desired = math.sin(self.yaw) * desired_speed

        ax = max(-self.max_acceleration, min(self.max_acceleration, vx_desired - self.vx))
        ay = max(-self.max_acceleration, min(self.max_acceleration, vy_desired - self.vy))
        self.vx += ax * dt
        self.vy += ay * dt

        self.vx *= (1 - self.drag*dt)
        self.vy *= (1 - self.drag*dt)

        self.vz = self.alt_pid.update(dz, dt)
        self.x += (self.vx + wind[0]) * dt
        self.y += (self.vy + wind[1]) * dt
        self.z += (self.vz + wind[2]) * dt
