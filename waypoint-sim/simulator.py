import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline

class Simulator:
    def __init__(self, uav, mission, environment, dt=0.05):
        self.uav = uav
        self.mission = mission
        self.env = environment
        self.dt = dt
        self.positions = []

    def generate_smooth_trajectory(self, waypoints, num_points=500):
        waypoints = np.array(waypoints)
        t = np.linspace(0, 1, len(waypoints))
        cs_x = CubicSpline(t, waypoints[:,0])
        cs_y = CubicSpline(t, waypoints[:,1])
        cs_z = CubicSpline(t, waypoints[:,2])
        ts = np.linspace(0, 1, num_points)
        trajectory = np.stack([cs_x(ts), cs_y(ts), cs_z(ts)], axis=1)
        return trajectory

    def run(self, trajectory):
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        for pos in trajectory:
            if self.mission.get_current_waypoint() is None:
                print("Mission complete!")
                break

            wind = self.env.get_wind()
            self.uav.update(pos, self.dt, wind=wind, noise_std=self.env.noise_std)
            self.mission.update(self.uav)
            self.positions.append((self.uav.x, self.uav.y, self.uav.z))

            ax.clear()
            xs, ys, zs = zip(*self.positions)
            ax.plot(trajectory[:,0], trajectory[:,1], trajectory[:,2], color='gray', label='Trajectory')
            ax.scatter(self.uav.x, self.uav.y, self.uav.z, color='blue', s=50, label='UAV')
            wp_x, wp_y, wp_z = zip(*self.mission.waypoints)
            ax.scatter(wp_x, wp_y, wp_z, color='red', s=30, label='Waypoints')
            ax.set_xlim(0, 160); ax.set_ylim(0, 60); ax.set_zlim(0, 25)
            ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
            ax.legend()
            plt.pause(0.01)

        plt.ioff()
        plt.show()
