"""
Defines the main Simulator class.

This module contains the core simulation engine. It is responsible for:
1. Generating a smooth trajectory from discrete waypoints.
2. Running the main simulation loop at discrete time steps.
3. Updating all simulation components (UAV, Mission, Environment).
4. Visualizing the simulation in a 3D plot using matplotlib.
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline

class Simulator:
    """
    Core simulation engine.

    This class orchestrates the entire simulation. It takes the UAV,
    Mission, and Environment objects and runs a time-step-based loop,
    updating the state of the UAV and plotting its position.

    Attributes:
        uav (UAV): The UAV instance to be simulated.
        mission (WaypointMission): The mission manager instance.
        env (Environment): The environment instance.
        dt (float): The simulation time step (delta-time) in seconds.
        positions (list): A history of the UAV's (x, y, z) positions
                          for plotting its path.
    """

    def __init__(self, uav, mission, environment, dt=0.05):
        """
        Initializes the Simulator.

        Args:
            uav (UAV): An instance of the UAV class.
            mission (WaypointMission): An instance of the WaypointMission class.
            environment (Environment): An instance of the Environment class.
            dt (float, optional): The simulation time step in seconds.
                                  Defaults to 0.05 (20 Hz).
        """
        self.uav = uav
        self.mission = mission
        self.env = environment
        self.dt = dt
        self.positions = []    # To store the UAV's path history

    def generate_smooth_trajectory(self, waypoints, num_points=500):
        """
        Generates a smooth 3D path from a list of waypoints.

        This method uses Cubic Spline Interpolation (from scipy) to create a
        continuous and smooth path that passes through all specified
        waypoints. This prevents instantaneous changes in direction.

        Args:
            waypoints (list): A list of (x, y, z) waypoint tuples.
            num_points (int, optional): The number of points to sample
                                        from the spline to create the
                                        final trajectory array. Defaults to 500.

        Returns:
            np.ndarray: An (N, 3) NumPy array representing the 3D coordinates
                        of the smoothed trajectory, where N = num_points.
        """

        # Convert waypoints to a NumPy array for easier slicing
        waypoints = np.array(waypoints)

        # Create a parameter 't' for the spline, 0 to 1,
        # with one 't' value for each waypoint.
        t = np.linspace(0, 1, len(waypoints))

        # --- Create separate 1D splines for each dimension (x, y, z) ---
        # The spline interpolates x as a function of t, y as a function of t, etc.
        cs_x = CubicSpline(t, waypoints[:,0])
        cs_y = CubicSpline(t, waypoints[:,1])
        cs_z = CubicSpline(t, waypoints[:,2])

        # Generate the new, densely populated 't' values for the smooth path
        ts = np.linspace(0, 1, num_points)

        # Sample the splines at the new 'ts' values
        # and stack them into a single (N, 3) array.
        trajectory = np.stack([cs_x(ts), cs_y(ts), cs_z(ts)], axis=1)
        
        return trajectory

    def run(self, trajectory):
        """
        Runs the main simulation loop.

        This method iterates through each point in the provided trajectory,
        using it as the target for the UAV's PID controllers. It updates
        the UAV's state, checks the mission status, and renders the 3D plot.

        Args:
            trajectory (np.ndarray): The (N, 3) array of target points
                                     from generate_smooth_trajectory().
        """

        # Enable interactive plotting mode in matplotlib
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # --- Main Simulation Loop ---
        # The UAV's control system follows the 'trajectory' (pos),
        # while the 'mission' object just checks for arrival at the
        # *original* waypoints.
        for pos in trajectory:
            # Check if the mission is complete (all waypoints reached)
            if self.mission.get_current_waypoint() is None:
                print("Mission complete!")
                break     # Exit the simulation loop
            
            # --- 1. Get Environmental Factors ---
            wind = self.env.get_wind()

            # --- 2. Update UAV State ---
            # The 'pos' from the trajectory is the *target* position
            # for this time step.
            self.uav.update(pos, self.dt, wind=wind, noise_std=self.env.noise_std)
            
            # --- 3. Update Mission Status ---
            # Check if the UAV has reached the *current waypoint*
            self.mission.update(self.uav)
            
            # --- 4. Record History ---
            # Store the UAV's *actual* position
            self.positions.append((self.uav.x, self.uav.y, self.uav.z))

            # --- 5. Update Plot (Live) ---
            ax.clear()     # Clear previous frame

            # Unzip the position history for plotting
            xs, ys, zs = zip(*self.positions)

            # Plot the smooth target trajectory (gray)
            ax.plot(trajectory[:,0], trajectory[:,1], trajectory[:,2], color='gray', label='Trajectory')
            
            # Plot the UAV's actual followed path (implicit in the blue dot's trail)
            # and its current position (blue dot)
            ax.scatter(self.uav.x, self.uav.y, self.uav.z, color='blue', s=50, label='UAV')
            
            # Plot the original mission waypoints (red dots)
            wp_x, wp_y, wp_z = zip(*self.mission.waypoints)
            ax.scatter(wp_x, wp_y, wp_z, color='red', s=30, label='Waypoints')
            
            # Set plot limits and labels
            # Note: Limits are hardcoded, may need adjustment for new missions
            ax.set_xlim(0, 160); ax.set_ylim(0, 60); ax.set_zlim(0, 25)
            ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
            ax.legend()
            
            # Pause briefly to allow the plot to update and create
            # a real-time animation effect
            plt.pause(0.01)

        # --- Simulation End ---
        # Disable interactive mode and show the final plot
        plt.ioff()
        plt.show()
