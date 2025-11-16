"""
Main entry point for the UAV Waypoint Simulation.

This script is the executable file that brings all the components
(UAV, Mission, Environment, Simulator) together to run a complete 
simulation.

When executed, it will:
1. Define a sample mission (a list of waypoints).
2. Instantiate the simulation components.
3. Generate a smooth trajectory from the waypoints.
4. Run the simulation, which opens a live 3D plot.
"""

from waypoint_sim.uav import UAV
from waypoint_sim.mission import WaypointMission
from waypoint_sim.environment import Environment
from waypoint_sim.simulator import Simulator

if __name__ == "__main__":
    # --- 1. Define the Mission ---
    # A list of (x, y, z) tuples representing the 3D coordinates 
    # of the waypoints in meters.
    waypoints = [(0,0,0), (50,20,10), (100,50,20), (150,0,15)]

    # --- 2. Initialize Simulation Components ---
    # Create an instance of the Unmanned Aerial Vehicle.
    uav = UAV()

    # Create the mission manager, passing it the waypoints.
    mission = WaypointMission(waypoints)

    # Create the simulation environment (which includes wind).
    environment = Environment()

    # Create the main simulator, injecting the uav, mission, 
    # and environment objects.
    sim = Simulator(uav, mission, environment)

    # --- 3. Generate the Flight Path ---
    # Convert the discrete list of waypoints into a smooth, continuous
    # 3D path for the UAV to follow. This uses Cubic Spline interpolation.
    trajectory = sim.generate_smooth_trajectory(waypoints, num_points=1000)
    
    # --- 4. Run the Simulation ---
    # Start the main simulation loop. This will update the UAV state
    # at each time step and display the live 3D plot.
    sim.run(trajectory)
