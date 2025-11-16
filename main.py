from waypoint_sim.uav import UAV
from waypoint_sim.mission import WaypointMission
from waypoint_sim.environment import Environment
from waypoint_sim.simulator import Simulator

if __name__ == "__main__":
    waypoints = [(0,0,0), (50,20,10), (100,50,20), (150,0,15)]

    uav = UAV()
    mission = WaypointMission(waypoints)
    environment = Environment()
    sim = Simulator(uav, mission, environment)

    trajectory = sim.generate_smooth_trajectory(waypoints, num_points=1000)
    sim.run(trajectory)
