import math

class WaypointMission:
    def __init__(self, waypoints, arrival_tolerance=1.0):
        self.waypoints = waypoints
        self.current_index = 0
        self.tolerance = arrival_tolerance

    def get_current_waypoint(self):
        if self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]
        return None

    def update(self, uav):
        wp = self.get_current_waypoint()
        if wp is None:
            return
        dx = wp[0] - uav.x
        dy = wp[1] - uav.y
        dz = wp[2] - uav.z
        if math.sqrt(dx**2 + dy**2 + dz**2) < self.tolerance:
            print(f"Reached waypoint {self.current_index}: {wp}")
            self.current_index += 1
