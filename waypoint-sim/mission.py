"""
Defines the WaypointMission class for managing mission state.

This module contains the logic for tracking the UAV's progress through
a predefined list of waypoints.
"""

import math

class WaypointMission:
    """
    Manages the sequence of waypoints and tracks mission progress.

    This class holds the list of all waypoints for a mission and keeps
    track of which waypoint is currently active. It updates the mission
    state by checking if the UAV has "arrived" at the current target.

    Attributes:
        waypoints (list): A list of (x, y, z) tuples representing the
                          mission's waypoints in meters.
        current_index (int): The index of the currently active waypoint
                             in the `waypoints` list.
        tolerance (float): The spherical radius (in meters) around a
                           waypoint. The UAV is considered to have
                           "arrived" if it enters this radius.
    """
    def __init__(self, waypoints, arrival_tolerance=1.0):
        """
        Initializes the WaypointMission.

        Args:
            waypoints (list): A list of (x, y, z) tuples defining the
                              3D path.
            arrival_tolerance (float, optional): The arrival radius in
                                                 meters. Defaults to 1.0.
        """
        self.waypoints = waypoints
        self.current_index = 0     # Start at the first waypoint
        self.tolerance = arrival_tolerance

    def get_current_waypoint(self):
        """
        Retrieves the coordinates of the currently active waypoint.

        Returns:
            tuple: The (x, y, z) coordinates of the current waypoint, or
                   None if the mission is complete.
        """
        # Check if the index is still within the bounds of the list
        if self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]
        # All waypoints have been reached
        return None

    def update(self, uav):
        """
        Updates the mission state based on the UAV's position.

        This method checks the distance from the UAV to the current
        waypoint. If the distance is within the arrival tolerance,
        it increments the waypoint index to the next one.

        Args:
            uav (UAV): The UAV object, used to get its current
                       (x, y, z) position.
        """
        # Get the target we are currently flying towards
        wp = self.get_current_waypoint()

        # If wp is None, the mission is over, so do nothing.
        if wp is None:
            return
        
        # --- Check for Waypoint Arrival ---
        # Calculate the 3D distance between the UAV and the waypoint
        # using the Euclidean distance formula: sqrt(dx^2 + dy^2 + dz^2)
        dx = wp[0] - uav.x
        dy = wp[1] - uav.y
        dz = wp[2] - uav.z

        # If the distance is within the tolerance, we have "arrived"
        if math.sqrt(dx**2 + dy**2 + dz**2) < self.tolerance:
            print(f"Reached waypoint {self.current_index}: {wp}")
            # Advance to the next waypoint in the list
            self.current_index += 1
