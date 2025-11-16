"""
Defines the simulation environment.

This module contains the Environment class, which is responsible for
modeling external physical conditions that affect the UAV, such as
wind and sensor noise.
"""

import numpy as np

class Environment:
    """
    Simulates the physical environment, including wind and sensor noise.

    This class holds the parameters for environmental effects and provides
    methods to generate stochastic values (like wind) during the simulation.

    Attributes:
        wind_base (tuple): The average, steady-state wind vector (wx, wy, wz)
                           in m/s.
        gust_std (tuple): The standard deviation (x, y, z) for the random
                          wind gusts, used to scale Gaussian noise.
        noise_std (float): The standard deviation for sensor noise (e.g., GPS
                           position noise) in meters. This value is stored here
                           and passed to the UAV by the simulator.
    """

    def __init__(self, wind_base=(0.2,0,0), gust_std=(0.05,0.0,0.02), noise_std=0.2):
        """
        Initializes the environment with specified parameters.

        Args:
            wind_base (tuple, optional): The base wind vector (wx, wy, wz)
                                         in m/s. Defaults to (0.2, 0, 0).
            gust_std (tuple, optional): The standard deviation of wind gusts
                                        (std_x, std_y, std_z). Defaults to
                                        (0.05, 0.0, 0.02).
            noise_std (float, optional): The standard deviation of sensor
                                         measurement noise in meters.
                                         Defaults to 0.2.
        """
        self.wind_base = wind_base
        self.gust_std = gust_std
        self.noise_std = noise_std

    def get_wind(self):
        """
        Calculates the current wind vector, including random gusts.

        The wind is modeled as the base wind plus a random component
        drawn from a normal (Gaussian) distribution (using np.random.randn)
        which is then scaled by the gust standard deviation for each axis.

        Returns:
            tuple: A (wx, wy, wz) tuple representing the total current
                   wind vector in m/s.
        """
        # Calculate wind: Base + (Random_Normal_Value * Gust_Std_Dev)
        # This is done for each of the 3 axes (i=0, 1, 2).
        return tuple(self.wind_base[i] + np.random.randn()*self.gust_std[i] for i in range(3))
