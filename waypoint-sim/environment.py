import numpy as np

class Environment:
    def __init__(self, wind_base=(0.2,0,0), gust_std=(0.05,0.0,0.02), noise_std=0.2):
        self.wind_base = wind_base
        self.gust_std = gust_std
        self.noise_std = noise_std

    def get_wind(self):
        return tuple(self.wind_base[i] + np.random.randn()*self.gust_std[i] for i in range(3))
