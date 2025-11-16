# waypoint-nav
This repository is a lightweight Python simulator for a 3D Unmanned Aerial Vehicle (UAV) navigating a series of waypoints. It features a kinematic UAV model controlled by PID controllers, a simple physics and environment model, and cubic spline interpolation for smooth path generation. The simulation is visualised in real-time.
## Overview
This project simulates a drone's flight along a predefined mission. Instead of simply flying from point to point, the simulator first generates a smooth, continuous trajectory that passes through all waypoints. The UAV then attempts to follow this trajectory, managed by independent PID controllers for altitude, speed, and heading. The simulation loop updates the UAV's state at discrete time steps (dt), factoring in environmental effects and control system outputs.

The project is structured into four main components:
* __UAV__: The vehicle model, including its state (position, velocity) and control systems.
* __Environment__: Simulates external forces, specifically wind (base + gust) and sensor noise.
* __WaypointMission__: Manages the mission logic, tracking which waypoint is currently active.
* __Simulator__: The main engine that generates the trajectory, runs the simulation loop, and handles plotting.

## Scientific Context
1. __PID Control__
The UAV's movement is governed by three independent PID (Proportional-Integral-Derivative) controllers. A PID controller is a classic control loop mechanism that calculates an "error" value as the difference between a measured process variable and a desired setpoint. It attempts to minimize the error over time by adjusting a control output.

The controller's output is the sum of three terms:
* __Proportional (P)__: Proportional to the current error (kp * error). This provides an immediate response.
* __Integral (I)__: Proportional to the accumulation of past errors (ki * integral). This helps eliminate steady-state error.
* __Derivative (D)__: Proportional to the rate of change of the error (kd * derivative). This predicts future error and dampens oscillations.
The equation is: output = (kp * error) + (ki * integral) + (kd * derivative).

In this simulation:
* __Altitude PID__: Controls vertical velocity (vz) based on the altitude error (dz).
* __Yaw PID__: Controls the yaw rate (rate of change of heading) based on the error between the UAV's current yaw and the target yaw.
* __Speed PID__: Controls the desired horizontal speed based on the 2D distance to the target. This desired speed is then used to calculate target horizontal velocities (vx_desired, vy_desired)

2. __Kinematics Model & Physics__
The UAV is modeled as a point mass with a simplified kinematic model, not a full rigid-body dynamics simulation.
* __Position Update:__ The position is updated using the first-order Euler method: position_new = position_old + velocity * dt.
* __Wind:__ Wind is modeled as a simple additive velocity. The final velocity used for the position update is (self.vx + wind[0]), (self.vy + wind[1]), and (self.vz + wind[2]).
* __Drag:__ A simple linear drag model is applied, which reduces the velocity by a fixed fraction each time step: self.vx *= (1 - self.drag*dt).
* __Constraints:__ The model enforces physical limits by clamping the maximum acceleration and speed.

3. __Trajectory Generation__
To create a smooth flight path from a discrete set of waypoints [(x1, y1, z1), (x2, y2, z2), ...], the simulator uses Cubic Spline Interpolation.

This method fits a series of piecewise cubic polynomials to the waypoints. This ensures that the resulting path is continuous and has continuous first and second derivatives, which translates to a path without instantaneous changes in velocity or acceleration. This is crucial for a realistic and physically-followable flight path. The scipy.interpolate.CubicSpline function is used to generate separate splines for the x, y, and z dimensions against a normalized time parameter t.

4. __Stochastic Elements__
The simulation includes two sources of randomness to model real-world imperfections:
* __Wind Gusts__: The Environment provides a get_wind() method that adds random noise (from np.random.randn()) to a base wind vector, simulating gusts.
* __Sensor Noise__: Before the UAV calculates its error for the PID controllers, it "reads" its position with added Gaussian noise (np.random.normal(0, noise_std)). This simulates an imperfect GPS or INS (Inertial Navigation System).

## Installation
1. Clone the repository:

```
git clone https://github.com/feynmanaerospace/waypoint-nav.git
cd waypoint-nav
```

2. Create and activate a virtual environment (Recommended):

```
python -m venv venv

source venv/bin/activate
# On Windows, use venv\Scripts\activate
```

3. Install the required dependencies from requirements.txt:

```
pip install -r requirements.txt
```

The main dependencies are numpy, scipy, and matplotlib.

## Usage
To run the simulation, simply execute the main.py script:
```python main.py```
This will open a matplotlib 3D plot window showing the simulation in real-time. The plot displays:
* __Waypoints__ (Red dots)
* __Smooth Trajectory__ (Gray line)
* __UAV Position__ (Blue dot, showing its current path)

## Example
```
from waypoint_sim.uav import UAV
from waypoint_sim.mission import WaypointMission
from waypoint_sim.environment import Environment
from waypoint_sim.simulator import Simulator

if __name__ == "__main__":
    # Define the mission waypoints (x, y, z)
    waypoints = [(0,0,0), (50,20,10), (100,50,20), (150,0,15)]

    # Initialize the components
    uav = UAV()
    mission = WaypointMission(waypoints)
    environment = Environment()
    sim = Simulator(uav, mission, environment)

    # Generate a smooth trajectory with 1000 points
    trajectory = sim.generate_smooth_trajectory(waypoints, num_points=1000)
    
    # Run the simulation
    sim.run(trajectory)
```
  
