# Safety Node

The Safety Node implements Automatic Emergency Braking (AEB) through Time to Collision (TTC) calculations. TTC represents the time it would take for the car to collide with an obstacle if it maintained its current heading and velocity.

## iTTC Calculation

We approximate the time to collision using Instantaneous Time to Collision (iTTC), which is calculated as:

\[ iTTC_i(t) = \frac{r_i(t)}{-\dot{r}_i(t)} \]

where:
- \( r_i(t) \) is the instantaneous range measurements.
- \( \dot{r}_i(t) \) is the current range rate.
- The operator \( [-\dot{r}_i(t)]^+ \) ensures meaningful calculations, handling positive and negative values appropriately.

### Calculation Details

- **LaserScan Measurements:** iTTC uses range measurements from the LaserScan message.
- **Longitudinal Velocity Mapping:** Velocity from the Odometry message is mapped to each scan beam's angle using \( \dot{r}_i(t) = v \cos(\theta) \), where \( v \) is the vehicle's longitudinal velocity and \( \theta \) is the beam angle from LaserScan messages.

## Code Implementation

The functionality has been implemented in both Python3 (`scripts/safety_node.py`) and C++ (`src/safety_node.cpp`), providing equivalent features and performance.

## Starting Up the Node

To start the Safety Node:

- **Python Version:**
  ```
  ros2 run safety_node safety_node.py
  ```

- **C++ Version:**
  ```
  ros2 run safety_node safety_node
  ```

Ensure the appropriate node version is chosen based on your development preferences and requirements.

