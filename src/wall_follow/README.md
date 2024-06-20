PID in the Time Domain

A PID controller adjusts a system to maintain parameters around a desired set point using:
u(t)=Kpe(t)+Ki∫e(t)dt+Kdde(t)dtu(t)=Kp​e(t)+Ki​∫e(t)dt+Kd​dtde(t)​
Wall Following

In this lab, the car autonomously follows walls by:

    Using laser scans aa and bb to determine distances.
    Calculating the angle θθ to the wall and projecting the car's future position.
    Applying a PID algorithm to compute a steering angle.
    Adjusting speed based on the steering angle for safe driving.

Implementation

Implement wall following to autonomously navigate the Levine Hall map, following inner walls (left if going counter-clockwise). Use either C++ or Python for implementation.
Running the Code

- **Python Version:**
  ```
  ros2 run wall_follow wall_follow.py
  ```

- **C++ Version:**
  ```
  ros2 run wall_follow_node wall_follow_node
  ```

Ensure smooth execution in the simulation environment provided.
