# Follow the Gap Package

## I. Learning Goals

- Reactive methods for obstacle avoidance

## II. Overview

In this pkg, we will try to implement a reactive algorithm for obstacle avoidance.

## III. Review of Follow the Gap

However, the steps are outlined over here:

1. Obtain laser scans and preprocess them.
2. Find the closest point in the LiDAR ranges array.
3. Draw a safety bubble around this closest point and set all points inside this bubble to 0. All other non-zero points are now considered “gaps” or “free space”.
4. Find the max length “gap”, in other words, the largest number of consecutive non-zero elements in your ranges array.
5. Find the best goal point in this gap. Naively, this could be the furthest point away in your gap, but you can probably go faster if you follow the “Better Idea” method as described in lecture.
6. Actuate the car to move towards this goal point by publishing an `AckermannDriveStamped` to the /drive topic.

### IV. Implementation

Implement a gap follow algorithm to make the car drive autonomously around the Levine Hall map. We can implement this node in either C++ or Python. There are two extra test maps `levine_blocked.png`:![blocked](./levine_blocked.png), which is empty, and `levine_obs.png`:![obstacle](./levine_obs.png), which has obstacles that are relatively hard to navigate through for us to evaluate our code on.

To change the map in the simulation, add the included `.png` and `.yaml` map files to `simulator/maps` directory. Then, change `simulator/config/sim.yaml` to use your desired map.

### V. Extra Resources

UNC Follow the Gap Video: <https://youtu.be/ctTJHueaTcY>
