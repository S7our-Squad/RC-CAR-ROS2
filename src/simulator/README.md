# The Simulator

This is a containerized ROS2 package that turns into a simulation for our RC Car.

# Installation

**Supported System:**

- Windows 10, macOS, and Ubuntu without an NVIDIA gpu (using [noVNC](https://novnc.com/info.html))

**Install the following dependencies:**

- **Docker** Follow the instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/) to install Docker. A short tutorial can be found [here](https://docs.docker.com/get-started/) if you're not familiar with Docker. If you followed the post-installation steps you won't have to prepend your docker and docker-compose commands with sudo.
- Additionally you'll need **docker-compose**. Follow the instruction [here](https://docs.docker.com/compose/install/) to install docker-compose.

## Getting Started

1. Clone this repo

  ```bash
  git clone https://github.com/S7our-Squad/RC-CAR-ROS2.git
  cd RC-CAR-ROS2
  ```

2. Bringup the noVNC container and the sim container with docker-compose:

  ```bash
  docker compose up
  ```

3. In a separate terminal, run the following, and you'll have the a bash session in the simulation container. [`tmux`](https://hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/) is available for convenience.

```bash
docker exec -it rc-car-ros2-sim-1 /bin/bash
```

4. In your browser, navigate to [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html), you should see the noVNC logo with the connect button. Click the connect button to connect to the session.

## Launching the Simulation

1. `tmux` is included in the container, so you can create multiple bash sessions in the same terminal.
2. To launch the simulation, make sure you source both the ROS2 setup script and the local workspace setup script. Run the following in the bash session from the container:

```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch simulator gym_bridge_launch.py
```

A rviz window should pop up showing the simulation either on your host system or in the browser window depending on the display forwarding you chose.

You can then run another node by creating another bash session in `tmux`.

# Configuring the simulation

- The configuration file for the simulation is at `simulator/config/sim.yaml`.
- Topic names and namespaces can be configured but is recommended to leave uncahnged.
- The map can be changed via the `map_path` parameter. You'll have to use the full path to the map file in the container. The map follows the ROS convention. It is assumed that the image file and the `yaml` file for the map are in the same directory with the same name. See the note below about mounting a volume to see where to put your map file.
- The `num_agent` parameter can be changed to either 1 or 2 for single or two agent racing.
- The ego and opponent starting pose can also be changed via parameters, these are in the global map coordinate frame.

The entire directory of the repo is mounted to a workspace `/sim_ws/src` as a package. All changes made in the repo on the host system will also reflect in the container. After changing the configuration, run `colcon build` again in the container workspace to make sure the changes are reflected.

# Keyboard Teleop

The keyboard teleop node from `teleop_twist_keyboard` is also installed as part of the simulation's dependency. To enable keyboard teleop, set `kb_teleop` to `True` in `sim.yaml`. After launching the simulation, in another terminal, run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Then, press `i` to move forward, `u` and `o` to move forward and turn, `,` to move backwards, `m` and `.` to move backwards and turn, and `k` to stop in the terminal window running the teleop node.


# updated gym bridge 

```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install -y \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-ackermann-msgs \
    ros-humble-tf2-ros \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2

# Navigate to your package directory
cd /path/to/your/workspace/src/simulator

# Install Python requirements
pip install -r requirements.txt

# Or install with pip3
pip3 install -r requirements.txt
```