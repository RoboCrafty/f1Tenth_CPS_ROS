# IMPORTANT NOTE

Anyone viewing this repository is strongly encouraged to go through the commit history. The commit log is very detailed and contains many different commits for a wide variety of tasks, making it a valuable resource for understanding the project's development and the rationale behind specific changes.

# F1Tenth CPS Lab

A ROS-based autonomous vehicle simulation and control project for the F1Tenth platform. This repository contains simulation environments, controllers, and utility nodes for developing and testing autonomous driving algorithms.

## Project Structure

```
f1Tenth_CPS_ROS/
├── build/                # Build artifacts (auto-generated)
├── devel/                # Development environment (auto-generated)
├── src/                  # Source code and ROS packages
│   ├── day1/             # Introductory ROS nodes and exercises
│   │   ├── launch/
│   │   ├── src/
│   │   │   ├── arith_avg.cpp
│   │   │   ├── avg_receiver_all.cpp
│   │   │   ├── emitter.cpp
│   │   │   ├── geo_avg.cpp
│   │   │   ├── har_avg.cpp
│   │   │   ├── med.cpp
│   │   │   ├── pub.cpp
│   │   │   ├── receiver.cpp
│   │   │   ├── sub.cpp
│   │   │   └── user_input.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── f110_simulator/   # Main simulator and vehicle nodes
│   │   ├── include/
│   │   ├── launch/
│   │   ├── lidar_env/
│   │   ├── maps/
│   │   ├── node/
│   │   │   ├── aeb.cpp
│   │   │   ├── js_driver.cpp
│   │   │   ├── lidar.py
│   │   │   ├── pid_controller.cpp
│   │   │   ├── pid_projection_controller.cpp
│   │   │   ├── random.cpp
│   │   │   └── simulator.cpp
│   │   ├── params.yaml
│   │   ├── racecar.xacro
│   │   ├── src/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── our_controller/   # Custom controllers and experiments
│   │   ├── launch/
│   │   ├── src/
│   │   │   ├── gap_follow.cpp
│   │   │   ├── keyboard_teleop.cpp
│   │   │   ├── mux.cpp
│   │   │   └── test.txt
│   │   ├── CMakeLists.txt
│   │   └── package.xml
├── .catkin_workspace
├── CMakeLists.txt
└── .gitignore
```

## Key Packages

- **day1**: Basic ROS publisher/subscriber exercises and simple algorithms.
- **f110_simulator**: The main simulation environment, including vehicle dynamics, sensor simulation, and various controllers.
- **our_controller**: Custom controllers, such as gap following, with speed modulation

## How to Build

```bash
cd f1Tenth_CPS_ROS
catkin_make
source devel/setup.bash
```

**Note:**
If you encounter compilation errors, you likely need to install missing dependencies. For example, if you see errors related to `ackermann_msgs`, you can install it with:

```bash
sudo apt-get install ros-<distro>-ackermann-msgs
```
Replace `<distro>` with your ROS distribution (e.g., `melodic`, `noetic`).

Similarly, install any other missing ROS packages as indicated by the error messages.
```

## How to Run

Launch files are provided in each package's `launch/` directory. For example, to run the simulator:

```bash
roslaunch f110_simulator simulator.launch
```

## Notable Nodes

- `gap_follow.cpp`: Implements the gap-following algorithm for autonomous navigation.
- `keyboard_teleop.cpp`: Manual control via keyboard.
- `mux.cpp`: Multiplexes between different control sources.
- `pid_controller.cpp`: PID-based steering controller.
- `simulator.cpp`: Main simulation node.

## Contributing

Feel free to open issues or submit pull requests for improvements and bug fixes.
