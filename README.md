TRASH_BOY ROVER SIMULATION

Trash_Boy is a ROS2-based simulated rover designed for research and development in teleoperation, autonomous navigation, and swarm robotics. This repository provides a Gazebo simulation environment, URDF robot model, and ROS2 control interface.

FEATURES (CURRENT)

* Fully teleoperated rover using ROS2 cmd_vel input.
* Simulated in Gazebo with accurate wheel dynamics using mecanum drive.
* Robot description in URDF/Xacro format.
* Robot state publishing for TF and joint state visualization.
* ROS2 control integration for wheel and chassis controllers.

PLANNED FEATURES (FUTURE DEVELOPMENT)

1) Autonomous Navigation

	* Integration of LiDAR, IMU, and camera sensors.
	* Path planning and obstacle avoidance using ROS2 Navigation stack.

2) Swarm Operations

	* Decentralized Swarm: Each rover makes local decisions based on sensor input and peer communication.
	* Centralized Swarm: A single coordinator node manages multiple rovers for coordinated tasks.

3) Multi-agent simulation in Gazebo for testing swarm behaviors.

4) Advanced sensor simulation and perception stack for real-world-like testing.

INSTALLATION: 
# Clone the repository
	git clone https://github.com/athiyan-s/Trash_boy.git
	cd trash_boy

# Build workspace
	colcon build
	source install/setup.bash

RUNNING THE SIMULATION
Launch the robot in Gazebo:

	ros2 launch trash_boy robot_gazebo.launch.py
	
CONTROL THE ROBOT MANUALLY:
	
	python3 src/trash_boy/trash_boy/mecanum_teleop.py
	

	
FILE STRUCTURE
	trash_boy/
	├── launch/                 # Launch files
	├── urdf/                   # URDF/Xacro robot model
	├── config/                 # Controller YAML and other configs
	├── src/                    # ROS2 nodes (future autonomous & swarm nodes)
	├── README.md               # Project description

DEPENDENCIES

* ROS2 Humble
* Gazebo ROS2 packages
* ros2_control and ros2_controllers
* Teleop keyboard package for manual testing

CONTRIBUTION

* Currently in teleoperation mode only.
* Contributions are welcome for autonomous navigation, swarm coordination, sensor integration, and controller tuning.
