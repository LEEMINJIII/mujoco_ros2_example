<a name="top"></a>
# MuJoCo-ROS2

> [!NOTE]
> This project is a work-in-progress, so new features will be added slowly.
> Feel free to contribute your own improvements.

This class establishes communication between a MuJoCo simulation and ROS2.

It publishes a `sensor_msgs::msg::JointState` topic for you to use, and allows commands to the joints via a `std_msgs::msg::Float64MultiArray` topic:

<p align = "center">
<img src ="https://github.com/user-attachments/assets/fff5be63-dc23-4c33-97a6-83f376ffccc6" width = "800" height = "auto" />
</p>

It can run in `POSITION`, `VELOCITY`, or `TORQUE` mode which may be set via the config and/or launch file(s).

>[!TIP]
> You can download MuJoCo robot models [here](https://github.com/google-deepmind/mujoco_menagerie.git).

- [Dependencies](#dependencies)
- [Installation](#installation)
   - [Prerequisites](#prerequisites)
   - [Building the Project](#building-the-project)
   - [Usage](#usage)
   - [Launching the Simulation](#launching-the-simulation)
- [Contributing](#contributing)
- [License](#license)


## Dependencies

- [ROS2](https://docs.ros.org/en/humble/index.html) (Humble Hawksbill or later)
- [MuJoCo 3.2.0](https://mujoco.org/) (or later?)
- GLFW
- Standard C++ libraries

[⬆️ Back to top.](#top)

## Installation

### Prerequisites

Ensure that you have ROS2 and MuJoCo installed on your system.

1. **Install ROS2:**
   Follow the [ROS2 installation guide](https://docs.ros.org/en/foxy/Installation.html) for your operating system.

2. **Install MuJoCo:**
   Download and install MuJoCo from the [official website](https://mujoco.org/).

4. **Install GLFW:**
```
   sudo apt-get install libglfw3-dev
```

[⬆️ Back to top.](#top)

### Building the Project

In the `src/` directory of your ROS2 workspace, clone the repository:
```
git clone https://github.com/Woolfrey/mujoco_ros2
cd mujoco_ros2
```
Your directory structure should look something like this:
```
ros2_workspace/
├── build/
├── install/
├── log/
└── src/
    └── mujoco_ros2/
        ├── config/
        ├── include/
        ├── launch/
        ├── model/
        ├── src/
        ├── CMakeLists.txt
        ├── LICENSE
        ├── package.xml
        └── README.md
```
In `CMakeLists.txt` you need to tell the compiler where to find the MuJoCo header files:
```
set(MUJOCO_PATH "/opt/mujoco/mujoco-3.2.0") # UPDATE THIS TO YOUR MUJOCO PATH
include_directories(${MUJOCO_PATH}/include/mujoco)                                                  # MuJoCo header files
include_directories(${MUJOCO_PATH}/include)                                                         # Additional MuJoCo header files
link_directories(${MUJOCO_PATH}/lib)                                                                # Location of MuJoCo libraries
```
As you can see, I've installed it in `/opt/mujoco/mujoco-3.2.0` but you should change it to match your directory and version.

Navigate back to the root of your ROS2 workspace and build the package:
```
colcon build --packages-select mujoco_ros2
```
Source the ROS2 workspace:
```
source install/setup.bash
```

[⬆️ Back to top.](#top)

### Usage

#### Launching the Simulation

> [!NOTE]
> I'm currently only running velocity mode. Torque mode will be made available soon.

To run **velocity control**, you can launch:
```
ros2 launch mujoco velocity_mode.py
```
This requires that the topic `/joint_commands` contains an array of velocites (in rad/s).


[⬆️ Back to top.](#top)

## Contributing

Contributions are welcome! Please fork the repository and submit pull requests.

[⬆️ Back to top.](#top)

## License

Distributed under the GNU General Public License. See LICENSE for more information.
Contact

Jon Woolfrey - jonathan.woolfrey@gmail.com

[⬆️ Back to top.](#top)
