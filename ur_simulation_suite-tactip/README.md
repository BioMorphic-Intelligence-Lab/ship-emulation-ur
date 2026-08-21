# Ursim suite of tools

Here are few examples of all the possible uses of ursim, the official UR robos controller simulator [[win](https://www.universal-robots.com/download/software-ur20ur30/simulator-non-linux/offline-simulator-e-series-and-ur20ur30-ur-sim-for-non-linux-5161/)],[[linux](https://www.universal-robots.com/download/software-ur20ur30/simulator-linux/offline-simulator-e-series-and-ur20ur30-ur-sim-for-linux-5161/)].


Docker compose is used for the setup, pulling the official images from Universal Robots and RoboDK. If you intend to use on your real robot, please make sure you use the correct [official ursim tag](https://hub.docker.com/r/universalrobots/ursim_e-series/tags) corresponding to you phisical controller Polyscope version. 

## Usage
Please change the `.env` file to match you setup parameters:

| Parameter              | Description                                                     |
| ---------------------- | --------------------------------------------------------------- |
| `ROBOT_MODEL=UR5`      | Robot model to simulate: `UR3`, `UR5` (default), `UR10`, `UR16` |
| `POLYSCOPE=5.11.11`    | Polyscope software version                                      |
| `TZ=Europe/Copenhagen` | Timezone to use. Defaults to Europe/Copenhagen.                 |

To use with official [ROS drivers](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver), the official [URCap external-control plugin](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap) needs to be installed while building the image.
TODO
**Currently not working, the file sha is corrupted or something?**
More info here: https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/user_docs/installation/install_urcap_e_series.html#install-urcap-e-series

## Other references
TODO
| Port       | Function                                                           |
| ---------- | ------------------------------------------------------------------ |
| `-p 5900`  | Allows VNC access to the robots interface.                         |
| `-p 502`   | Allows access to Universal Robots Modbus port.                     |
| `-p 29999` | Allows access to Universal Robots dashboard server interface port. |
| `-p 30001` | Allows access to Universal Robots primary interface port.          |
| `-p 30002` | Allows access to Universal Robots secondary interface port.        |
| `-p 30003` | Allows access to Universal Robots real-time interface port.        |
| `-p 30004` | Allows access to Universal Robots RTDE interface port.             |

RoboDK image for off-line programming:
TODO

[Setup URSim with Docker](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/installation/ursim_docker.html#external-control)