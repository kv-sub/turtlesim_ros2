# ROS 2 Turtle Simulation

## Project Overview
This project implements a **ROS 2-based simulation** where a **master turtle** autonomously navigates the environment to "catch" other turtles. The simulation is built using **ROS 2's communication paradigms**, including topics, services, and custom messages.

## Features
- **Autonomous Master Turtle** – Uses a **P-controller** to navigate toward target turtles.
- **Dynamic Turtle Spawning** – A dedicated `turtle_spawner` node handles spawning and tracking multiple turtles.
- **Custom ROS 2 Messages and Services** – Enables efficient communication for status updates and interactions between turtles.
- **Real-time Interaction** – The master turtle continuously updates its trajectory to capture target turtles dynamically.

## Project Structure
```
ros2_turtle_sim/
├── turtlesim_control/
│   ├── src/
│   │   ├── turtle_spawner.cpp
│   │   ├── turtle_controller.cpp
│   ├── include/
│   │   ├── turtle_spawner.hpp
│   │   ├── turtle_controller.hpp
│   ├── msg/
│   │   ├── TurtleStatus.msg
│   ├── srv/
│   │   ├── SpawnTurtle.srv
│   ├── launch/
│   │   ├── turtle_sim.launch.py
│   ├── CMakeLists.txt
│   ├── package.xml
```


## Installation & Setup
### Prerequisites
Ensure you have **ROS 2 Humble** installed. If not, follow the [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html).

### Build Instructions
1. Navigate to your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
2. Clone the repository:
   ```bash
    git clone <repository-url>
3. Build the package:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select turtlesim_control
4. Source the workspace:
    ```bash
    source install/setup.bash
## Usage
## Launching the Simulation
Run the following command to start the turtlesim environment and nodes:
      
    ros2 launch turtlesim_control turtle_sim.launch.py

## Spawning Additional Turtles
You can spawn a new turtle using the ROS 2 service:
      
    ros2 service call /spawn_turtle turtlesim_control/SpawnTurtle "{name: 'turtle2', x: 5.0, y: 5.0}"
## Viewing Turtle Status
To check the status of all spawned turtles, echo the topic:
    
    ros2 topic echo /turtle_status

## Future Enhancements
Implementing more advanced path-planning algorithms.
Enhancing turtle behavior with AI-based decision-making.
Integrating visualization tools like RViz.

## License
This project is open-source and available under the MIT License.

## Author
Subramanian KV
