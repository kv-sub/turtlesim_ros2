# ROS2 Learning Project

## Project Overview
This project was developed as part of my learning journey in ROS2. While I gained foundational knowledge from a Udemy course, I personally implemented the code and structured the project to deepen my understanding of ROS2 concepts such as publishers, subscribers, messages, services, and action servers.

## Features
- **Autonomous Master Turtle** – Uses a **P-controller** to navigate toward target turtles.
- **Dynamic Turtle Spawning** – A dedicated `turtle_spawner` node handles spawning and tracking multiple turtles.
- **Custom ROS 2 Messages and Services** – Enables efficient communication for status updates and interactions between turtles.
- **Real-time Interaction** – The master turtle continuously updates its trajectory to capture target turtles dynamically.

## Project Structure
```
src
├── my_py_pkg
│   ├── my_py_pkg
│   │   ├── __pycache__
│   │   ├── Python nodes (publishers, subscribers, services, etc.)
│   ├── resource
│   ├── test
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
├── my_robot_bringup
│   ├── launch
│   ├── CMakeLists.txt
│   └── package.xml
├── my_robot_interfaces
│   ├── msg
│   ├── srv
│   ├── CMakeLists.txt
│   └── package.xml
└── my_turtle_sim_catch
    ├── my_turtle_sim_catch
    ├── resource
    ├── test
    ├── package.xml
    ├── setup.cfg
    └── setup.py
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
    git clone https://github.com/kv-sub/turtlesim_ros2.git
3. Build the package:
    ```bash
    cd turtlesim_ros2
    colcon build
4. Source the workspace:
    ```bash
    source install/setup.bash

## Running the Project
To launch the different functionalities:
- Run a publisher node:
 
      ros2 run my_py_pkg number_publisher
- Run a subscriber node:

      ros2 run my_py_pkg number_counter
- Launch turtlesim simulation:

      ros2 launch my_robot_bringup turtlesim_catch_them_all.launch.py
  
## Spawning Additional Turtles
You can spawn a new turtle using the ROS 2 service:
      
    ros2 service call /spawn_turtle turtlesim_control/SpawnTurtle "{name: 'turtle2', x: 5.0, y: 5.0}"
    
## Viewing Turtle Status
To check the status of all spawned turtles, echo the topic:
    
    ros2 topic echo /turtle_status


## License
This project is for educational purposes and follows standard open-source licensing.

## Acknowledgments
I would like to acknowledge Edouard Renard, the course creator and instructor, for providing an excellent introduction to ROS2 concepts through the Udemy course. While this course served as a solid foundation for my project, the work and development within this repository reflect my own efforts and deepening understanding of ROS2. I hope this project serves as a useful resource for others exploring ROS2, demonstrating both the concepts learned and the practical implementation of those concepts.


