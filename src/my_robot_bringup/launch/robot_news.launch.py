from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    id = LaunchDescription()

    robot_names = ["GISKARD", "BB8", "DANEEL", "JANDER", "C3PO"]

    robot_names_station_nodes = []

    for name in robot_names:
        robot_names_station_nodes.append(Node(
            package="my_py_pkg" ,
            executable="robot_news_station",
            name="robot_news_station" + name.lower(),
            parameters=[{"robot_name": name}]
        ))

    smartphone = Node(
        package="my_py_pkg" ,
        executable="smartphone",
        name="smartphone",
    )
    
    for node in robot_names_station_nodes:
        id.add_action(node)
    id.add_action(smartphone)
    return id
