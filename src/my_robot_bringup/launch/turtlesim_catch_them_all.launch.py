from struct import pack
from launch import LaunchDescription
from launch_ros.actions import Node
from my_turtle_sim_catch import turtle_spawnner

def generate_launch_description():

    id = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"

    )

    turtle_spawnner_node = Node(
        package="my_turtle_sim_catch",
        executable="turtlespawnner",
        parameters=[
            {"spawner_frequency":0.5},
            {"turtle_name_prefix": "my_turtle"}
        ]
    )

    turtle_controller_node = Node(
        package="my_turtle_sim_catch",
        executable="turtlecontroller",
        parameters=[
            {"catch_closest_turtle": True}
        ]
    )

    

    id.add_action(turtlesim_node)
    id.add_action(turtle_spawnner_node)
    id.add_action(turtle_controller_node)
    return id
