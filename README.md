# Simple Commander Demo

There are 2 simple programs to show how to use [Simple Commander](https://navigation.ros.org/commander_api/index.html).

## Nav2 Test

[nav2_test.py](nav2_test.py) is a very simple example that shows how to set inital pose and use `goToPose()` to navigate the robot.

## Nav2 Waypoints

[nav2_waypoints.py](nav2_waypoints.py) builds on the previous example and uses `followWaypoints()` for more complex navigation.

## Dependencies

Install the dependencies:

    sudo apt update
    sudo apt upgrade
    sudo apt install ros-humble-nav2-simple-commander ros-humble-tf-transformations

## Running

You will need Gazebo running with the turtlebot simulation:

    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

In the next window start the navigator (you will need to be in the correct location with respect to `maps` directory - i.e. in the same dir as this README):

    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/world_slam_toolbox.yaml

Finally, run the navigation files:

    python3 nav2_test.py