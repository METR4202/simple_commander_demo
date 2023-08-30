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
    sudo apt install ros-humble-nav2-simple-commander ros-humble-tf-transformations python3-transforms3d

### !!! You need to do this step every time after the the turtlebot3 libraries are updated (i.e. after running sudo apt update & supo apt upgrade) !!!
Update the params file for the turtlebot model that you are using (i.e. `waffle-pi`):

    sudo vim /opt/ros/humble/share/turtlebot3_navigation2/param/waffle_pi.yaml

And update line 29 to read:

    robot_model_type: "nav2_amcl::DifferentialMotionModel"


## Running

You will need Gazebo running with the turtlebot simulation:

    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

In the next window start the navigator (you will need to be in the correct location with respect to `maps` directory - i.e. in the same dir as this README):

    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/world_slam_toolbox.yaml

#### !!! If the map doesn't load and you are sure the map location is correct, go back to the previous step and update the `waffle_pi.yaml` !!!!

Finally, run the navigation files:

    python3 nav2_test.py

Or:

    python3 nav2_waypoints.py