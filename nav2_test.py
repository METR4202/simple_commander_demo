import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations as tft

# install dependencies sudo apt install ros-humble-nav2-simple-commander ros-humble-tf-transformations
# this requires gazebo to be running (see programming_with_nav2.md):
# ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# cd ~/workspace/nav2_ws
# ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/world_slam_toolbox.yaml

def main():
    rclpy.init()

    nav = BasicNavigator()

    # set the initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = -2.0
    initial_pose.pose.position.y = -0.5
    initial_pose.pose.position.z = 0.0

    # you can use this to convert Euler to Quat:
    qx, qy, qz, qw = tft.quaternion_from_euler(0, 0, 0)

    initial_pose.pose.orientation.x = qx
    initial_pose.pose.orientation.y = qy
    initial_pose.pose.orientation.z = qz
    initial_pose.pose.orientation.w = qw

    # !!! you only need to run it the first time !!! otherwise the initial position will be set wrongly
    nav.setInitialPose(initial_pose)

    # this waits until the message has been received and executed
    nav.waitUntilNav2Active()

    # you can use the map displayed in RViz to estimate the position (the grid is 1m x 1m)
    goal1 = PoseStamped()
    goal1.header.frame_id = 'map'
    goal1.header.stamp = nav.get_clock().now().to_msg()

    goal1.pose.position.x = 1.5
    goal1.pose.position.y = -1.0
    
    qx, qy, qz, qw = tft.quaternion_from_euler(0, 0, 1.57)
    goal1.pose.orientation.x = qx
    goal1.pose.orientation.y = qy
    goal1.pose.orientation.z = qz
    goal1.pose.orientation.w = qw

    goal2 = PoseStamped()
    goal2.header.frame_id = 'map'
    goal2.header.stamp = nav.get_clock().now().to_msg()

    goal2.pose.position.x = -1.5
    goal2.pose.position.y = 1.0
    
    qx, qy, qz, qw = tft.quaternion_from_euler(0, 0, -1.57)
    goal2.pose.orientation.x = qx
    goal2.pose.orientation.y = qy
    goal2.pose.orientation.z = qz
    goal2.pose.orientation.w = qw

    goals = [goal1, goal2]

    goal_counter = 0
    while True:
        nav.goToPose(goals[goal_counter % 2])
        goal_counter += 1

        while not nav.isTaskComplete():
            pass
            # print(nav.getFeedback())

        print('Nav result: ', nav.getResult())

    rclpy.shutdown()

if __name__ == '__main__':
    main()