#!/usr/bin/env python3.8
import rospy
import numpy as np
from nav_msgs.msg import Odometry
import time
import csv
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler
import threading
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


# ░█▀▄░█▀█░▀█▀░█▀█░░░█▀▀░▀█▀░█▀▄░█░█░█▀▀░▀█▀░█░█░█▀▄░█▀▀
# ░█░█░█▀█░░█░░█▀█░░░▀▀█░░█░░█▀▄░█░█░█░░░░█░░█░█░█▀▄░█▀▀
# ░▀▀░░▀░▀░░▀░░▀░▀░░░▀▀▀░░▀░░▀░▀░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀░▀▀▀

# Waypoint Class
class WayPoint:
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

# Test Case Class
class TestCase:
    def __init__(self, case_id, start_index, goal_index):
        self.case_id = case_id
        self.start_index = start_index
        self.goal_index = goal_index
        self.start_time = None
        self.end_time = None
        self.success = False
        self.timeout = None

    def start(self, timeout):
        self.start_time = time.time()
        self.timeout = timeout

    def stop(self, success=False):
        self.end_time = time.time()
        self.success = success

    def get_duration(self):
        if self.start_time and self.end_time:
            return self.end_time - self.start_time
        else:
            return None
        

# ░█▀▀░█░░░█▀█░█▀▄░█▀█░█░░░░░█░█░█▀█░█▀▄░▀█▀░█▀█░█▀▄░█░░░█▀▀░█▀▀
# ░█░█░█░░░█░█░█▀▄░█▀█░█░░░░░▀▄▀░█▀█░█▀▄░░█░░█▀█░█▀▄░█░░░█▀▀░▀▀█
# ░▀▀▀░▀▀▀░▀▀▀░▀▀░░▀░▀░▀▀▀░░░░▀░░▀░▀░▀░▀░▀▀▀░▀░▀░▀▀░░▀▀▀░▀▀▀░▀▀▀

global_goal = None
global_goal_idx = None
current_position = None
threshold = 0.5
current_test_case = None
results = []
lin_velocity_log = []
ang_velocity_log = []
limit_velocity = 0.2  # m/s
margin = 2.0
odom_subscriber = None
goal_subscriber = None
is_succuess_reset_pose = False
path_marker_publisher = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
manual_mode = False

# Logging Color
fc = {
    "black": '\033[1;30m',
    "red": '\033[1;31m',
    "green": '\033[1;32m',
    "yellow": '\033[1;33m',
    "blue": '\033[1;34m',
    "magenta": '\033[1;35m',
    "cyan": '\033[1;36m',
    "white": '\033[1;37m',
    "off": '\033[1;0m',
}

# Initialize Waypoints
waypoints = [
    # WayPoint(79.166397, -70.053866, 0.371950, -0.013301, -0.002974, 0.554374),
    # WayPoint(146.946609, 0.401820, 0.133211, -0.000146, -0.000135, -2.550938),
    WayPoint(-122.757195, 101.487122, 0.189785, -0.000440, -0.000481, -1.709635),   #0
    WayPoint(-185.676590, 7.973891, 0.131324, -0.000056, -0.000101, -1.629169),     #1
    WayPoint(-164.742020, -151.474289, 0.182272, 0.000031, 0.000002, -1.500553),    #2
    WayPoint(-22.505510, -173.799545, 0.182272, 0.000025, -0.000021, -0.720796),    #3
    WayPoint(164.758188, -130.812440, 0.182224, -0.000219, -0.000117, 1.621685),    #4
    WayPoint(107.723560, 31.089726, 0.182336, 0.000134, -0.000030, 1.723253),       #5
    WayPoint(9.231220, 156.007675, 17.163360, -0.293528, 0.044426, 2.774223),       #6
    WayPoint(-140.764633, 175.916199, 11.174964, -0.016145, 0.268510, -1.269022),   #7
    # Obstacle Avoidance
    WayPoint(-151.211853, 115.654060, 0.130703, -0.000089, 0.000112, -0.914802),    #8
    WayPoint(-144.540017, -58.171172, 0.182459, -0.000407, 0.000146, -1.406206),    #9
    # Add more waypoints as needed
]

# Initialize Test Cases
initial_test_cases = [
    TestCase(1, 0, 1),
    TestCase(2, 1, 2),
    TestCase(3, 2, 3),
    TestCase(4, 3, 4),
    TestCase(5, 4, 5),
    TestCase(6, 5, 6),
    TestCase(7, 6, 7),
    TestCase(8, 7, 0),
    TestCase(9, 8, 9),
    # Add more test cases as needed
]


# ░█▀█░█▀█░█▀▀░█▀▄░█▀█░▀█▀░▀█▀░█▀█░█▀█░█▀▀
# ░█░█░█▀▀░█▀▀░█▀▄░█▀█░░█░░░█░░█░█░█░█░▀▀█
# ░▀▀▀░▀░░░▀▀▀░▀░▀░▀░▀░░▀░░▀▀▀░▀▀▀░▀░▀░▀▀▀

# Set Robot Goal
def set_goal(goal_index):
    global global_goal, global_goal_idx, fc
    global_goal = waypoints[goal_index]
    global_goal_idx = goal_index
    rospy.set_param('/global_goal_x', global_goal.x)
    rospy.set_param('/global_goal_y', global_goal.y)
    rospy.set_param('/global_goal_idx', global_goal_idx)
    rospy.loginfo(fc['green']+f"\nGoal Point\t>>\tx={global_goal.x:.2f}, y={global_goal.y:.2f}, index={global_goal_idx}"+fc['off'])
    visualize_waypoints()  # Update waypoint visualization with the new goal

def set_robot_pose(waypoint):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = 'husky'
        model_state.pose.position.x = waypoint.x
        model_state.pose.position.y = waypoint.y
        model_state.pose.position.z = waypoint.z
        quaternion = quaternion_from_euler(waypoint.roll, waypoint.pitch, waypoint.yaw)
        model_state.pose.orientation.x = quaternion[0]
        model_state.pose.orientation.y = quaternion[1]
        model_state.pose.orientation.z = quaternion[2]
        model_state.pose.orientation.w = quaternion[3]
        set_state(model_state)
        rospy.loginfo(fc['green']+f"\nStart Point\t>>\tx={waypoint.x:.2f}, y={waypoint.y:.2f}, z={waypoint.z:.2f}, roll={waypoint.roll:.2f}, pitch={waypoint.pitch:.2f}, yaw={waypoint.yaw:.2f}"+fc['off'])
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def pause_gazebo():
    rospy.wait_for_service('/gazebo/pause_physics')
    try:
        pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        pause_physics()
        rospy.loginfo("Gazebo simulation paused.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def unpause_gazebo():
    rospy.wait_for_service('/gazebo/unpause_physics')
    try:
        unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        unpause_physics()
        rospy.loginfo("Gazebo simulation unpaused.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def visualize_waypoints():
    marker_array = MarkerArray()
    counter = 0
    for idx, waypoint in enumerate(waypoints):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = counter
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = waypoint.x
        marker.pose.position.y = waypoint.y
        marker.pose.position.z = 1.1
        quaternion = quaternion_from_euler(waypoint.roll, waypoint.pitch, waypoint.yaw)
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]
        marker.scale.x = 1.5
        marker.scale.y = 1.5
        marker.scale.z = 1.5
        marker.color.a = 1.0

        # Set color based on whether it is the current goal
        if idx == global_goal_idx:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0

        marker.lifetime = rospy.Duration()
        marker_array.markers.append(marker)
        counter += 1
    path_marker_publisher.publish(marker_array)

def odometry_callback(msg):
    global current_position, global_goal, current_test_case, lin_velocity_log, ang_velocity_log, manual_mode
    current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    lin_velocity_log.append(abs(msg.twist.twist.linear.x))
    ang_velocity_log.append(abs(msg.twist.twist.angular.z))
    if global_goal and current_test_case :
        goal_position = (global_goal.x, global_goal.y)
        distance = np.linalg.norm(np.array(current_position) - np.array(goal_position))
        
        if distance < threshold:
            current_test_case.stop(success=True)
            duration = current_test_case.get_duration()
            rospy.loginfo(fc['cyan']+f"\nGoal reached in {duration:.2f} seconds."+fc['off'])
            
            if duration is not None:
                distance = np.linalg.norm(
                    np.array((waypoints[current_test_case.start_index].x, waypoints[current_test_case.start_index].y)) -
                    np.array((waypoints[current_test_case.goal_index].x, waypoints[current_test_case.goal_index].y))
                )
                average_lin_speed = sum(lin_velocity_log) / len(lin_velocity_log)
                average_ang_speed = sum(ang_velocity_log) / len(ang_velocity_log)
                lin_velocity_log, ang_velocity_log = [], []
                rospy.loginfo(fc['cyan']+f"\nThe Average Speed is {average_lin_speed:.2f} m/s, {average_ang_speed:.2f} rad/s."+fc['off'])
                results.append({
                    'test_case_index': current_test_case.case_id,
                    'start_index': current_test_case.start_index,
                    'goal_index': current_test_case.goal_index,
                    'distance': distance,
                    'driving_time': duration,
                    'average_lin_speed': average_lin_speed,
                    'average_ang_speed': average_ang_speed,
                    'success': current_test_case.success
                })
                
            if manual_mode :
                manual_mode = False
                stop_goal_subscription()
                
            set_next_goal(current_test_case.success)
        
        elif current_test_case.start_time is not None :
            if time.time() - current_test_case.start_time > current_test_case.timeout:
                rospy.logwarn("Timeout reached. Moving to the next test case.")
                current_test_case.stop(success=False)
                duration = current_test_case.get_duration()
                if duration is not None:
                    distance = np.linalg.norm(
                        np.array((waypoints[current_test_case.start_index].x, waypoints[current_test_case.start_index].y)) -
                        np.array((waypoints[current_test_case.goal_index].x, waypoints[current_test_case.goal_index].y))
                    )
                    average_lin_speed = 0.0
                    average_ang_speed = 0.0
                    lin_velocity_log, ang_velocity_log = [], []
                    results.append({
                        'test_case_index': current_test_case.case_id,
                        'start_index': current_test_case.start_index,
                        'goal_index': current_test_case.goal_index,
                        'distance': distance,
                        'driving_time': duration,
                        'average_lin_speed': average_lin_speed,
                        'average_ang_speed': average_ang_speed,
                        'success': current_test_case.success
                    })
                if manual_mode :
                    manual_mode = False
                    stop_goal_subscription()
                set_next_goal(current_test_case.success)

def set_next_goal(success=False):
    global current_test_case, is_succuess_reset_pose, manual_mode
    pause_gazebo()  # Pause Gazebo simulation on goal reached
    
    user_input = input(fc['yellow']+"Enter the test case number to run (or 'q' to quit): "+fc['off'])
    if user_input.lower() == 'q':
        rospy.loginfo(fc['yellow']+"\nTest completed."+fc['off'])
        rospy.signal_shutdown("User requested shutdown.")
        return
    if user_input.lower() == 'm':
        rospy.loginfo("Manual mode activated. Set the goal using RViz.")
        manual_mode = True
        start_goal_subscription()
        return
    try:
        test_case_number = int(user_input)
        current_test_case = next((tc for tc in initial_test_cases if tc.case_id == test_case_number), None)
        if current_test_case is None:
            rospy.logwarn("Invalid test case number. Please try again.")
            set_next_goal(success)
            return
    except ValueError:
        rospy.logwarn("Invalid input. Please enter a valid test case number or 'q' to quit.")
        set_next_goal(success)
        return

    distance = np.linalg.norm(
        np.array((waypoints[current_test_case.start_index].x, waypoints[current_test_case.start_index].y)) -
        np.array((waypoints[current_test_case.goal_index].x, waypoints[current_test_case.goal_index].y))
    )
    timeout = distance / limit_velocity * margin
    rospy.loginfo(fc['green']+f"\n========== Test Case {current_test_case.case_id} =========="+fc['off'])
    if not success or (success and is_succuess_reset_pose):
        set_robot_pose(waypoints[current_test_case.start_index])
    set_goal(current_test_case.goal_index)
    current_test_case.start(timeout=timeout)
    unpause_gazebo()  # Unpause Gazebo simulation after resetting for the next test case

def wait_for_start():
    rospy.loginfo(fc['yellow']+"\nWaiting for keyboard input to start the test. Press 'p' to start..."+fc['off'])
    while True:
        user_input = input(fc['yellow']+"\nPress 'p' to start the test: "+fc['off'])
        if user_input.lower() == 'p':
            rospy.loginfo("Starting the test...")
            break

def start_odometry_subscription():
    global odom_subscriber
    if odom_subscriber is None:
        odom_subscriber = rospy.Subscriber("/odometry_gt", Odometry, odometry_callback)
        rospy.loginfo("Odometry subscription started.")

def stop_odometry_subscription():
    global odom_subscriber
    if odom_subscriber is not None:
        odom_subscriber.unregister()
        odom_subscriber = None
        rospy.loginfo("Odometry subscription stopped.")

# Set Manual Goal
def goal_callback(msg):
    """Callback function to handle goal pose from RViz."""
    global manual_mode
    if manual_mode:
        set_manual_goal(msg)

def start_goal_subscription():
    global goal_subscriber
    if goal_subscriber is None:
        goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
        rospy.loginfo("Subscribed to RViz goal pose.")

def stop_goal_subscription():
    global goal_subscriber
    if odom_subscriber is not None:
        goal_subscriber.unregister()
        goal_subscriber = None
        rospy.loginfo("RViz Goal subscription stopped.")

def set_manual_goal(goal_pose):
    global global_goal, fc
    roll, pitch, yaw = euler_from_quaternion([goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w])
    global_goal = WayPoint(goal_pose.pose.position.x,
                           goal_pose.pose.position.y,
                           goal_pose.pose.position.z,
                           roll, pitch, yaw)
    rospy.set_param('/global_goal_x', global_goal.x)
    rospy.set_param('/global_goal_y', global_goal.y)
    rospy.loginfo(fc['green']+f"\nManual Goal Set\t>>\tx={global_goal.x:.2f}, y={global_goal.y:.2f}"+fc['off'])
    visualize_waypoints()


# ░█▄█░█▀█░▀█▀░█▀█░░░█░░░█▀█░█▀█░█▀█
# ░█░█░█▀█░░█░░█░█░░░█░░░█░█░█░█░█▀▀
# ░▀░▀░▀░▀░▀▀▀░▀░▀░░░▀▀▀░▀▀▀░▀▀▀░▀░░

if __name__ == '__main__':
    # Initialize ROS Node
    rospy.init_node('hanwha_user_test_node')
    # Set the robot to the starting position of the first test case
    set_robot_pose(waypoints[initial_test_cases[0].start_index])
    threading.Thread(target=visualize_waypoints).start()  # Start waypoint visualization in a separate thread
    # Wait for user input to start the test
    wait_for_start()
    # Start Test  
    start_odometry_subscription()
    set_next_goal()
    rospy.spin()