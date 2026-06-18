import time
import argparse
import subprocess
import os
from os.path import join

import numpy as np
import rospy
import rospkg
import math

from gazebo_simulation import GazeboSimulation
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan

INIT_POSITION = [-2, 3, 1.57]  # in world frame
GOAL_POSITION = [0, 10]  # relative to the initial position

def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def path_coord_to_gazebo_coord(x, y):
        RADIUS = 0.075
        r_shift = -RADIUS - (30 * RADIUS * 2)
        c_shift = RADIUS + 5

        gazebo_x = x * (RADIUS * 2) + r_shift
        gazebo_y = y * (RADIUS * 2) + c_shift

        return (gazebo_x, gazebo_y)

def get_initial_odom():
    # Wait for the first message to see where the EKF thinks the robot is
    data = rospy.wait_for_message('/odometry/filtered', Odometry)
    return data.pose.pose.position.x, data.pose.pose.position.y

def recover_from_collision():
    print("Collision detected!")

    rospy.set_param("/pure_pursuit_enabled", False)

    try:
        scan_msg = rospy.wait_for_message("/front/scan", LaserScan, timeout=1.0)
        ranges = scan_msg.ranges
    except rospy.ROSException:
        print("No scan detected")
        ranges = None

    cmd_vel_pub = rospy.Publisher("/twist_marker_server/cmd_vel", Twist, queue_size=1)
    recovery_msg = Twist()

    # Check if collision is front or back
    if ranges:
        front_dist = min(ranges[120:600])

        if front_dist < 0.25:
            recovery_msg.linear.x = -0.3
        else:
            recovery_msg.linear.x = 0.3
    else:
        recovery_msg.linear.x = 0.3

    # Back off from obstacle
    start_recovery = rospy.get_time()
    while rospy.get_time() - start_recovery < 1:
        cmd_vel_pub.publish(recovery_msg)
        rospy.sleep(0.05)

    try:
        odom_msg = rospy.wait_for_message("/odometry/filtered", Odometry, timeout=1.0)
        path_msg = rospy.wait_for_message("/A_Star_Planned_Path_Rviz", Path, timeout=1.0)
        
        # Convert quaternion to yaw
        orientation_q = odom_msg.pose.pose.orientation
        current_yaw = get_yaw_from_quaternion(orientation_q)
        
        target_yaw = get_target_yaw(odom_msg.pose.pose, path_msg)

        # Rotate to align with A* path
        align_msg = Twist()
        while True:
            # 1. Calculate raw error
            error = target_yaw - current_yaw
            
            # 2. NORMALIZE the error to [-pi, pi]
            # This ensures the robot always turns the shortest distance
            while error > math.pi: error -= 2.0 * math.pi
            while error < -math.pi: error += 2.0 * math.pi
            
            # 3. Check tolerance
            if abs(error) < 0.15:
                break

            # 4. Determine direction based on normalized error
            align_msg.angular.z = 0.4 if (error > 0) else -0.4
            cmd_vel_pub.publish(align_msg)
            
            # Update current_yaw
            odom_msg = rospy.wait_for_message("/odometry/filtered", Odometry, timeout=0.1)
            current_yaw = get_yaw_from_quaternion(odom_msg.pose.pose.orientation)
            
    except Exception as e:
        print("Alignment failed, resuming anyway: ", e)

    cmd_vel_pub.publish(Twist())
    rospy.set_param("/pure_pursuit_enabled", True)

def get_target_yaw(robot_pose, path_msg):
    # Find the index of the closest point on the path
    min_dist = float('inf')
    closest_idx = 0
    
    rx = robot_pose.position.x
    ry = robot_pose.position.y

    for i, pose in enumerate(path_msg.poses):
        px = pose.pose.position.x
        py = pose.pose.position.y
        dist = math.sqrt((px - rx)**2 + (py - ry)**2)
        if dist < min_dist:
            min_dist = dist
            closest_idx = i

    # Now look 5-10 points ahead of the closest point
    target_idx = min(closest_idx + 10, len(path_msg.poses) - 1)
    target_point = path_msg.poses[target_idx].pose.position

    dx = target_point.x - rx
    dy = target_point.y - ry
    return math.atan2(dy, dx)

def get_yaw_from_quaternion(q):
    """
    Manually convert ROS quaternion (x, y, z, w) to Yaw (Z-axis rotation)
    Works in both Python 2 and 3 without tf library.
    """
    try:
        # 1. Try as ROS Object (q.x, q.y...)
        if hasattr(q, 'x'):
            x, y, z, w = q.x, q.y, q.z, q.w
        # 2. Try as List/Tuple/Array (q[0], q[1]...)
        else:
            x, y, z, w = q[0], q[1], q[2], q[3]
            
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    except Exception as e:
        # Fallback to avoid crashing the thread
        return 0.0

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = 'test BARN navigation challenge')
    parser.add_argument('--world_idx', type=int, default=0)
    parser.add_argument('--gui', action="store_true")
    parser.add_argument('--out', type=str, default="out.txt")
    args = parser.parse_args()
    
    ##########################################################################################
    ## 0. Launch Gazebo Simulation
    ##########################################################################################
    
    # Hokuyo ust10 config
    """ os.environ["JACKAL_LASER"] = "1"
    os.environ["JACKAL_LASER_MODEL"] = "ust10"
    os.environ["JACKAL_LASER_OFFSET"] = "-0.065 0 0.01" """
    
    # Livox MID-360 config
    os.environ["JACKAL_LASER_3D"] = "1"
    os.environ["JACKAL_LASER_3D_MODEL"] = "mid360"
    os.environ["JACKAL_LASER_3D_OFFSET"] = "0.05 0 0.06"

    # RealSense D435 config
    os.environ["DEPTH_CAMERA"] = "1"
    os.environ["DEPTH_CAMERA_OFFSET"] = "0.05 0 0.02"
    
    if args.world_idx < 300:  # static environment from 0-299
        world_name = "BARN/world_%d.world" %(args.world_idx)
        INIT_POSITION = [-2.25, 3, 1.57]  # in world frame
        GOAL_POSITION = [0, 10]  # relative to the initial position
    elif args.world_idx < 360:  # Dynamic environment from 300-359
        world_name = "DynaBARN/world_%d.world" %(args.world_idx - 300)
        INIT_POSITION = [11, 0, 3.14]  # in world frame
        GOAL_POSITION = [-19, 0]  # relative to the initial position
    elif args.world_idx <= 365:
        world_name = "URBAN/world_%d.world" %(args.world_idx)
        INIT_POSITION = [0, -7, 1.57]  # in world frame
        GOAL_POSITION = [2, 20]  # relative to the initial position
    elif args.world_idx <= 372:
        world_name = "TUNNEL/world_%d.world" %(args.world_idx)
        INIT_POSITION = [0, -18, 1.57]  # in world frame
        GOAL_POSITION = [1.2, 10]  # relative to the initial position
    else:
        raise ValueError("World index %d does not exist" %args.world_idx)
    
    print(">>>>>>>>>>>>>>>>>> Loading Gazebo Simulation with %s <<<<<<<<<<<<<<<<<<" %(world_name))   
    rospack = rospkg.RosPack()
    base_path = rospack.get_path('jackal_helper')
    os.environ['GAZEBO_PLUGIN_PATH'] = os.path.join(base_path, "plugins")
    
    launch_file = join(base_path, 'launch', 'gazebo_launch.launch')
    world_name = join(base_path, "worlds", world_name)
    
    gazebo_process = subprocess.Popen([
        'roslaunch',
        launch_file,
        'world_name:=' + world_name,
        'gui:=' + ("true" if args.gui else "false"),
        'pause:=true',
        'x:=' + str(INIT_POSITION[0]),
        'y:=' + str(INIT_POSITION[1]),
        'yaw:=' + str(INIT_POSITION[2])
    ])
    time.sleep(5)  # sleep to wait until the gazebo being created
    
    rospy.init_node('gym', anonymous=True) #, log_level=rospy.FATAL)
    rospy.set_param('/use_sim_time', True)

    # Get initial coordinates of robot's spawn location
    start_x, start_y = get_initial_odom()
    
    # GazeboSimulation provides useful interface to communicate with gazebo  
    gazebo_sim = GazeboSimulation(init_position=INIT_POSITION)
    
    init_coor = (INIT_POSITION[0], INIT_POSITION[1])
    goal_coor = (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1])
    
    pos = gazebo_sim.get_model_state().pose.position
    curr_coor = (pos.x, pos.y)
    collided = True
    
    # check whether the robot is reset, the collision is False
    while compute_distance(init_coor, curr_coor) > 0.1 or collided:
        gazebo_sim.reset() # Reset to the initial position
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        collided = gazebo_sim.get_hard_collision()
        rospy.loginfo(f"Reset: position={curr_coor}, collision={collided}")
        time.sleep(1)
    
    gazebo_sim.unpause()

    # ########################################################################################
    # 1. Launch your navigation stack
    # (Customize this block to add your own navigation stack)
    # ########################################################################################
    
    import rospkg
    rospack = rospkg.RosPack()
    trajectory_planning_path = rospack.get_path('trajectory_planning')
    trajectory_planning_launch_file = join(trajectory_planning_path, 'launch', 'treajectory_planning.launch')
    trajectory_tracking_path = rospack.get_path('trajectory_tracking')
    trajectory_tracking_launch_file = join(trajectory_tracking_path, 'launch', 'treajectory_tracking.launch')
    trajectory_odom_launch_file = join(trajectory_planning_path, 'launch', 'odom.launch')

    launch_file = join(base_path, '..', 'jackal_helper/launch/move_base_jinyu.launch')

    rviz_file = join(base_path, 'launch', 'rviz_launch.launch')

    pointcloud_to_laserscan_file = join(base_path, 'launch', 'pointcloud_to_laserscan.launch')
    bounding_box_file = join(base_path, 'launch', 'bounding_box.launch')

    darknet_path = rospack.get_path('darknet_ros')
    darknet_file = join(darknet_path, 'launch', 'darknet_ros.launch')

    if args.world_idx >= 300 and args.world_idx < 360:
        nav_stack_process = subprocess.Popen([
            'roslaunch',
            trajectory_odom_launch_file,
            launch_file,
            pointcloud_to_laserscan_file,
            bounding_box_file,
            rviz_file,
            darknet_file
        ])
    else:
        nav_stack_process = subprocess.Popen([
            'roslaunch',
            trajectory_odom_launch_file,
            trajectory_planning_launch_file,
            trajectory_tracking_launch_file,
            pointcloud_to_laserscan_file,
            bounding_box_file,
            rviz_file,
            darknet_file
        ])

    # Make sure your navigation stack recives the correct goal position defined in GOAL_POSITION
    import actionlib
    from geometry_msgs.msg import Quaternion, Twist
    from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
    nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = 'odom'
    mb_goal.target_pose.pose.position.x = start_x + GOAL_POSITION[0] # To resolve difference in baselink and odom frames at start of simulation
    mb_goal.target_pose.pose.position.y = start_y + GOAL_POSITION[1]
    mb_goal.target_pose.pose.position.z = 0
    mb_goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

    nav_as.wait_for_server()
    nav_as.send_goal(mb_goal)

    # # 使用话题直接发布目标位置
    # from geometry_msgs.msg import PoseStamped

    # # 创建话题发布者 - 发布到move_base_simple/goal话题
    # goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    # # 创建目标消息
    # def create_goal_msg():
    #     pose_msg = PoseStamped()
    #     pose_msg.header.stamp = rospy.Time.now()
    #     pose_msg.header.frame_id = 'odom'
    #     pose_msg.pose.position.x = GOAL_POSITION[0]
    #     pose_msg.pose.position.y = GOAL_POSITION[1]
    #     pose_msg.pose.position.z = 0.0
    #     pose_msg.pose.orientation.x = 0.0
    #     pose_msg.pose.orientation.y = 0.0
    #     pose_msg.pose.orientation.z = 0.0
    #     pose_msg.pose.orientation.w = 1.0
    #     return pose_msg

    # rate = rospy.Rate(20)  # 20 Hz
    # start_time = rospy.Time.now().to_sec()

    # while not rospy.is_shutdown():
    #     goal_msg = create_goal_msg()
    #     goal_pub.publish(goal_msg)
    #     pos = gazebo_sim.get_model_state().pose.position
    #    # print("Time: %.2f (s), x: %.2f (m), y: %.2f (m)" %(rospy.get_time(), pos.x, pos.y), end="\r")
    #     rate.sleep()

    ##########################################################################################
    ## 2. Start navigation
    ##########################################################################################
    
    curr_time = rospy.get_time()
    pos = gazebo_sim.get_model_state().pose.position
    curr_coor = (pos.x, pos.y)

    
    # check whether the robot started to move
    while compute_distance(init_coor, curr_coor) < 0.1:
        curr_time = rospy.get_time()
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        time.sleep(0.01)
    
    # start navigation, check position, time and collision
    start_time = curr_time
    start_time_cpu = time.time()
    collided = False
    
    while compute_distance(goal_coor, curr_coor) > 1 and curr_time - start_time < 30:
        curr_time = rospy.get_time()
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        print("Time: %.2f (s), x: %.2f (m), y: %.2f (m)" %(curr_time - start_time, *curr_coor), end="\r")

        collided = gazebo_sim.get_hard_collision()
        if collided:
            recover_from_collision()
            continue

        while rospy.get_time() - curr_time < 0.1:
            time.sleep(0.01)

    ##########################################################################################
    ## 3. Report metrics and generate log
    ##########################################################################################
    
    print(">>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<")
    success = False
    if collided:
        status = "collided"
    elif curr_time - start_time >= 30:
        status = "timeout"
    else:
        status = "succeeded"
        success = True
    print("Navigation %s with time %.4f (s)" %(status, curr_time - start_time))
    
    if args.world_idx >= 300:  # DynaBARN environment which does not have a planned path
        path_length = GOAL_POSITION[0] - INIT_POSITION[0]
    else:
        path_file_name = join(base_path, "worlds/BARN/path_files", "path_%d.npy" %args.world_idx)
        path_array = np.load(path_file_name)
        path_array = [path_coord_to_gazebo_coord(*p) for p in path_array]
        path_array = np.insert(path_array, 0, (INIT_POSITION[0], INIT_POSITION[1]), axis=0)
        path_array = np.insert(path_array, len(path_array), (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1]), axis=0)
        path_length = 0
        for p1, p2 in zip(path_array[:-1], path_array[1:]):
            path_length += compute_distance(p1, p2)
    
    # Navigation metric: 1_success *  optimal_time / clip(actual_time, 2 * optimal_time, 8 * optimal_time)
    optimal_time = path_length / 2
    actual_time = curr_time - start_time
    nav_metric = int(success) * optimal_time / np.clip(actual_time, 2 * optimal_time, 8 * optimal_time)
    print("Navigation metric: %.4f" %(nav_metric))
    
    with open(args.out, "a") as f:
        f.write("%d %d %d %d %.4f %.4f\n" %(args.world_idx, success, collided, (curr_time - start_time)>=100, curr_time - start_time, nav_metric))
    
    gazebo_process.terminate()
    gazebo_process.wait()
    nav_stack_process.terminate()
    nav_stack_process.wait()