import yaml
import os
import copy
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import moveit_msgs
import math
import numpy as np
PI = math.pi

rospy.init_node('produce_demons')
arm = moveit_commander.MoveGroupCommander("manipulator")
arm.allow_replanning(True)
reference_frame = 'base_link'
arm.set_pose_reference_frame('base_link')
arm.set_goal_position_tolerance(0.001)
arm.set_goal_orientation_tolerance(0.001)
arm.set_max_acceleration_scaling_factor(0.3)
arm.set_max_velocity_scaling_factor(0.5)
end_effector_link = arm.get_end_effector_link()

arm.set_joint_value_target([0.4, -1, 0.5, -1.07, -PI/2, 0.1])
arm.go(wait=True)
joint_space = False
if joint_space:
    arm.set_joint_value_target([0.35, -0.89, 1.85, -1.0, 0, -PI * 0.98])
    plan = arm.plan()
    ee_traj_file = open("traj_joint.txt", 'w+')
    for i in range(len(plan.joint_trajectory.points)):
        position = plan.joint_trajectory.points[i].positions
        np.savetxt(ee_traj_file, np.array(position).reshape(1, len(position)), fmt='%.4e')
    ee_traj_file.close()
    arm.execute(plan)
else:
    current_pose = arm.get_current_pose().pose

    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = 0.331958
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.5
    target_pose.pose.orientation.x = -0.105772335665#-0.482974
    target_pose.pose.orientation.y = 0.699471663193#0.517043
    target_pose.pose.orientation.z = 0.10560709424#-0.504953
    target_pose.pose.orientation.w = 0.698855311952#-0.494393

    arm.set_pose_target(target_pose, end_effector_link)
    arm.go()
    waypoints = []
    waypoints.append(target_pose.pose)

    centerA = target_pose.pose.position.y
    centerB = target_pose.pose.position.z
    radius = 0.2
    ee_traj_file = open("ee_traj.txt", 'w+')
    for th in np.arange(0, 3.14, 0.02):
        target_pose.pose.position.y = centerA + radius * math.cos(th)
        target_pose.pose.position.z = centerB + radius * math.sin(th)
        wpose = copy.deepcopy(target_pose.pose)
        waypoints.append(copy.deepcopy(wpose))
        p = [target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
             target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w]
        np.savetxt(ee_traj_file, np.array(p).reshape(1, len(p)), fmt='%.4e')
    ee_traj_file.close()
    fraction = 0.0
    maxtries = 100
    attempts = 0

    arm.set_start_state_to_current_state()

    while fraction < 1.0 and attempts < maxtries:
        (plan, fraction) = arm.compute_cartesian_path(
            waypoints,
            0.01,
            0.0,
            True)
        attempts += 1
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
    if fraction == 1.0:
        rospy.loginfo("Path computed successfully. Moving the arm.")
        # print(plan)
        arm.execute(plan)
        rospy.loginfo("Path execution complete.")
    else:
        rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")
