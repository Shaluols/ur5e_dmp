
import numpy as np
import matplotlib.pyplot as plt
import pydmps
import pydmps.dmp_discrete
import rospy, moveit_commander, moveit_msgs
from geometry_msgs.msg import Pose, PoseStamped
import copy, os

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

target_pose = PoseStamped()
target_pose.header.frame_id = reference_frame
target_pose.header.stamp = rospy.Time.now()
target_pose.pose.position.x = 0.331958
target_pose.pose.position.y = 0.2
target_pose.pose.position.z = 0.5
target_pose.pose.orientation.x = -0.105772335665  # -0.482974
target_pose.pose.orientation.y = 0.699471663193  # 0.517043
target_pose.pose.orientation.z = 0.10560709424  # -0.504953
target_pose.pose.orientation.w = 0.698855311952  # -0.494393

arm.set_pose_target(target_pose, end_effector_link)
arm.go()
cwd = os.path.dirname(os.path.realpath(__file__))
y_des = np.loadtxt(cwd+'/ee_traj.txt').T
y_des -= y_des[:, 0][:, None]
dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=7, n_bfs=500, ay=np.ones(7) * 20.0)
y_ = dmp.imitate_path(y_des=y_des, plot=False)
y_track_normal, _, _ = dmp.rollout(tau=1)

centerA = target_pose.pose.position.y
centerB = target_pose.pose.position.z
initial_pose = target_pose
waypoints = []
for i in range(y_track_normal.shape[0]):
    target_pose.pose.position.y = centerA + y_track_normal[i, 1]
    target_pose.pose.position.z = centerB + y_track_normal[i, 2]
    wpose = copy.deepcopy(target_pose.pose)
    waypoints.append(copy.deepcopy(wpose))

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
    arm.execute(plan)
    rospy.loginfo("Path execution complete.")
else:
    rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

change_goal = False
if change_goal:
    arm.set_pose_target(initial_pose, end_effector_link)
    arm.go()

    dmp.reset_state()
    dmp.goal += np.array([0, -0.2, 0, 0, 0, 0, 0])
    y_new_goal, _, _ = dmp.rollout(tau=1)
    target_pose = initial_pose
    waypoints = []
    for i in range(y_new_goal.shape[0]):
        target_pose.pose.position.y = centerA + y_new_goal[i, 1]
        target_pose.pose.position.z = centerB + y_new_goal[i, 2]
        wpose = copy.deepcopy(target_pose.pose)
        waypoints.append(copy.deepcopy(wpose))

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
        arm.execute(plan)
        rospy.loginfo("Path execution complete.")
    else:
        rospy.loginfo(
            "Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

# plt.figure(1, figsize=(6, 6))
#
# plt.plot(y_track_normal[:, 1], y_track_normal[:, 2], "b", lw=2)
# plt.legend(['Normal'])
# plt.title("DMP system - draw half circle")
# plt.axis("equal")
# plt.xlim([-1, 1])
# plt.ylim([-1, 0.5])
#
# plt.figure(2)
# plt.subplot(3, 1, 1)
# plt.title("DMP system - draw half circle")
# plt.plot(y_track_normal[:, 1:3])
# plt.show()
