#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from object_msgs.msg import ObjectPose

class ArmControl:
	def __init__(self):
		self._planning_group = "ur5_planning_group"
		self._commander = moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander()
		self._scene = moveit_commander.PlanningSceneInterface()
		self._group = moveit_commander.MoveGroupCommander(self._planning_group)
		self._display_trajectory_publisher = rospy.Publisher(
			'/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

		self._exectute_trajectory_client = actionlib.SimpleActionClient(
        	'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
		self._exectute_trajectory_client.wait_for_server()

		self._planning_frame = self._group.get_planning_frame()
		self._eef_link = self._group.get_end_effector_link()
		self._group_names = self._robot.get_group_names()
		self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
		self._curr_state = self._robot.get_current_state()

		# rospy.loginfo(
		# 	'\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
		# rospy.loginfo(
		# 	'\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
		# rospy.loginfo(
		# 	'\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

		# rospy.loginfo('\033[94m' + " >>> ArmControl init done." + '\033[0m')

	def set_end_effector_pose(self, arg_pose):
        #current joint & pose values
		list_joint_values = self._group.get_current_joint_values()
		# rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
		# rospy.loginfo(list_joint_values)

		pose_values = self._group.get_current_pose().pose
		# rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
		# rospy.loginfo(pose_values)
        #current joint & pose values

        #set pose & joint values
        # self._group.set_joint_value_target(arg_list_joint_angles)
		self._group.set_pose_target(arg_pose)
		flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        #set pose & joint values

        #final joint & pose values
		pose_values = self._group.get_current_pose().pose
		# rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
		# rospy.loginfo(pose_values)

		list_joint_values = self._group.get_current_joint_values()
		# rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
		# rospy.loginfo(list_joint_values)
		# final joint & pose values

		if (flag_plan == True):
			rospy.loginfo('\033[94m' + ">>> set_end_effector_pose() Success" + '\033[0m')
		else:
			rospy.logerr('\033[94m' + ">>> set_end_effector_pose() Failed. Solution for Pose not Found." + '\033[0m')
		return flag_plan

	def set_joint_angles(self, arg_list_joint_angles):

		list_joint_values = self._group.get_current_joint_values()
		# rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
		# rospy.loginfo(list_joint_values)

		self._group.set_joint_value_target(arg_list_joint_angles)
		self._group.plan()
		flag_plan = self._group.go(wait=True)

		list_joint_values = self._group.get_current_joint_values()
		# rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
		# rospy.loginfo(list_joint_values)

		pose_values = self._group.get_current_pose().pose
		# rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
		# rospy.loginfo(pose_values)

		# if (flag_plan == True):
		# 	rospy.loginfo('\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
		# else:
		# 	rospy.logerr('\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

		return flag_plan

	def print_joint_angles(self):
		return self._group.get_current_joint_values()
		# print(list_joint_values)
        # rospy.loginfo('\033[94m' + "\nJoint Values: \n\n" +
        #               "ur5_shoulder_pan_joint: {}\n".format(math.degrees(list_joint_values[0])) +
        #               "ur5_shoulder_lift_joint: {}\n".format(math.degrees(list_joint_values[1])) +
        #               "ur5_elbow_joint: {}\n".format(math.degrees(list_joint_values[2])) +
        #               "ur5_wrist_1_joint: {}\n".format(math.degrees(list_joint_values[3])) +
        #               "ur5_wrist_2_joint: {}\n".format(math.degrees(list_joint_values[4])) +
        #               "ur5_wrist_3_joint: {}\n".format(math.degrees(list_joint_values[5])) +
        #               '\033[0m') 

    # Destructor
	def __del__(self):
		moveit_commander.roscpp_shutdown()
		# rospy.loginfo('\033[94m' + "Object of class ArmControl Deleted." + '\033[0m')

class GripperControl:
	def __init__(self):
		self._planning_group = "gripper_planning_group"
		self._commander = moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander()
		self._scene = moveit_commander.PlanningSceneInterface()
		self._group = moveit_commander.MoveGroupCommander(self._planning_group)
		self._display_trajectory_publisher = rospy.Publisher(
			'/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

		self._exectute_trajectory_client = actionlib.SimpleActionClient(
        	'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
		self._exectute_trajectory_client.wait_for_server()

		self._planning_frame = self._group.get_planning_frame()
		self._eef_link = self._group.get_end_effector_link()
		self._group_names = self._robot.get_group_names()
		self._box_name = ''

		# Current State of the Robot is needed to add box to planning scene
		self._curr_state = self._robot.get_current_state()

		# rospy.loginfo(
		# 	'\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
		# rospy.loginfo(
		# 	'\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
		# rospy.loginfo(
		# 	'\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

		# rospy.loginfo('\033[94m' + " >>> GripperControl init done." + '\033[0m')

	def set_joint_angles(self, arg_list_joint_angles):

		list_joint_values = self._group.get_current_joint_values()
		# rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
		# rospy.loginfo(list_joint_values)

		self._group.set_joint_value_target(arg_list_joint_angles)
		self._group.plan()
		flag_plan = self._group.go(wait=True)

		list_joint_values = self._group.get_current_joint_values()
		# rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
		# rospy.loginfo(list_joint_values)

		pose_values = self._group.get_current_pose().pose
		# rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
		# rospy.loginfo(pose_values)

		# if (flag_plan == True):
		# 	rospy.loginfo('\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
		# else:
		# 	rospy.logerr('\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

		return flag_plan

    # Destructor
	def __del__(self):
		moveit_commander.roscpp_shutdown()
		# rospy.loginfo('\033[94m' + "Object of class GripperControl Deleted." + '\033[0m')

# function to return key for any value
def get_key(val,my_dict):
    for key, value in my_dict.items():
         if val == value:
             return key

def core_working(arm_control,gripper_control,current_room,file_in_ctrl):
	PI = 3.141592654
	POSE_FILE_PATH = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined/scripts/pose.txt"
	OBJECTS_FILE_PATH = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined/scripts/objects.txt"
	STATUS_FILE_PATH = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined/scripts/status.txt"

	# init coord msg
	ur5_obj_coord = geometry_msgs.msg.Pose()
	# coord are extracted from pose.txt by this list
	detected_objects_pose = []

	# Reading pose.txt and dont close it yet
	pose_file = open(POSE_FILE_PATH, "r")
	object_to_pick = pose_file.readline()[:-1]
	pose_file_mode = pose_file.readline()[:-1]
	# pixel coord not needed
	pixel_coord = pose_file.readline()[:-1]

	# if pose file isn't updated exit the function
	if object_to_pick == "":
		return

	if pixel_coord == "0 0":
		pose_file.close()
		if current_room == "meeting_pick" or current_room == "pantry_right" or current_room == "store_right" or current_room == "store_mid" or current_room == "store_left":
			interm_jt_ang = vision_pose[	vision_map[	"vision"+"_"+current_room	]	]
			interm_jt_ang[0] = math.radians(-100)
			arm_control.set_joint_angles(	interm_jt_ang	)
			arm_control.set_joint_angles(	interm_jt_ang	)
			arm_control.set_joint_angles(	interm_jt_ang	)

			interm_jt_ang[0] = math.radians(0)	
		arm_control.set_joint_angles(	original_pose	)
		arm_control.set_joint_angles(	original_pose	)
		arm_control.set_joint_angles(	original_pose	)
		arm_control.set_joint_angles(	original_pose	)

		pose_file = open(POSE_FILE_PATH,"w")
		pose_file.writelines(object_to_pick + "\n")
		pose_file.writelines(pose_file_mode + "\n")
		pose_file.writelines("\n")
		pose_file.writelines("init")
		pose_file.close()
		return "nav"

	# EDIT objects.txt for other objects
	# Reading objects.txt
	objects_file = open(OBJECTS_FILE_PATH,"r")
	print(object_to_pick)
	for i in range(0,objects_file_map[object_to_pick]):
		objects_file.readline()
	# spacing from the object to avoid penetration of gripper into object
	allowance_y_z = list(	map(	float,(objects_file.readline()[:-1]).split(" ")	)	)
	# end effector roll pitch yaw
	gripper_angles = list(	map(	float,(objects_file.readline()[:-1]).split(" ")	)	)
	objects_file.close()

	# setting the ee gripper angles obtained from objects.txt
	gripper_angles_quat = quaternion_from_euler(gripper_angles[0],gripper_angles[1],gripper_angles[2])
	ur5_obj_coord.orientation.x = gripper_angles_quat[0]
	ur5_obj_coord.orientation.y = gripper_angles_quat[1]
	ur5_obj_coord.orientation.z = gripper_angles_quat[2]
	ur5_obj_coord.orientation.w = gripper_angles_quat[3]	

	if pose_file_mode == "vision":
		print(file_in_ctrl + ":" + object_to_pick + ":" + pose_file_mode)
		if current_room == "meeting_pick" or current_room == "pantry_right" or current_room == "store_right" or current_room == "store_mid" or current_room == "store_left":
			interm_jt_ang = original_pose
			interm_jt_ang[0] = math.radians(-100)
			arm_control.set_joint_angles(	interm_jt_ang	)
			arm_control.set_joint_angles(	interm_jt_ang	)
			arm_control.set_joint_angles(	interm_jt_ang	)

			interm_jt_ang[0] = math.radians(-28)

			interm_jt_ang = vision_pose[	vision_map[	pose_file_mode+"_"+current_room	]	]
			interm_jt_ang[0] = math.radians(-100)
			arm_control.set_joint_angles(	interm_jt_ang	)
			arm_control.set_joint_angles(	interm_jt_ang	)
			arm_control.set_joint_angles(	interm_jt_ang	)

			interm_jt_ang[0] = math.radians(0)
			arm_control.set_joint_angles(	interm_jt_ang	)
			arm_control.set_joint_angles(	interm_jt_ang	)
			arm_control.set_joint_angles(	interm_jt_ang	)

		# moving the ur5 to the vision pose
		arm_control.set_joint_angles(	vision_pose[	vision_map[	pose_file_mode+"_"+current_room	]	]	)
		arm_control.set_joint_angles(	vision_pose[	vision_map[	pose_file_mode+"_"+current_room	]	]	)
		arm_control.set_joint_angles(	vision_pose[	vision_map[	pose_file_mode+"_"+current_room	]	]	)
		arm_control.set_joint_angles(	vision_pose[	vision_map[	pose_file_mode+"_"+current_room	]	]	)

		# close the pose.txt file here as all the data is extracted for current mode
		pose_file.close()

		# add gripper angles to last line of pose.txt
		pose_file = open(POSE_FILE_PATH,"w")
		pose_file.writelines(object_to_pick + "\n")
		pose_file.writelines(pose_file_mode + "\n")
		pose_file.writelines("\n")
		pose_file.writelines("init" + "\n")
		# this line enters the file only when pnp runs and is calculated wrt world horizontal
		pose_file.writelines(str(PI - gripper_angles[0]*(-1)) + " " + str(allowance_y_z[0]) + " " + str(allowance_y_z[1]) + " " + str(PI - gripper_angles[3]*(-1)))
		pose_file.close()

	else:
		# read the coord wrt ebot_base from line4
		current_coord = pose_file.readline()[:-1]
		print(file_in_ctrl + ":" + object_to_pick + ":" + pose_file_mode + ":"+current_coord)
		# store the coord in a list
		detected_objects_pose = list(	map(	float,current_coord.split(" ")	)	)
		# roll angle is needed to calculate safe X dist from object to keep camera pointed at the object
		pose_file_line5 = pose_file.readline()
		roll_allowance_arr = list(	map(	float,pose_file_line5.split(" ")	)	)	
		# close the pose.txt file here as all the data is extracted for current mode
		pose_file.close()

		camera_dist_from_ee_interm = 0.05

		if pose_file_mode == "interm":
			ur5_obj_coord.position.x = detected_objects_pose[0] + camera_dist_from_ee_interm
			ur5_obj_coord.position.y = detected_objects_pose[1]
			ur5_obj_coord.position.z = table_height_map[object_to_pick] + allowance_y_z[1]

			# moving the ur5 ee to the given mode coord
			arm_control.set_end_effector_pose(	ur5_obj_coord	)
			arm_control.set_end_effector_pose(	ur5_obj_coord	)
			arm_control.set_end_effector_pose(	ur5_obj_coord	)
			arm_control.set_end_effector_pose(	ur5_obj_coord	)

		if pose_file_mode == "final" or pose_file_mode == "pnp_objd":
			ur5_obj_coord.position.x = detected_objects_pose[0]		
			ur5_obj_coord.position.y = detected_objects_pose[1]
			ur5_obj_coord.position.z = table_height_map[object_to_pick] + z_lift_for_picking[object_to_pick]

			arm_control.set_end_effector_pose(	ur5_obj_coord	)
			arm_control.set_end_effector_pose(	ur5_obj_coord	)
			arm_control.set_end_effector_pose(	ur5_obj_coord	)
			arm_control.set_end_effector_pose(	ur5_obj_coord	)

		if pose_file_mode == "grasp":
			ur5_obj_coord.position.x = detected_objects_pose[0] + camera_dist_from_ee_grasp[object_to_pick]
			ur5_obj_coord.position.y = detected_objects_pose[1]
			ur5_obj_coord.position.z = table_height_map[object_to_pick] + z_lift_for_picking[object_to_pick] + (-1)*z_decrement_for_picking[object_to_pick]

			arm_control.set_end_effector_pose(	ur5_obj_coord	)
			arm_control.set_end_effector_pose(	ur5_obj_coord	)
			arm_control.set_end_effector_pose(	ur5_obj_coord	)
			arm_control.set_end_effector_pose(	ur5_obj_coord	)

			gripper_control.set_joint_angles(	gripper_angle_map[object_to_pick]	)
			gripper_control.set_joint_angles(	gripper_angle_map[object_to_pick]	)
			gripper_control.set_joint_angles(	gripper_angle_map[object_to_pick]	)

			# object picked print on main terminal
			status_file = open(STATUS_FILE_PATH,"w")
			status_file.writelines("print_line" + "\n")
			status_file.writelines(actual_object_names[object_to_pick] + " " + "Picked" + "\n")
			status_file.writelines("print")
			status_file.close()			

			arm_control.set_joint_angles(	vision_pose[	vision_map[	"vision"+"_"+current_room	]	]	)
			arm_control.set_joint_angles(	vision_pose[	vision_map[	"vision"+"_"+current_room	]	]	)
			arm_control.set_joint_angles(	vision_pose[	vision_map[	"vision"+"_"+current_room	]	]	)
			arm_control.set_joint_angles(	vision_pose[	vision_map[	"vision"+"_"+current_room	]	]	)

			if object_to_pick == "glue":
				interm_jt_ang = vision_pose[	vision_map[	"vision"+"_"+current_room	]	]
				interm_jt_ang[0] = math.radians(-100)
				arm_control.set_joint_angles(	interm_jt_ang	)
				arm_control.set_joint_angles(	interm_jt_ang	)
				arm_control.set_joint_angles(	interm_jt_ang	)

				interm_jt_ang[0] = math.radians(0)

			arm_control.set_joint_angles(	original_pose	)
			arm_control.set_joint_angles(	original_pose	)
			arm_control.set_joint_angles(	original_pose	)
			arm_control.set_joint_angles(	original_pose	)

			# change mode to drop
			pose_file = open(POSE_FILE_PATH,"w")
			pose_file.writelines(object_to_pick + "\n")
			pose_file.writelines("drop"+"\n")
			pose_file.writelines("\n")
			pose_file.writelines(current_coord + "\n")
			pose_file.writelines(pose_file_line5)
			pose_file.close()

		if pose_file_mode == "drop":
			arm_control.set_joint_angles(	vision_pose[	vision_map[	"vision"+"_"+current_room	]	]	)
			arm_control.set_joint_angles(	vision_pose[	vision_map[	"vision"+"_"+current_room	]	]	)
			arm_control.set_joint_angles(	vision_pose[	vision_map[	"vision"+"_"+current_room	]	]	)
			arm_control.set_joint_angles(	vision_pose[	vision_map[	"vision"+"_"+current_room	]	]	)

			arm_control.set_joint_angles(	dropbox_poses[current_room]	)
			arm_control.set_joint_angles(	dropbox_poses[current_room]	)
			arm_control.set_joint_angles(	dropbox_poses[current_room]	)
			arm_control.set_joint_angles(	dropbox_poses[current_room]	)

			gripper_control.set_joint_angles(	gripper_drop	)
			gripper_control.set_joint_angles(	gripper_drop	)
			gripper_control.set_joint_angles(	gripper_drop	)
			gripper_control.set_joint_angles(	gripper_drop	)

			# object dropped print on main terminal
			status_file = open(STATUS_FILE_PATH,"w")
			status_file.writelines("print_line" + "\n")
			if current_room == "meeting_pick":
				status_file.writelines("Mission Accomplished!" + "\n")
			else:
				status_file.writelines(actual_object_names[object_to_pick] + " " + "Dropped in" + " " + dropbox_name_map[current_room] + "\n")
			status_file.writelines("print")
			status_file.close()

			arm_control.set_joint_angles(	vision_pose[	vision_map[	"vision"+"_"+current_room	]	]	)
			arm_control.set_joint_angles(	vision_pose[	vision_map[	"vision"+"_"+current_room	]	]	)
			arm_control.set_joint_angles(	vision_pose[	vision_map[	"vision"+"_"+current_room	]	]	)
			arm_control.set_joint_angles(	vision_pose[	vision_map[	"vision"+"_"+current_room	]	]	)

			arm_control.set_joint_angles(	original_pose	)
			arm_control.set_joint_angles(	original_pose	)
			arm_control.set_joint_angles(	original_pose	)
			arm_control.set_joint_angles(	original_pose	)

			# change mode to vision
			pose_file = open(POSE_FILE_PATH,"w")
			pose_file.writelines(objects_map[object_to_pick] + "\n")
			pose_file.writelines("vision"+"\n")
			pose_file.writelines("\n")
			pose_file.writelines("init" + "\n")
			pose_file.writelines(pose_file_line5)
			pose_file.close()		

	return pose_file_mode

# battery meeting_pick -> table_height_map 0.9; z_lift_for_picking 0.11; z_decrement_for_picking 0.0; gripper_angle_map math.radians(10)

# Execution maps
scripts_map = {"nav":"pnp","pnp":"objd","objd":"pcl","pcl":"nav"}
gripper_angle_map = {"battery":[math.radians(24)],"coke_can":[math.radians(14.5)],"glue":[math.radians(8)],"pair_of_wheels_package":[math.radians(12)],"eyfi_board":[math.radians(12)],"fpga_board":[math.radians(10)],"glass":[math.radians(12)],"adhesive":[math.radians(12)]}
vision_map = {"vision_store_left":1,"vision_store_mid":1,"vision_store_right":1,"vision_pantry_left":2,"vision_pantry_right":2,"vision_meeting_drop":0,"vision_meeting_pick":1,"vision_researchS":1,"vision_researchM":1,"vision_conf":1}
actual_object_names = {"coke_can":"Coke Can","glue":"Glue","fpga_board":"FPGA Board","battery":"Battery","adhesive":"Adhesive"}
dropbox_name_map = {"meeting_drop":"DropBox2","conf":"DropBox1","researchS":"DropBox3","researchM":"DropBox3"}
objects_map = {"battery":"adhesive","adhesive":"done"}
# EDIT for other objects
objects_file_map = {"battery":1,"coke_can":4,"glue":7,"pair_of_wheels_package":10,"eyfi_board":13,"fpga_board":16,"glass":19,"adhesive":22,"done":25}
camera_dist_from_ee_grasp = {"battery":0.08,"coke_can":0.08,"glue":0.009,"pair_of_wheels_package":0.08,"eyfi_board":0.08,"fpga_board":0.08,"glass":0.08,"adhesive":0.08}
z_decrement_for_picking = {"battery":0,"coke_can":0,"glue":0.0,"pair_of_wheels_package":0,"eyfi_board":0,"fpga_board":0.04,"glass":0,"adhesive":0}
z_lift_for_picking = {"battery":0.08,"coke_can":0.125,"glue":0.08,"pair_of_wheels_package":0.04,"eyfi_board":0.125,"fpga_board":0.125,"glass":0.125,"adhesive":0.125}
table_height_map = {"battery":1.1,"coke_can":0.9,"glue":0.9,"pair_of_wheels_package":1.1,"eyfi_board":1.1,"fpga_board":1.1,"glass":0.9,"adhesive":1.1}
# EDIT for other objects
# room_map = {"pantry_left":"meeting_drop","pantry_right":"meeting_drop","meeting_drop":"meeting_pick","meeting_pick":"research","research":"store_left","store_left":"conf","store_mid":"conf","store_right":"conf"}
room_map = {"store_mid":"researchS","researchS":"pantry_right","pantry_right":"conf","store_left":"conf","conf":"meeting_pick"}

# room_map = {"meeting_bonus":"meeting_drop","meeting_drop":"meeting_bonus","meeting_bonus":"researchM","researchM":"meeting_bonus","meeting_bonus":"conf","conf":"pantry_right"}


# hard coded values for clear vision for object detection
vision_pose = [ [math.radians(-7),math.radians(-25),math.radians(-72),math.radians(-69),math.radians(-82),math.radians(175)],
				[math.radians(0),math.radians(-44),math.radians(-35),math.radians(-88),math.radians(-87),math.radians(180)],
				[math.radians(2),math.radians(-36),math.radians(-34),math.radians(-97),math.radians(-88),math.radians(181)],
				[math.radians(3),math.radians(-40),math.radians(-22),math.radians(-106),math.radians(-89),math.radians(182)],
				[math.radians(8),math.radians(-4),math.radians(-4),math.radians(-22),math.radians(8),math.radians(-1)]	]

# orignal position
original_pose = [math.radians(-28),math.radians(-1),math.radians(30),math.radians(-58),math.radians(-34),math.radians(0)]

# dropbox poses
dropbox_poses = {"meeting_drop":[math.radians(82),math.radians(21),math.radians(-64),math.radians(12),math.radians(89),math.radians(-3)],
"conf":[math.radians(0),math.radians(0),math.radians(-36),math.radians(-179),math.radians(-91),math.radians(182)],
"researchS":[math.radians(0),math.radians(0),math.radians(-36),math.radians(-179),math.radians(-91),math.radians(182)],
"researchM":[math.radians(0),math.radians(0),math.radians(-36),math.radians(-179),math.radians(-91),math.radians(182)]}

# setting the gripper 0 for dropping objects
gripper_drop = [0]

def test_manual_coord(xyz,rpy,joint_angles,mode,arm_control):
	if mode == "ee":
		ur5_obj_coord = geometry_msgs.msg.Pose()
		ur5_obj_coord.position.x = xyz[0]
		ur5_obj_coord.position.y = xyz[1]
		ur5_obj_coord.position.z = xyz[2]
		gripper_angles_quat = quaternion_from_euler(rpy[0],rpy[1],rpy[2])
		ur5_obj_coord.orientation.x = gripper_angles_quat[0]
		ur5_obj_coord.orientation.y = gripper_angles_quat[1]
		ur5_obj_coord.orientation.z = gripper_angles_quat[2]
		ur5_obj_coord.orientation.w = gripper_angles_quat[3]
		arm_control.set_end_effector_pose(ur5_obj_coord)

	if mode == "joint":
		arm_control.set_joint_angles(joint_angles)
		arm_control.set_joint_angles(joint_angles)
		arm_control.set_joint_angles(joint_angles)

	if mode == "print":
		jt_ang = arm_control.print_joint_angles()
		print(jt_ang)
		print(math.radians(jt_ang[5]))

def main():
	rospy.init_node('pick_n_place', anonymous=True)

	arm_control = ArmControl()
	gripper_control = GripperControl()

	xyz = [0.026766,0.109193,1.281104]
	rpy = [-2.188045,0.1,2.299467]
	joint_angles = original_pose

	mode = "joint"

	test_manual_coord(xyz,rpy,joint_angles,mode,arm_control)

	while not rospy.is_shutdown():
		CONTROL_FILE_PATH = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined/scripts/control.txt"
		POSE_FILE_PATH = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined/scripts/pose.txt"
		# Read control.txt
		control_file = open(CONTROL_FILE_PATH,"r")
		file_in_ctrl = control_file.readline()[:-1]
		start_room_pt = control_file.readline()[:-1]
		current_room = control_file.readline()
		control_file.close()

		if file_in_ctrl == "pnp":
			pose_file_mode = core_working(arm_control,gripper_control,current_room,file_in_ctrl)
			# Writing to control.txt to change control script
			control_file = open(CONTROL_FILE_PATH,"w")
			# upon grasping the object the ebot needs to be navigated to dropping location
			if pose_file_mode == "grasp" or pose_file_mode == "drop":
				control_file.writelines("nav"+"\n")
			# else if no object detected move to different point in same room
			elif pose_file_mode == "nav":
				control_file.writelines("nav"+"\n")
			# else follow usual loop
			else:
				control_file.writelines(scripts_map[file_in_ctrl]+"\n")
			control_file.writelines(start_room_pt + "\n")
			if pose_file_mode == "grasp" or pose_file_mode == "drop":
				control_file.writelines(room_map[current_room])
			else:	
				control_file.writelines(current_room)
			control_file.close()

			# end of run
			# if current_room == "meeting_pick":
			# 	rospy.signal_shutdown("")			

	rospy.spin()
if __name__ == '__main__':
	main()