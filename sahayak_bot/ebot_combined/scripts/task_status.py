#!/usr/bin/env python
import rospy

# DropBox1	Conference Room	FPGA board
# DropBox2	Meeting Room	Coke
# DropBox3	Research lab	Glue

# Started Run!
# <room_name Reached
# <object_name> Identified
# <object_name> Picked
# <object_name> Dropped in <dropbox_name>
# Mission Accomplished!

# status.txt
# 	{list/print_line}
# 	{objects_list}
# 	{print/don't print}

rospy.init_node('task_status',anonymous = True)

STATUS_FILE_PATH = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined/scripts/status.txt"

class_id_to_object_name = {'1':"Battery",'2':"Coke Can",'3':"Glue",'4':"Pair of Wheels Package",'5':"eYFI Board",'6':"FPGA board",'7':"Glass",'8':"Adhesive"}

while not rospy.is_shutdown():
	status_file = open(STATUS_FILE_PATH,"r")
	status_mode = status_file.readline()[:-1]
	line2 = status_file.readline()[:-1]
	print_decision = status_file.readline()
	if print_decision == "print":
		if status_mode == "list":
			identified_objects = (line2[:-1]).split(" ")
			for obj in identified_objects:
				if obj != '':
					rospy.loginfo('\033[94m' + class_id_to_object_name[obj] + " " + "Identified" + '\033[0m')
		if status_mode == "print_line":
			rospy.loginfo('\033[94m' + line2 + '\033[0m')
		status_file = open(STATUS_FILE_PATH,"w")
		status_file.writelines(status_mode + "\n")
		status_file.writelines(line2 + "\n")
		status_file.writelines("dont_print")
		status_file.close()

rospy.spin()