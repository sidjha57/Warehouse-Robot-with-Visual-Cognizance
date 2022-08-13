#!/usr/bin/env python
import rospy

# run this file by opening the directory int terminal & -> python reset_text_files.py
# this creates all the necessary text files required for making the script work
# also don't forget to edit the path in the respective scripts for these text files
# 	navigate.py line numbers -> 9
# 	pick_n_place.py line numbers -> 178,179,313 & 314
# 	object_detect.py line numbers -> 91 & 92
# 	pcl.cpp line number ->	39 & 40

# objects.txt
# fpga_board
# -0.123 0.0
# -3.141504 0.007942 -3.141536
# coke_can
# -0.1 0.085
# -2.781598 0.009109 -3.140695
# glue
# -0.15 0.0
# -3.141504 0.007942 -3.141536
# dropbox

rospy.init_node('reset_text_files',anonymous = True)

# update path here as well
CONTROL_FILE_PATH = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined/scripts/control.txt"
POSE_FILE_PATH = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined/scripts/pose.txt"
STATUS_FILE_PATH = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined/scripts/status.txt"

# EDIT for other routes
control_file = open(CONTROL_FILE_PATH,"w")
control_file.writelines("nav"+"\n")
control_file.writelines("start" + "\n")
control_file.writelines("store_mid")
control_file.close()

# EDIT for other sequences
pose_file = open(POSE_FILE_PATH,"w")
pose_file.writelines("battery"+"\n")
pose_file.writelines("vision"+"\n")
pose_file.writelines("\n")
pose_file.writelines("init")
pose_file.close()

# Printing Start of Run on main terminal
status_file = open(STATUS_FILE_PATH,"w")
status_file.writelines("print_line" + "\n")
status_file.writelines("Started Run!" + "\n")
status_file.writelines("print")
status_file.close()

rospy.signal_shutdown("")