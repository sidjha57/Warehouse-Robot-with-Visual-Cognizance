#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

CONTROL_FILE_PATH = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined/scripts/control.txt"
STATUS_FILE_PATH = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined/scripts/status.txt"

client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

def reach_goal(table,table_angle):
    # assign goal coordinates to ebot_base
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()    
    goal.target_pose.pose.position.x = table[0]
    goal.target_pose.pose.position.y = table[1]
    goal.target_pose.pose.orientation.x = table_angle[0]
    goal.target_pose.pose.orientation.y = table_angle[1]
    goal.target_pose.pose.orientation.z = table_angle[2]
    goal.target_pose.pose.orientation.w = table_angle[3]   
    client.send_goal(goal) 
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

# hard coded room points and ebot orientation
table_coord = {
"pantry":[[12.791071,0.804028]],
"pantry_left":[[14.658420,-1.037887]],
"pantry_right":[[11.10,-0.809125]],
"pantry_exit":[[13.083143,-0.262219],[13.024012,0.855195]],
"meeting":[[8.691749,1.311024],[8.710741,2.242054]],
"meeting_drop":[[6.862687,2.495237],[6.957338,2.577140]],
"meeting_pick":[[7.969441,2.495237],[8.064076,2.680834]],
"meeting_exit":[[8.064076,2.680834],[8.630914,1.924502],[8.775154,1.186096]],
"researchM":[[10.603350,1.103348],[10.761787,10.068030],[11.621890,10.153056]],
"researchM_exit":[[14.868692,10.135546]],
"researchS":[[14.868692,10.135546],[11.621890,10.153056]],
"researchS_exit":[[12.578657,11.032706],[10.761787,10.068030],[10.713684,7.028123]],
"store":[[16.075034,1.130147],[24.480119,-2.289210]],
"store_left":[[25.789904,-3.338421]],
"store_mid":[[25.789904,-3.338421]],
"store_right":[[25.789904,-3.338421],[25.761535,-3.467302]], 
"store_exit":[[24.695766,-2.217103],[14.786498,3.713265]],
"conf":[[5.220025,1.026368],[5.249661,-0.028320],[5.589226,-0.633491]],
"conf_exit":[[5.589226,-0.633491],[5.321815,0.113872],[5.139622,0.850968]]
}

table_angles = {
"pantry":[quaternion_from_euler(0,0,-1.145174)],
"pantry_left":[quaternion_from_euler(0,0,-0.215539)],
"pantry_right":[quaternion_from_euler(0,0,-3.136397)],
"pantry_exit":[quaternion_from_euler(0,0,1.622419),quaternion_from_euler(0,0,1.618521)],
"meeting":[quaternion_from_euler(0,0,1.558938),quaternion_from_euler(0,0,3.030832)],
"meeting_drop":[quaternion_from_euler(0,0,3.14),quaternion_from_euler(0,0,0)],
"meeting_pick":[quaternion_from_euler(0,0,-0.006798),quaternion_from_euler(0,0,1.562901)],
"meeting_exit":[quaternion_from_euler(0,0,-1.016319),quaternion_from_euler(0,0,-1.351501),quaternion_from_euler(0,0,-0.022250)],
"researchM":[quaternion_from_euler(0,0,1.590540),quaternion_from_euler(0,0,0.788266),quaternion_from_euler(0,0,-1.578276)],
"researchM_exit":[quaternion_from_euler(0,0,-1.045800)],
"researchS":[quaternion_from_euler(0,0,2.423048),quaternion_from_euler(0,0,-1.578276)],
"researchS_exit":[quaternion_from_euler(0,0,-2.995918),quaternion_from_euler(0,0,-1.959147),quaternion_from_euler(0,0,-1.589767)],
"store":[quaternion_from_euler(0,0,-0.593287),quaternion_from_euler(0,0,-0.563331)],
"store_left":[quaternion_from_euler(0,0,-0.665531)],
"store_mid":[quaternion_from_euler(0,0,-0.665531)],
"store_right":[quaternion_from_euler(0,0,-2.209354),quaternion_from_euler(0,0,-0.665531)],
"store_exit":[quaternion_from_euler(0,0,2.481335),quaternion_from_euler(0,0,-3.140157),quaternion_from_euler(0,0,-1.547272)],
"conf":[quaternion_from_euler(0,0,-1.556558),quaternion_from_euler(0,0,-1.557076),quaternion_from_euler(0,0,-1.557076)],
"conf_exit":[quaternion_from_euler(0,0,1.938747),quaternion_from_euler(0,0,1.901113),quaternion_from_euler(0,0,1.588829)]
}

# Execution maps
room_names = ["Pantry","Meeting Room","Research Lab","Store Room","Conference Room"]

actual_room_names = {
"pantry":room_names[0],"pantry_right":room_names[0],"pantry_left":room_names[0],"pantry_exit":room_names[0],
"meeting":room_names[1],"meeting_drop":room_names[1],"meeting_pick":room_names[1],"meeting_exit":room_names[1],
"researchM":room_names[2],"researchM_exit":room_names[2],"researchS":room_names[2],"researchS_exit":room_names[2],
"store":room_names[3],"store_left":room_names[3],"store_mid":room_names[3],"store_right":room_names[3],"store_exit":room_names[3],
"conf":room_names[4],"conf_exit":room_names[4]
}

dont_print_when_room_reached = ["researchS_exit","researchM_exit","pantry_right","pantry_left","pantry_exit","store_left","store_right","store_mid","store_exit","meeting_drop","meeting_pick","meeting_exit","conf_exit"]

room_map = {"pantry_right":"meeting_drop","meeting_drop":"meeting_pick","meeting_pick":"research","research":"store_left","store_left":"store_mid","store_mid":"conf"}

def main(reach_pt,room_to_reach):
    # to reach certain rooms there may be a few waypoints required
    print(reach_pt+":"+room_to_reach)
    num_of_interm_points = len(table_coord[reach_pt])
    for pt in range(0,num_of_interm_points):
        # print(file_in_ctrl+":"+reach_pt+":"+str(pt))
        goal_reached = reach_goal(table_coord[reach_pt][pt],table_angles[reach_pt][pt])

        # if the endpoint of room is reached pass the control to pnp keeping the room same
        if pt == num_of_interm_points-1 and goal_reached:
            print_decision = "print"
            for room in dont_print_when_room_reached:
                if room == reach_pt:
                    print_decision = "dont_print"
            status_file = open(STATUS_FILE_PATH,"w")
            status_file.writelines("print_line" + "\n")
            status_file.writelines(actual_room_names[reach_pt] + " " + "Reached" + "\n")
            status_file.writelines(print_decision)
            status_file.close()

            control_file = open(CONTROL_FILE_PATH,"w")
            # control_file.writelines("nav"+"\n")
            if reach_pt == room_to_reach:
                control_file.writelines("pnp"+"\n")
            else:
                control_file.writelines("nav"+"\n")
            control_file.writelines(reach_pt + "\n")
            control_file.writelines(room_to_reach)
            control_file.close()

# pantry_right -> meeting_drop
#     pantry_right -> pantry_exit -> meeting -> meeting_drop

# meeting_drop -> research
#     meeting_drop -> meeting_exit -> research

# start -> pantry_right
#     start -> pantry -> pantry_right

if __name__ == '__main__':
    try:
        rospy.init_node('navigate')
        client.wait_for_server()
        while not(rospy.is_shutdown()):
            # check for file control
            control_file = open(CONTROL_FILE_PATH,"r")
            file_in_ctrl = control_file.readline()[:-1]
            start_room_pt = control_file.readline()[:-1]
            room_to_reach = control_file.readline()
            control_file.close()
            if file_in_ctrl == "nav":
                srp_arr = start_room_pt.split("_")
                rtr_arr = room_to_reach.split("_")
                print(srp_arr)
                print(rtr_arr)

                if srp_arr[0] == rtr_arr[0]:
                    reach_pt = room_to_reach
                else:
                    reach_pt = srp_arr[0] + "_exit"

                if len(srp_arr) == 2:
                    if srp_arr[1] == "exit":
                        reach_pt = rtr_arr[0]
                        print("in")

                if srp_arr[0] == "start":
                    reach_pt = rtr_arr[0]

                main(reach_pt,room_to_reach)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")