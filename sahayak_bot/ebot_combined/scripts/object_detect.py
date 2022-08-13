#!/usr/bin/env python
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math

from object_msgs.msg import ObjectPose

from sensor_msgs.msg import PointCloud2

import os
import cv2
import numpy as np
try:
    import tensorflow as tf
except ImportError:
    print("unable to import TensorFlow. Is it installed?")
    print("  sudo apt install python-pip")
    print("  sudo pip install tensorflow")
    sys.exit(1)

# ROS related imports
from std_msgs.msg import String , Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Object detection module imports
import object_detection
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# SET FRACTION OF GPU YOU WANT TO USE HERE
GPU_FRACTION = 0.4

# EDIT this to your own path
top_level_dir = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined"

######### Set model here ############
MODEL_NAME =  'ssd_mobilenet_v1_coco_11_06_2017'
# By default models are stored in data/models/
MODEL_PATH = top_level_dir + "/data/models/" + MODEL_NAME
# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_PATH + '/frozen_inference_graph.pb'
######### Set the label map file here ###########
LABEL_NAME = 'object_detection.pbtxt'
# By default label maps are stored in data/labels/
PATH_TO_LABELS = top_level_dir + "/data/labels/" + LABEL_NAME
######### Set the number of classes here #########
NUM_CLASSES = 9

detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.compat.v1.GraphDef()
    with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`,
# we know that this corresponds to `airplane`.  Here we use internal utility functions,
# but anything that returns a dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Setting the GPU options to use fraction of gpu that has been set
config = tf.compat.v1.ConfigProto()
config.gpu_options.per_process_gpu_memory_fraction = GPU_FRACTION

# Detected Objects Info
# EDIT for other objects
detected_objects = {'1':[[],[]],'2':[[],[]],'3':[[],[]],'4':[[],[]],'5':[[],[]],'6':[[],[]],'7':[[],[]],'8':[[],[]],'9':[[],[]]}
detection_script_run_twice = 0
# variable used to check if the object to pick is in the captured image
check_if_required_object_in_pic = False

# Execution maps
vision_map = {"store_left":[0.030528,0.108197,1.281109],
"store_mid":[0.030528,0.108197,1.281109],
"store_right":[0.030528,0.108197,1.281109],
"pantry_left":[0.108824,0.112535,1.275647],
"pantry_right":[0.030528,0.108197,1.281109],
"meeting_pick":[0.030528,0.108197,1.281109],
"conf":[0.030528,0.108197,1.281109],
"researchM":[0.030528,0.108197,1.281109],
"researchS":[0.030528,0.108197,1.281109]}
scripts_map = {"nav":"pnp","pnp":"objd","objd":"pcl","pcl":"nav"}
# EDIT for other objects
label_map = {"battery":"1","coke_can":"2","glue":"3","pair_of_wheels_package":"4","eyfi_board":"5","fpga_board":"6","glass":"7","adhesive":"8","done":"12"}

# txt file paths
CONTROL_FILE_PATH = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined/scripts/control.txt"
POSE_FILE_PATH = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined/scripts/pose.txt"
STATUS_FILE_PATH = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined/scripts/status.txt"

class Detector:
    def __init__(self):
        self.bridge = CvBridge()
        self.sess = tf.compat.v1.Session(graph=detection_graph,config=config)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, self.image_cb, queue_size=1, buff_size=2**24)
        # subscribing to sensor_msgs/Image for debugging
        self.image_pub = rospy.Publisher("sensor_msgs/Image",Image, queue_size=1)

    def run_tf_session(self,data,object_to_pick,pose_file_mode,file_in_ctrl,detection_script_run_twice):
        global vision_map,scripts_map,label_map
        global check_if_required_object_in_pic 

        objArray = Detection2DArray()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        image=cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)

        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        image_np = np.asarray(image)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        scores = detection_graph.get_tensor_by_name('detection_scores:0')
        classes = detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')

        (boxes, scores, classes, num_detections) = self.sess.run([boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

        objects=vis_util.visualize_boxes_and_labels_on_image_array(
            image,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=2)

        objArray.detections =[]
        objArray.header=data.header
        object_count=1

        for i in range(len(objects)):
            object_count+=1
            # if the class id of the object matches the id of object to pick
            # then the required object is in the image
            if object_to_pick != '':
                if str(objects[i][0]) == label_map[object_to_pick]:
                    check_if_required_object_in_pic = True
            objArray.detections.append(self.object_predict(objects[i],data.header,image_np,cv_image))

        img=cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)      
        
        # window popup on detected object for 0.2s
        if pose_file_mode == "vision" and file_in_ctrl == "objd" and detection_script_run_twice == 1:
            cv2.imshow('',img)
            cv2.waitKey(1000)  
            cv2.destroyAllWindows()

        image_out = Image()
        try:
            image_out = self.bridge.cv2_to_imgmsg(img,"bgr8")
        except CvBridgeError as e:
            print(e)
        image_out.header = data.header
        # publishing image to sensor_msgs/Image for debugging
        self.image_pub.publish(image_out)

    def image_cb(self, data):
        global vision_map,scripts_map,label_map,objects_map
        global CONTROL_FILE_PATH,POSE_FILE_PATH
        global check_if_required_object_in_pic
        global detected_objects
        global identified_objects_print_arr
        global detection_script_run_twice

        # reading control
        control_file = open(CONTROL_FILE_PATH,"r")
        file_in_ctrl = control_file.readline()[:-1]
        start_room_pt = control_file.readline()[:-1]
        current_room = control_file.readline()
        control_file.close()

        # updating the mode of file
        pose_file = open(POSE_FILE_PATH, "r")
        object_to_pick = pose_file.readline()[:-1]
        pose_file_mode = pose_file.readline()[:-1]
        pose_file_line3 = pose_file.readline()[:-1]
        current_coord = pose_file.readline()[:-1]
        pose_file_line5 = pose_file.readline()
        pose_file.close()

        # self.run_tf_session(data,object_to_pick,pose_file_mode,file_in_ctrl,detection_script_run_twice)
        if file_in_ctrl == "objd" and object_to_pick != "":
            print(file_in_ctrl + ":" + object_to_pick + ":" + pose_file_mode + ":"+current_coord)
            print("1:"+current_coord)
            self.run_tf_session(data,object_to_pick,pose_file_mode,file_in_ctrl,detection_script_run_twice)
            # measure of deflection of object from center pt
            err_y = 0
            tolerance_for_misalignment = 2
            # writing detected coordinates to pose file
            pose_file = open(POSE_FILE_PATH, "w")  
            pose_file.writelines(object_to_pick + "\n")
            pose_file.writelines(pose_file_mode + "\n")
            # update line3 working line to store pixel coord
            if check_if_required_object_in_pic :
                err_y = 320 - detected_objects[label_map[object_to_pick]][0][0]
                print(str(detected_objects[label_map[object_to_pick]][0][0]) + " " + str(detected_objects[label_map[object_to_pick]][0][1]) + " " + str(detected_objects[label_map[object_to_pick]][1][0]))
                pose_file.writelines(    str(detected_objects[label_map[object_to_pick]][0][0]) + " " + str(detected_objects[label_map[object_to_pick]][0][1]) + " " + str(detected_objects[label_map[object_to_pick]][1][0]) + " " + str(detected_objects[label_map[object_to_pick]][1][1]) + "\n"   )
            else:
                pose_file.writelines( "0 0" + "\n" )
            # update line4 working line to store world coord
            # if ee is at vision pose then insert the current ee xyz rpy values at line4
            if current_coord == "init":
                pose_file.writelines(   str(vision_map[current_room][0]) + " " + str(vision_map[current_room][1]) + " " + str(vision_map[current_room][2]) + "\n"  )
            # if ee is at pnp_objd mode then insert the corrected Y values keeping X & Z same
            elif pose_file_mode == "pnp_objd":
                print("2:"+current_coord)
                update_current_coord = list(    map(    float,current_coord.split(" ")  )   )
                if err_y < 0:
                    update_current_coord[1] -= 0.001
                if err_y > 0:
                    update_current_coord[1] += 0.001
                pose_file.writelines(   str(update_current_coord[0]) + " " + str(update_current_coord[1]) + " " + str(update_current_coord[2]) + "\n")
            # else keep line4 unchanged
            else:
                pose_file.writelines(current_coord + "\n")

            pose_file.writelines(pose_file_line5)
            pose_file.close()             

            detection_script_run_twice+=1           

            # detecting objects twice
            if detection_script_run_twice == 2:
                check_if_required_object_in_pic = False
                # print on main terminal
                if pose_file_mode == "vision":
                    status_file = open(STATUS_FILE_PATH,"w")
                    status_file.writelines("list" + "\n")
                    print_data = ""
                    for class_id in range(1,NUM_CLASSES):
                        if detected_objects[str(class_id)][0]:
                            print_data+= (str(class_id) + " ")
                            detected_objects[str(class_id)][0] = []
                    status_file.writelines(print_data + "\n")
                    status_file.writelines("print")
                    status_file.close()   

                if current_room == "meeting_pick":
                    rospy.signal_shutdown("")
                    
                # passing control to pcl 
                control_file = open(CONTROL_FILE_PATH,"w")
                # pass control to pnp if error is not in tolerance range and mode is pnp_objd
                if pose_file_mode == "pnp_objd" and not(abs(err_y) <= tolerance_for_misalignment):
                    control_file.writelines("pnp" + "\n")
                    print(err_y)
                # else pass control to pcl
                else:
                    control_file.writelines(scripts_map[file_in_ctrl] + "\n")
                control_file.writelines(start_room_pt + "\n")
                control_file.writelines(current_room)
                control_file.close()
                detection_script_run_twice = 0

    def object_predict(self,object_data, header, image_np,image):
    	global detected_objects

        image_height,image_width,channels = image.shape
        obj=Detection2D()
        obj_hypothesis= ObjectHypothesisWithPose()

        object_id=object_data[0]
        object_score=object_data[1]
        dimensions=object_data[2]

        obj.header=header
        obj_hypothesis.id = object_id
        obj_hypothesis.score = object_score
        obj.results.append(obj_hypothesis)
        obj.bbox.size_y = int((dimensions[2]-dimensions[0])*image_height)
        obj.bbox.size_x = int((dimensions[3]-dimensions[1] )*image_width)
        obj.bbox.center.x = int((dimensions[1] + dimensions[3])*image_width/2)
        obj.bbox.center.y = int((dimensions[0] + dimensions[2])*image_height/2)

        # Data Extraction
        center_pt = []
        box_size = []
        center_pt.append(obj.bbox.center.x)
        center_pt.append(obj.bbox.center.y)

        box_size.append(obj.bbox.size_x)
        box_size.append(obj.bbox.size_y)
            
        detected_objects[str(object_id)][0] = center_pt
        detected_objects[str(object_id)][1] = box_size
        return obj

def main(args):

    # In this node objects are detected and their(center of bounding box) pixels values are stored in pose.txt,
    # these pixel values are available to pcl.cpp through the sam

    # This same file is again used for correcting the y position of the detected objects through 
    # 3 intermediate steps.

    # pcl.cpp passes the 3d point obtained from the pixel values to pick_n_place.py
    #   the 3d point obtained in terms of camera link is converted wrt to base_link in the same cpp node

    # pick_n_place.py moves the ur5

    # navigate.py moves the ebot to given location

    # pick_n_place.py works in conjuncton with object_detect.py to correct y values prevailing due to errors in yaw angle of ebot

    # combined.launch launches all nodes simultaneously and execution of checkpoints is handled as per the control
    #   control.txt file is used for the purpose of changing the execution control so things happen serially
    #   following are the keywords for 1st line of the file
    #       nav -> navigate.py
    #       pnp -> pick_n_place.py
    #       objd -> object_detect.py
    #       pcl -> pcl.cpp

    #   following are the keywords for 2nd line of the file
    #       store -> store room
    #       pantry_left -> pantry room left table
    #       pantry_right -> pantry room right table
    #       conf -> conference room
    #       meeting_drop -> meeting room dropbox location
    #       meeting_pick -> meeting room pickup location    
    #       research -> research room

    #   at any given time the structure of the file is as follows:
    #       {script_to_run}
    #       {room_to_reach/current_room}

    # We made use of text file for transfering messages between our scripts 
    # pose.txt is used to give commands to pick_n_place.py to move the ur5 arm
    # this same file is also used to check status of objects for pick and place

    # keywords used in the 1st line of file are :
    #   fpga_board
    #   coke_can
    #   glue

    # keywords used in the 2nd line of file are :
    #   vision -> getting clear view of all objects 
    #   interm -> moving ur5 to an intermediate position where y = z_allowance/roll_angle
    #   final -> move closer to object for further correcting Y coord
    #   pnp_objd -> correct Y values by bringing the center of bounding box to image center
    #   grasp -> grasp the object by hard coded grasping angles & navigate to drop location
    #   drop -> drop the object in drop box through hard coded dropbox poses & navigate to next picking location

    #   at any given time the structure of the file is as follows:
    #       {object_to_pick}
    #       {checkpoint}
    #       {detected_coorinates_of_object/pixels}
    #       {coorinates_of_object/real_world}
    #       {roll angle of gripper for object} {allowance_y} {allowance_z}

    # objects.txt stores the data in the following structure:
    #       {object}
    #       {allowance_for_y} {allowance_for_z}
    #       {gripper_angle_roll} {gripper_angle_pitch} {gripper_angle_yaw}


    # TensorFlow object detection was used for object detection 
    # ssd_mobilenet_v1_coco_11_06_2017 was trained on custom dataset 
    # Gdrive Link for the trained model
    #   https://drive.google.com/drive/folders/1TUrpOQtc_qeZRNbwEfrsupzmn2AJa_wo?usp=sharing

    rospy.init_node('object_detect', anonymous=True)
    obj=Detector()
    try:
		rospy.spin()
    except KeyboardInterrupt:
		print("ShutDown")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)