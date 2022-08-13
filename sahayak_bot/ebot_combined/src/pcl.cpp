#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

// array to get pixel values from object_detect.py
int obj_coord_arr_pixels[4];
// grasping angle
float grasping_angle=40;
float roll_allowance_line5[4];
// array to send out the calculated coordinates
float obj_coord_arr_real_world[3];
// filestreams for text files
std::fstream pose_file,control_file;

std::string file_in_ctrl;
std::string start_room_pt;
std::string current_room;
std::string pose_file_data_extraction[5];

//Detection Position of End Effector
const int NUMBER_OF_ROOMS = 5;

// Execution maps
std::string ckpt_keys[6] = {"vision","interm","final","pnp_objd","grasp","drop"};
std::string ckpt_values[6] = {"interm","final","pnp_objd","grasp","drop","vision"};
//EDIT for changing the path to minimise time
// std::string same_room_different_table_keys[3] = {"pantry_right","store_left","store_mid"};
// std::string same_room_different_table_values[3] = {"pantry_left","store_mid","store_right"};

std::string same_room_different_table_keys[10] = {"store_mid","researchS","pantry_right","pantry_left","conf"};
std::string same_room_different_table_values[10] = {"researchS","pantry_right","pantry_left","store_left","meeting_pick"};
// std::string room_map_keys[NUMBER_OF_ROOMS] = {"pantry_right","pantry_left"};
// std::string room_map_values[NUMBER_OF_ROOMS] = {"meeting_drop","meeting_drop"};
//EDIT for changing the path to minimise time

//Path to the text files
std::string CONTROL_FILE_PATH = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined/scripts/control.txt";
std::string POSE_FILE_PATH = "/home/ojas/catkin_ws/src/sahayak_bot/ebot_combined/scripts/pose.txt";

/* array for storing the current position of end effector
 to shift the coordinates to base_link */
float current_ee_coord[3];

/* All necessary data extraction functions */
/* Function used to write data to pose.txt file */
void write_pose(std::string object_name,std::string next_ckpt){
  pose_file.open(POSE_FILE_PATH,std::ofstream::out | std::ofstream::trunc);
  pose_file<<object_name;
  pose_file<<"\n";
  pose_file<<next_ckpt;
  pose_file<<"\n";
  if(next_ckpt == "interm"){
    pose_file<<grasping_angle;
  }
  else if(next_ckpt == "vision"){
    pose_file<<pose_file_data_extraction[2];
  }
  pose_file<<"\n";
  // if the next checkpoint is vision pass init to real world coord line to get ee coord
  if(next_ckpt == "vision"){
    pose_file<<"init";
  }
  // don't update the line for last 2 ckpt as pcl would pass nan
  else if(next_ckpt == "pnp_objd" || next_ckpt == "grasp"){
    pose_file<<pose_file_data_extraction[3];
  }
  // else update the line
  else{
    pose_file<<obj_coord_arr_real_world[0];
    pose_file<<" ";
    pose_file<<obj_coord_arr_real_world[1];
    pose_file<<" ";
    pose_file<<obj_coord_arr_real_world[2];
  }
  pose_file<<"\n";
  pose_file<<pose_file_data_extraction[4];
  pose_file.close();
}

/* Function used to change control */
void write_control(std::string script_name,std::string start_room_pt,std::string current_room){
  control_file.open(CONTROL_FILE_PATH,std::ofstream::out | std::ofstream::trunc);
  control_file<<script_name;
  control_file<<"\n";
  control_file<<start_room_pt;
  control_file<<"\n";
  control_file<<current_room;
  control_file.close();
}

/* Function used to extract pose.txt file data */
void read_pose(){
  pose_file.open(POSE_FILE_PATH);
  getline(pose_file,pose_file_data_extraction[0]);
  getline(pose_file,pose_file_data_extraction[1]);
  getline(pose_file,pose_file_data_extraction[2]);
  getline(pose_file,pose_file_data_extraction[3]);
  getline(pose_file,pose_file_data_extraction[4]);
  pose_file.close();
}

/* Function used to check control */
void read_control(){
  control_file.open(CONTROL_FILE_PATH);
  getline(control_file,file_in_ctrl);
  getline(control_file,start_room_pt);
  getline(control_file,current_room);
  control_file.close();
}
/* All necessary data extraction functions */


/* All necessary preliminary functions */
/* Function used to get element number from array */
int find_element_num(std::string elem,std::string array[]){
  for(int i = 0;i<sizeof(array);i++){
    if(array[i]==elem){
      return i;
    }
  }
}

/* Function used to split string into int */
void split_string_pixel_coord(std::string pixel_coord_line){
  int pos = 0;
  std::string word = "";
  for (auto x : pixel_coord_line) {
    if (x == ' ') {
      obj_coord_arr_pixels[pos] = std::stoi(word);
      std::cout<<obj_coord_arr_pixels[pos]<<std::endl;
      pos++;
      word = "";
    }
    else {
      word = word + x;
    }
  }
  obj_coord_arr_pixels[pos] = std::stoi(word);
  std::cout<<obj_coord_arr_pixels[pos]<<std::endl;
}

/* Function used to split string into float */
void split_string_real_world_coord(std::string real_world_coord_line){
  int pos = 0;
  std::string word = "";
  for (auto x : real_world_coord_line) {
    if (x == ' ') {
      current_ee_coord[pos] = stof(word);
      pos++;
      word = "";
    }
    else {
      word = word + x;
    }
  }
  current_ee_coord[pos] = stof(word);
}

/* Function used to split string into float */
void split_string_roll_allowance(std::string roll_allowance_line){
  int pos = 0;
  std::string word = "";
  for (auto x : roll_allowance_line) {
    if (x == ' ') {
      roll_allowance_line5[pos] = stof(word);
      pos++;
      word = "";
    }
    else {
      word = word + x;
    }
  }
  roll_allowance_line5[pos] = stof(word);
}
/* All necessary preliminary functions */

/**
  Function to convert 2D pixel point to 3D point by extracting point
  from PointCloud2 corresponding to input pixel coordinate. This function
  can be used to get the X,Y,Z coordinates of a feature using an 
  RGBD camera, e.g., Kinect.                                            **/
void getXYZ_object(const sensor_msgs::PointCloud2& pCloud){
  int col_width = obj_coord_arr_pixels[0];
  int row_height = obj_coord_arr_pixels[1];

  // std::cout << col_width <<std :: endl;
  // std::cout << row_height <<std :: endl;

  // Convert from u (column / width), v (row/height) to position in array
  // where X,Y,Z data starts
  int arrayPosition = row_height*pCloud.row_step + col_width*pCloud.point_step;

  // compute position in array where x,y,z data start
  int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
  int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
  int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;

  memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

  // get roll angle of gripper
  split_string_roll_allowance(pose_file_data_extraction[4]);
  float ee_theta_roll = roll_allowance_line5[0];
  if(pose_file_data_extraction[1]=="vision"){
    ee_theta_roll = roll_allowance_line5[3];
  }
  float allowance_y = roll_allowance_line5[1];
  float allowance_z = roll_allowance_line5[2];
  // std::cout << "x:" << X <<std :: endl;
  // std::cout << "y:" << Y <<std :: endl;
  // std::cout << "z:" << Z <<std :: endl;

  std::cout << "x':" << (-1)*X <<std :: endl;
  std::cout << "y':" << Z*sin(ee_theta_roll) + Y*cos(ee_theta_roll) <<std :: endl;
  std::cout << "z':" << Z*cos(ee_theta_roll) - Y*sin(ee_theta_roll) <<std :: endl;
  // std::cout << current_ee_coord[0] <<std :: endl;
  // std::cout << current_ee_coord[1] <<std :: endl;

  float X1 = X;
  float Y1 = Z*sin(ee_theta_roll) + Y*cos(ee_theta_roll);
  float Z1 = Z*cos(ee_theta_roll) - Y*sin(ee_theta_roll);

  // if(pose_file_data_extraction[1]=="interm"){
  //   int length_in_pixels = obj_coord_arr_pixels[2];    
  //   float arrayPosY1 = 0.0;
  //   float arrayPosY2 = 0.0;

  //   arrayPosition = row_height*pCloud.row_step + (col_width-length_in_pixels/2)*pCloud.point_step;
  //   memcpy(&arrayPosY1, &pCloud.data[(arrayPosition + pCloud.fields[0].offset)], sizeof(float));
  //   arrayPosition = row_height*pCloud.row_step + (col_width+length_in_pixels/2)*pCloud.point_step;
  //   memcpy(&arrayPosY2, &pCloud.data[(arrayPosition + pCloud.fields[0].offset)], sizeof(float));
    
  //   grasping_angle = (0.085-(arrayPosY2-arrayPosY1))*46/(0.085);

  //   std::cout<<length_in_pixels<<":"<<(col_width-length_in_pixels/2)<<":"<<(col_width+length_in_pixels/2)<<std::endl;
  //   std::cout<<arrayPosY1<<"-"<<arrayPosY2<<std::endl;
  //   std::cout<<grasping_angle<<std::endl;
  // }

  // bool room_cond_rotated_coord = (current_room == "store_left" || current_room == "store_mid" || current_room == "store_right");

  obj_coord_arr_real_world[0] = current_ee_coord[0] + Z1;
  obj_coord_arr_real_world[1] = current_ee_coord[1] - X1;
  obj_coord_arr_real_world[2] = current_ee_coord[2] - Y1;

  // if(room_cond_rotated_coord){
  //   obj_coord_arr_real_world[0] = current_ee_coord[0] + X1;
  //   obj_coord_arr_real_world[1] = current_ee_coord[1] + Z1;
  //   obj_coord_arr_real_world[2] = current_ee_coord[2] - Y1;    
  // }  

  if(pose_file_data_extraction[1]=="vision"){
    ee_theta_roll = roll_allowance_line5[0];
    obj_coord_arr_real_world[0] = current_ee_coord[0] + Z1 - allowance_z/tan(ee_theta_roll);
  }

  // if(pose_file_data_extraction[1]=="vision" && room_cond_rotated_coord){
  //   ee_theta_roll = roll_allowance_line5[0];
  //   obj_coord_arr_real_world[1] = current_ee_coord[1] + Z1 - allowance_z/tan(ee_theta_roll);
  // }  
}
void pixelTo3DPoint(const sensor_msgs::PointCloud2& pCloud){ 
  read_control();
  if(file_in_ctrl == "pcl"){
    read_pose();
    split_string_pixel_coord(pose_file_data_extraction[2]);
    std::cout<<file_in_ctrl<<":"<<pose_file_data_extraction[0]<<":"<<pose_file_data_extraction[1]<<":"<<pose_file_data_extraction[3]<<std::endl;
    if(obj_coord_arr_pixels[0] == 0 && obj_coord_arr_pixels[1] == 0){
      // if object not in image navigate to different point in the room
      write_pose(pose_file_data_extraction[0],pose_file_data_extraction[1]);
      write_control("pnp",current_room,same_room_different_table_values[find_element_num(current_room,same_room_different_table_keys)]);
    }
    else{
      // if object is in the image get pixel coord
      split_string_real_world_coord(pose_file_data_extraction[3]);
      getXYZ_object(pCloud);
      write_pose(pose_file_data_extraction[0],ckpt_values[find_element_num(pose_file_data_extraction[1],ckpt_keys)]);
      // if current ckpt is final then pass control to objd for pnp_objd
      if(pose_file_data_extraction[1]=="final"){
        write_control("objd",current_room,current_room);
      }
      // else follow usual loop
      else{
        write_control("pnp",current_room,current_room);
      }
    }
  }
}

int main (int argc, char** argv){
  ros::init (argc, argv, "pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe ("/camera2/depth/points2", 1, pixelTo3DPoint);
  ros::spin ();
  return 0;
}