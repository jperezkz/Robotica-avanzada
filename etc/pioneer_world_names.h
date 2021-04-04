//////////////////////////////////////////
// Node and edges names for Viriato world
//////////////////////////////////////////

#ifndef DSR_NAMES_H
#define DSR_NAMES_H

// NODES
const std::string robot_name = "pioneer";
const std::string robot_body_name = "pioneer_body";
const std::string world_name = "world";
const std::string floor_name = "floor";
const std::string pioneer_head_camera_left_name = "pioneer_camera_left";
const std::string pioneer_head_camera_right_name = "pioneer_camera_right";
const std::string pioneer_camera_virtual_name = "pioneer_camera_virtual";
const std::string pioneer_laser_name = "pioneer_laser";
const std::string battery_name = "battery";
const std::string wifi_name = "wifi";
const std::string ultrasound_name = "ultrasound";


//NODE TYPES
const std::string differentialrobot_type = "differentialrobot";
const std::string rgbd_type = "rgbd";
const std::string intention_type = "intention";
const std::string path_to_target_type = "path_to_target";

// EDGES TYPES
const std::string think_type = "thinks";
const std::string has_type = "has";

#endif