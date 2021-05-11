//////////////////////////////////////////
// Node and edges names for Pioneer world
//////////////////////////////////////////

#ifndef DSR_NAMES_H
#define DSR_NAMES_H

// NODES
const std::string robot_name = "pioneer";
const std::string robot_body_name = "body";
const std::string robot_mind_name = "mind";

const std::string world_name = "world";
const std::string floor_name = "floor";
const std::string pioneer_head_camera_left_name = "pioneer_camera_left";
const std::string pioneer_head_camera_right_name = "pioneer_camera_right";
const std::string pioneer_camera_virtual_name = "pioneer_camera_virtual";
const std::string laser_name = "pioneer_laser";
const std::string battery_name = "battery";
const std::string wifi_name = "wifi";
const std::string ultrasound_name = "ultrasound";
const std::string current_grid_name = "current_grid";
const std::string current_intention_name = "current_intention";
const std::string current_path_name = "current_path";

//NODE TYPES
const std::string differentialrobot_type_name = "differentialrobot";
const std::string rgbd_type_name = "rgbd";
const std::string laser_type_name = "laser";
const std::string intention_type_name = "intention";
const std::string path_to_target_type_name = "path_to_target";
const std::string grid_type_name = "grid";

// EDGES TYPES
const std::string think_type = "thinks";
const std::string has_type = "has";

#endif