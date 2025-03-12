#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <geometry_msgs/Pose.h>
#include <yaml-cpp/yaml.h>

// Function to load start and goal poses from YAML
void loadConfig(const std::string& yaml_file, geometry_msgs::Pose& start_pose, geometry_msgs::Pose& goal_pose) {
    try {
        YAML::Node config = YAML::LoadFile(yaml_file);

        start_pose.position.x = config["start_pose"]["position"]["x"].as<double>();
        start_pose.position.y = config["start_pose"]["position"]["y"].as<double>();
        start_pose.position.z = config["start_pose"]["position"]["z"].as<double>();
        start_pose.orientation.x = config["start_pose"]["orientation"]["x"].as<double>();
        start_pose.orientation.y = config["start_pose"]["orientation"]["y"].as<double>();
        start_pose.orientation.z = config["start_pose"]["orientation"]["z"].as<double>();
        start_pose.orientation.w = config["start_pose"]["orientation"]["w"].as<double>();

        goal_pose.position.x = config["goal_pose"]["position"]["x"].as<double>();
        goal_pose.position.y = config["goal_pose"]["position"]["y"].as<double>();
        goal_pose.position.z = config["goal_pose"]["position"]["z"].as<double>();
        goal_pose.orientation.x = config["goal_pose"]["orientation"]["x"].as<double>();
        goal_pose.orientation.y = config["goal_pose"]["orientation"]["y"].as<double>();
        goal_pose.orientation.z = config["goal_pose"]["orientation"]["z"].as<double>();
        goal_pose.orientation.w = config["goal_pose"]["orientation"]["w"].as<double>();

    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML parsing error: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "stomp_planner_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";  // Change this if using another robot

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::string yaml_file;

    if (ros::param::get("scene_file", yaml_file)) {
        ROS_INFO("Scene File: %s", yaml_file.c_str());
    } else {
        ROS_WARN("Failed to get 'Scene_file'.");
    }

    // Load start/goal poses
    geometry_msgs::Pose start_pose, goal_pose;
    loadConfig(yaml_file, start_pose, goal_pose);

    // Move to Start Position
    //move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(goal_pose);
    move_group.move();

    ros::shutdown();
    return 0;
}
