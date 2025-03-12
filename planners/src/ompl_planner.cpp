#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

void loadConfig(const std::string& yaml_file,  
                geometry_msgs::Pose& start_pose, 
                geometry_msgs::Pose& goal_pose) {
    YAML::Node config = YAML::LoadFile(yaml_file);

    // Load start pose
    start_pose.position.x = config["start_pose"]["position"]["x"].as<double>();
    start_pose.position.y = config["start_pose"]["position"]["y"].as<double>();
    start_pose.position.z = config["start_pose"]["position"]["z"].as<double>();
    start_pose.orientation.x = config["start_pose"]["orientation"]["x"].as<double>();
    start_pose.orientation.y = config["start_pose"]["orientation"]["y"].as<double>();
    start_pose.orientation.z = config["start_pose"]["orientation"]["z"].as<double>();
    start_pose.orientation.w = config["start_pose"]["orientation"]["w"].as<double>();

    // Load goal pose
    goal_pose.position.x = config["goal_pose"]["position"]["x"].as<double>();
    goal_pose.position.y = config["goal_pose"]["position"]["y"].as<double>();
    goal_pose.position.z = config["goal_pose"]["position"]["z"].as<double>();
    goal_pose.orientation.x = config["goal_pose"]["orientation"]["x"].as<double>();
    goal_pose.orientation.y = config["goal_pose"]["orientation"]["y"].as<double>();
    goal_pose.orientation.z = config["goal_pose"]["orientation"]["z"].as<double>();
    goal_pose.orientation.w = config["goal_pose"]["orientation"]["w"].as<double>();
}


int main(int argc, char** argv) {
    //Node Intialization
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";

    //Intialize the planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    std::string planner_ID;
    std::string yaml_file;
    geometry_msgs::Pose start_pose, goal_pose;
    
    //nh.getParam("default_planner_config", planner_ID);
    
    if (ros::param::get("scene_file", yaml_file)) {
        ROS_INFO("Scene File: %s", yaml_file.c_str());
    } else {
        ROS_WARN("Failed to get 'Scene_file'.");
    }

    if (ros::param::get("default_planner_config", planner_ID)) {
        ROS_INFO("Planner: %s", planner_ID.c_str());
    } else {
        ROS_WARN("Failed to get 'default_planner_config'.");
    }

    move_group.setPlannerId(planner_ID);


    // Visualization tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");  //change this to a rosparam
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    
    // Load obstacles and poses from the YAML file
    loadConfig(yaml_file, start_pose, goal_pose);
    
    // Set start pose
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(goal_pose);

    // Plan to the target pose
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    ROS_INFO("Planning %s", success ? "SUCCEEDED" : "FAILED");

    if (success) {
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP));
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' in RViz to execute the motion");

        // Execute the planned trajectory
        move_group.execute(my_plan);
    }

    std::string planner_id = move_group.getPlannerId();
    ROS_INFO("Current Planner ID: %s", planner_id.c_str());

    ros::shutdown();
    return 0;
}
