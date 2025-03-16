#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
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
    std::string yaml_file;

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    // Load Robot Model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotState start_state(kinematic_model);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");  //change this to a rosparam
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    if (ros::param::get("scene_file", yaml_file)) {
        ROS_INFO("Scene File: %s", yaml_file.c_str());
    } else {
        ROS_WARN("Failed to get 'Scene_file'.");
    }

    // Load start/goal poses
    geometry_msgs::Pose start_pose, goal_pose;
    loadConfig(yaml_file, start_pose, goal_pose);

    // Set start pose
    // Solve Inverse Kinematics (IK) to get joint values for the given pose
    bool found_ik = start_state.setFromIK(joint_model_group, start_pose);

    // Remove the following commented section if you want a specific start pose 
    /*
    if (found_ik) {
        // Set the computed start state in MoveGroup
        move_group.setStartState(start_state);
    } else {
        ROS_ERROR("IK solution not found for the given start pose!");
    }
    */
    move_group.setJointValueTarget(goal_pose);
    
    // Plan a motion
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

    ros::shutdown();
    return 0;
}
