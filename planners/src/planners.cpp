#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

// Global variable to store received goal
geometry_msgs::Pose goal_pose;
bool goal_received = false; // Flag to check if goal has been received

// Callback function to update goal position
void goalCallback(const geometry_msgs::Point::ConstPtr& msg) 
{
    goal_pose.position.x = msg->x;
    goal_pose.position.y = msg->y;
    goal_pose.position.z = msg->z;
    
    // Set default orientation
    goal_pose.orientation.w = 1.0;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.0;

    goal_received = true; 
    //ROS_INFO("New Goal Received: x=%f, y=%f, z=%f", goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Define the planning group for MoveIt!
    static const std::string PLANNING_GROUP = "panda_arm";  
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Load Robot Model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotState start_state(kinematic_model);

    std::string planner_ID;
    std::string planning_pipeline;
    
    // 获取当前规划管道
    if (ros::param::get("planning_pipeline", planning_pipeline)) {
        ROS_INFO("Using Planning Pipeline: %s", planning_pipeline.c_str());
    } else {
        ROS_WARN("Failed to get 'planning_pipeline'. Defaulting to OMPL.");
        planning_pipeline = "ompl"; // 默认使用 OMPL
    }
    
    // 仅在 OMPL 时，设置 planner ID
    if (planning_pipeline == "ompl") {
        if (ros::param::get("default_planner_config", planner_ID)) {
            ROS_INFO("Using Planner: %s", planner_ID.c_str());
            move_group.setPlannerId(planner_ID);
        } else {
            ROS_WARN("Failed to get 'default_planner_config'. Using default OMPL planner.");
            planner_ID = "RRTstar";  
            move_group.setPlannerId(planner_ID);
        }
    }
    
    // Initialize RViz Visualization Tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // Subscribe to goal position topic
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::Point>("goal_position", 10, goalCallback);

    ros::Rate rate(10);  

    while (ros::ok()) {
        ros::spinOnce();  

        if (goal_received) {
            ROS_INFO("Planning to new goal position using %s...", planning_pipeline.c_str());

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = false;

            if (planning_pipeline == "ompl") {
                // 使用 OMPL 进行规划
                move_group.setPoseTarget(goal_pose);
                success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            } 
            else if (planning_pipeline == "chomp") {
                ROS_INFO("Using OMPL (RRTConnect) to generate initial trajectory for CHOMP...");
            
                // **手动切换为 OMPL**
                ros::param::set("/move_group/planning_pipeline", "ompl");
                move_group.setPlannerId("RRTConnect");
            
                // 规划前等待参数生效
                ros::Duration(1.0).sleep();
            
                moveit::planning_interface::MoveGroupInterface::Plan ompl_plan;
                move_group.setPoseTarget(goal_pose);
                bool ompl_success = (move_group.plan(ompl_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            
                std::vector<double> goal_joint_positions;
                bool success = false;
            
                if (!ompl_success) {
                    ROS_ERROR("OMPL failed. Trying to compute goal_joint_positions using IK...");
            
                    // **使用 IK 计算 goal_pose 对应的关节角度**
                    robot_state::RobotState start_state(*move_group.getCurrentState());
                    bool found_ik = start_state.setFromIK(joint_model_group, goal_pose);
            
                    if (!found_ik) {
                        ROS_ERROR("CHOMP: Inverse Kinematics solution not found for the given goal pose!");
                    } else {
                        start_state.copyJointGroupPositions(joint_model_group, goal_joint_positions);
            
                        // **打印 goal_joint_positions**
                        ROS_INFO("Goal Joint Positions from IK:");
                        for (size_t i = 0; i < goal_joint_positions.size(); i++) {
                            ROS_INFO("Joint[%zu]: %f", i, goal_joint_positions[i]);
                        }
                    }
                } else {
                    ROS_INFO("OMPL (RRTConnect) generated initial trajectory. Passing to CHOMP...");
            
                    // **从 OMPL 轨迹提取 goal_joint_positions**
                    if (!ompl_plan.trajectory_.joint_trajectory.points.empty()) {
                        goal_joint_positions = ompl_plan.trajectory_.joint_trajectory.points.back().positions;
                        ROS_INFO("Goal Joint Positions extracted from OMPL:");
                    } else {
                        ROS_WARN("OMPL trajectory is empty. Cannot extract goal joint positions.");
                    }
                }
            
                if (!goal_joint_positions.empty()) {
                    // **手动切换回 CHOMP**
                    ros::param::set("/move_group/planning_pipeline", "chomp");
                    move_group.setPlannerId("chomp");
            
                    // 规划前等待参数生效
                    ros::Duration(1.0).sleep();
            
                    // **让 CHOMP 处理关节空间目标**
                    move_group.setJointValueTarget(goal_joint_positions);
                    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            
                    if (success) {
                        move_group.execute(my_plan);
                    } else {
                        ROS_ERROR("CHOMP failed to generate a valid plan.");
                    }
                } else {
                    ROS_ERROR("No valid goal joint positions found!");
                }
            }
            
            
            ROS_INFO("Planning %s", success ? "SUCCEEDED" : "FAILED");

            if (success) {
                // 可视化
                visual_tools.publishTrajectoryLine(my_plan.trajectory_, move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP));
                visual_tools.trigger();
                visual_tools.prompt("Press 'next' in RViz to execute the motion");

                // 执行规划
                move_group.execute(my_plan);
            }

            goal_received = false; 
        }

        rate.sleep();  
    }

    ros::shutdown();
    return 0;
}
