#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include <sys/stat.h>
#include <tf2_eigen/tf2_eigen.h> 

geometry_msgs::Pose goal_pose;
bool goal_received = false;

void goalCallback(const geometry_msgs::Point::ConstPtr& msg) {
    goal_pose.position.x = msg->x;
    goal_pose.position.y = msg->y;
    goal_pose.position.z = msg->z;
    goal_pose.orientation.w = 1.0;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.0;
    goal_received = true;
}

bool fileExists(const std::string& path) {
    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0);
}


double calculateAvgJointMovement(const trajectory_msgs::JointTrajectory& trajectory) {
    if (trajectory.points.empty()) return 0.0;

    double total_movement = 0.0;
    int T = trajectory.points.size();
    int n = trajectory.joint_names.size();

    for (const auto& point : trajectory.points) {
        double joint_sum = 0.0;
        for (double theta : point.positions) {
            joint_sum += fabs(theta);
        }
        total_movement += (joint_sum / n); 
    }

    return total_movement / T; 
}

double calculateTrajectoryLength(const trajectory_msgs::JointTrajectory& trajectory) {
    if (trajectory.points.size() < 2) return 0.0;
    double length = 0.0;
    for (size_t i = 1; i < trajectory.points.size(); ++i) {
        double segment_length = 0.0;
        for (size_t j = 0; j < trajectory.points[i].positions.size(); ++j) {
            double delta = trajectory.points[i].positions[j] - trajectory.points[i - 1].positions[j];
            segment_length += delta * delta;
        }
        length += std::sqrt(segment_length);
    }
    return length;
}

double calculateNormalizedJerkScore(const trajectory_msgs::JointTrajectory& trajectory) {
    if (trajectory.points.size() < 3) return 0.0;
    double jerk_sum = 0.0;
    double T = trajectory.points.back().time_from_start.toSec();
    double L = calculateTrajectoryLength(trajectory);
    if (L == 0.0) return 0.0;
    for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
        double jerk_norm = 0.0;
        for (size_t j = 0; j < trajectory.points[i].accelerations.size(); ++j) {
            double dt1 = trajectory.points[i].time_from_start.toSec() - trajectory.points[i - 1].time_from_start.toSec();
            double dt2 = trajectory.points[i + 1].time_from_start.toSec() - trajectory.points[i].time_from_start.toSec();
            if (dt1 > 0.0 && dt2 > 0.0) {
                double jerk = (trajectory.points[i + 1].accelerations[j] - trajectory.points[i].accelerations[j]) / dt2;
                jerk_norm += jerk * jerk;
            }
        }
        jerk_sum += jerk_norm;
    }
    return sqrt(pow(T, 5) / pow(L, 2) * jerk_sum);
}

double calculateEndEffectorDistance(const moveit::planning_interface::MoveGroupInterface& move_group, 
                                    const trajectory_msgs::JointTrajectory& trajectory) {
    if (trajectory.points.size() < 2) return 0.0;
    double distance = 0.0;
    
    for (size_t i = 1; i < trajectory.points.size(); ++i) {
        robot_state::RobotState state1(*move_group.getCurrentState());
        robot_state::RobotState state2(*move_group.getCurrentState());
        
        state1.setVariablePositions(trajectory.joint_names, trajectory.points[i - 1].positions);
        state2.setVariablePositions(trajectory.joint_names, trajectory.points[i].positions);
        
        Eigen::Isometry3d tf1 = state1.getGlobalLinkTransform(move_group.getEndEffectorLink());
        Eigen::Isometry3d tf2 = state2.getGlobalLinkTransform(move_group.getEndEffectorLink());
        
        geometry_msgs::Pose pose1, pose2;
        tf2::convert(tf1, pose1);
        tf2::convert(tf2, pose2);

        double dx = pose2.position.x - pose1.position.x;
        double dy = pose2.position.y - pose1.position.y;
        double dz = pose2.position.z - pose1.position.z;
        distance += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return distance;
}

double getPlanningTime(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    return plan.planning_time_;
}

double calculateFrechetDistance(const std::vector<double>& start_positions,
                                 const std::vector<double>& end_positions) {
    double frechet = 0.0;
    for (size_t i = 0; i < start_positions.size(); ++i) {
        frechet += std::pow(end_positions[i] - start_positions[i], 2);
    }
    return std::sqrt(frechet);
}

double calculateEnergyConsumption(const trajectory_msgs::JointTrajectory& trajectory) {
    double energy = 0.0;
    const auto& points = trajectory.points;
    for (size_t i = 1; i < points.size(); ++i) {
        for (size_t j = 0; j < points[i].positions.size(); ++j) {
            double dq = points[i].positions[j] - points[i - 1].positions[j];
            double dt = points[i].time_from_start.toSec() - points[i - 1].time_from_start.toSec();
            if (dt > 0.0) {
                double v = dq / dt;
                double a = (points[i].velocities[j] - points[i - 1].velocities[j]) / dt;
                energy += std::abs(v * a);
            }
        }
    }
    return energy;
}

bool planToGoal(const geometry_msgs::Pose& goal_pose,
                moveit::planning_interface::MoveGroupInterface& move_group,
                moveit::planning_interface::MoveGroupInterface::Plan& plan,
                const std::string& planning_pipeline,
                const moveit::core::JointModelGroup* joint_model_group) {
    if (planning_pipeline == "ompl") {
        move_group.setPoseTarget(goal_pose);
        return (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    } else {
        robot_state::RobotStatePtr current_state = move_group.getCurrentState();
        robot_state::RobotState ik_state(*current_state);
        if (!ik_state.setFromIK(joint_model_group, goal_pose, 3, 0.1)) {
            ROS_ERROR("IK solution not found.");
            return false;
        }
        std::vector<double> goal_joint_positions;
        ik_state.copyJointGroupPositions(joint_model_group, goal_joint_positions);
        ros::param::set("/move_group/planning_pipeline", planning_pipeline);
        move_group.setPlannerId(planning_pipeline);
        ros::Duration(1.0).sleep();
        move_group.setJointValueTarget(goal_joint_positions);
        return (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }
}

void logMetrics(std::ofstream& log_file, const std::string& pipeline, const std::string& planner_ID,
                double planning_time, double ee_distance, double avg_joint_movement, double frechet,
                double energy ,double njs, int successful, int total) {
    if (log_file.is_open()) {
        log_file << "Pipeline: " << pipeline << "\n"
                 << "Planner: " << planner_ID << "\n"
                 << "Planning Time: " << std::fixed << std::setprecision(4) << planning_time << " s\n"
                 << "Planned End Effector Distance: " << ee_distance << " m\n"
                 << "Avg Joint Movement: " << avg_joint_movement << " rad\n"
                 << "Frechet Distance: " << frechet << "\n"
                 << "Energy: " << energy << "\n"
                 << "Normalized Jerk Score: " << njs << "\n" 
                 << "Success Rate: " << (100.0 * successful / total) << "%\n"
                 << "----------------------------------------\n";
        log_file.flush();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    std::string planner_ID, planning_pipeline;
    ros::param::param<std::string>("planning_pipeline", planning_pipeline, "ompl");
    ros::param::param<std::string>("default_planner_config", planner_ID, "RRTstar");
    move_group.setPlannerId(planner_ID);

    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::Point>("goal_position", 10, goalCallback);
    ros::Rate rate(10);

    int total_attempts = 0, successful_plans = 0;
    std::string home_dir = std::getenv("HOME");
    std::string log_path = home_dir + "/benchmark_ws/planning_metrics.txt";
    std::ofstream log_file(log_path, std::ios::app);

    while (ros::ok()) {
        ros::spinOnce();

        if (goal_received) {
            total_attempts++;
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            std::vector<double> start_positions;
            move_group.getCurrentState()->copyJointGroupPositions(joint_model_group, start_positions);
            ros::Time t0 = ros::Time::now();

            bool success = planToGoal(goal_pose, move_group, my_plan, planning_pipeline, joint_model_group);
            double planning_time = getPlanningTime(my_plan);

            if (success) {
                successful_plans++;
                std::vector<double> end_positions = my_plan.trajectory_.joint_trajectory.points.back().positions;
                double avg_joint_movement = calculateAvgJointMovement(my_plan.trajectory_.joint_trajectory);
                double frechet = calculateFrechetDistance(start_positions, end_positions);
                double energy = calculateEnergyConsumption(my_plan.trajectory_.joint_trajectory);
                double njs = calculateNormalizedJerkScore(my_plan.trajectory_.joint_trajectory);
                double ee_distance = calculateEndEffectorDistance(move_group, my_plan.trajectory_.joint_trajectory);

                ROS_INFO("[Metrics] Planning Time: %.4f s", planning_time);
                ROS_INFO("[Metrics] Planned End Effector Distance: %.4f m", ee_distance);
                ROS_INFO("[Metrics] Avg Joint Movement: %.4f rad", avg_joint_movement);
                ROS_INFO("[Metrics] Frechet Distance: %.4f", frechet);
                ROS_INFO("[Metrics] Energy: %.4f", energy);
                ROS_INFO("[Metrics] Success Rate: %.2f%%", 100.0 * successful_plans / total_attempts);
                ROS_INFO("[Metrics] Normalized Jerk Score: %.4f", njs);

                logMetrics(log_file, planning_pipeline, planner_ID, planning_time, ee_distance,
                           avg_joint_movement, frechet, energy, njs,
                           successful_plans, total_attempts);

                visual_tools.publishTrajectoryLine(my_plan.trajectory_, move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP));
                visual_tools.trigger();
                visual_tools.prompt("Press 'next' in RViz to execute the motion");
                move_group.execute(my_plan);
            }
            goal_received = false;
        }
        rate.sleep();
    }

    if (log_file.is_open()) 
    {
        log_file.close();
    }

    ros::shutdown();
    return 0;
}
