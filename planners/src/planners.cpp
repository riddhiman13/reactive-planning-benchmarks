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
#include <moveit/dynamics_solver/dynamics_solver.h>

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

double calculateTrajectoryLength(const trajectory_msgs::JointTrajectory& trajectory) 
{
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

double calculateNormalizedJerkScoreCartesian(const moveit::planning_interface::MoveGroupInterface& move_group, const trajectory_msgs::JointTrajectory& trajectory)
{
    if (trajectory.points.size() < 3) {
        ROS_WARN("Not enough trajectory points => NJS=0");
        return 0.0;
    }

    double T = trajectory.points.back().time_from_start.toSec();
    if (T < 1e-9) {
        ROS_WARN("Trajectory time=0 => NJS=0");
        return 0.0;
    }
    std::vector<Eigen::Vector3d> cart_positions;
    cart_positions.reserve(trajectory.points.size());
    std::vector<double> times;
    times.reserve(trajectory.points.size());

    robot_state::RobotStatePtr ref_state = move_group.getCurrentState();
    if (!ref_state) {
        ROS_ERROR("No valid RobotState from move_group => NJS=0");
        return 0.0;
    }

    std::string eef_link = move_group.getEndEffectorLink();
    if (eef_link.empty()) {
        ROS_ERROR("No end effector link found => NJS=0");
        return 0.0;
    }

    for (size_t i=0; i<trajectory.points.size(); ++i)
    {
        // clone
        robot_state::RobotState st(*ref_state);
        // set joint positions
        st.setVariablePositions(trajectory.joint_names, trajectory.points[i].positions);

        // FK => getGlobalLinkTransform
        const Eigen::Isometry3d& tf = st.getGlobalLinkTransform(eef_link);
        cart_positions.push_back(tf.translation());

        double tsec = trajectory.points[i].time_from_start.toSec();
        times.push_back(tsec);
    }

    double L=0.0;
    for (size_t i=1; i<cart_positions.size(); ++i) {
        L += (cart_positions[i] - cart_positions[i-1]).norm();
    }
    if (L < 1e-9) {
        ROS_WARN("End effector path length ~0 => NJS=0");
        return 0.0;
    }

    size_t n = cart_positions.size();
    std::vector<Eigen::Vector3d> velocity(n), acceleration(n);

    for (size_t i=0; i +1 < n; ++i)
    {
        double dt = times[i+1] - times[i];
        if (dt<1e-9) dt=1e-9;
        velocity[i] = (cart_positions[i+1] - cart_positions[i]) / dt;
        double speed_m_s = velocity[i].norm();
        ROS_INFO("velocity[%zu] = %.4f (m/s), dt=%.4f s", 
                 i, speed_m_s, dt);
    
    }

    ROS_WARN("T=%.3f", T);
    ROS_WARN("L=%.3f", L);
    velocity[n-1] = velocity[n-2]; // near copy

    for (size_t i=0; i +1 < n; ++i)
    {
        double dt = times[i+1] - times[i];
        if (dt<1e-9) dt=1e-9;
        acceleration[i] = (velocity[i+1] - velocity[i]) / dt;
    }
    acceleration[n-1] = acceleration[n-2];

    double jerk_sum=0.0;
    for (size_t i=0; i +1 < n; ++i)
    {
        double dt = times[i+1] - times[i];
        if (dt<1e-9) dt=1e-9;

        Eigen::Vector3d jerk = (acceleration[i+1] - acceleration[i]) / dt;
        jerk_sum += jerk.squaredNorm();
    }


    double njs = std::sqrt( std::pow(T,5)/(L*L) * jerk_sum );

    return njs;
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

std::vector<Eigen::VectorXd> extractJointTrajectoryPoints(const trajectory_msgs::JointTrajectory& traj) {
    std::vector<Eigen::VectorXd> points;
    for (const auto& pt : traj.points) {
        Eigen::VectorXd v(pt.positions.size());
        for (size_t i = 0; i < pt.positions.size(); ++i) {
            v[i] = pt.positions[i];
        }
        points.push_back(v);
    }
    return points;
}

std::vector<Eigen::VectorXd> generateStraightLinePath(const Eigen::VectorXd& start,
                                                      const Eigen::VectorXd& end,
                                                      int num_points) {
    std::vector<Eigen::VectorXd> path;
    for (int i = 0; i < num_points; ++i) {
        double alpha = static_cast<double>(i) / (num_points - 1);
        path.push_back((1 - alpha) * start + alpha * end);
    }
    return path;
}

double euclidean(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    return (a - b).norm();
}

double discreteFrechetDistance(const std::vector<Eigen::VectorXd>& path1,
                               const std::vector<Eigen::VectorXd>& path2) {
    int m = path1.size();
    int n = path2.size();
    std::vector<std::vector<double>> dp(m, std::vector<double>(n, -1.0));

    std::function<double(int, int)> compute = [&](int i, int j) -> double {
        if (dp[i][j] > -0.5) return dp[i][j];

        double dist = euclidean(path1[i], path2[j]);

        if (i == 0 && j == 0) {
            dp[i][j] = dist;
        } else if (i == 0) {
            dp[i][j] = std::max(compute(0, j - 1), dist);
        } else if (j == 0) {
            dp[i][j] = std::max(compute(i - 1, 0), dist);
        } else {
            dp[i][j] = std::max(
                std::min({compute(i - 1, j), compute(i - 1, j - 1), compute(i, j - 1)}),
                dist);
        }
        return dp[i][j];
    };

    return compute(m - 1, n - 1);
}


double compareTrajectoryWithStraightLine(const trajectory_msgs::JointTrajectory& traj) {
    auto path_real = extractJointTrajectoryPoints(traj);
    if (path_real.empty()) return 0.0;

    const Eigen::VectorXd& start = path_real.front();
    const Eigen::VectorXd& end = path_real.back();
    auto path_straight = generateStraightLinePath(start, end, path_real.size());

    return discreteFrechetDistance(path_real, path_straight);
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


double calculateVarianceOfTCPSpeed(const trajectory_msgs::JointTrajectory& trajectory,
                                   const moveit::core::RobotStatePtr& state,
                                   const moveit::core::JointModelGroup* joint_model_group,
                                   double frequency_hz = 10.0) {
    std::vector<double> tcp_speeds;

    for (size_t i = 1; i < trajectory.points.size(); ++i) {
        robot_state::RobotState state_prev(*state);
        robot_state::RobotState state_curr(*state);

        state_prev.setVariablePositions(trajectory.joint_names, trajectory.points[i - 1].positions);
        state_curr.setVariablePositions(trajectory.joint_names, trajectory.points[i].positions);

        const Eigen::Isometry3d& pose_prev = state_prev.getGlobalLinkTransform(joint_model_group->getLinkModelNames().back());
        const Eigen::Isometry3d& pose_curr = state_curr.getGlobalLinkTransform(joint_model_group->getLinkModelNames().back());

        Eigen::Vector3d p_prev = pose_prev.translation();
        Eigen::Vector3d p_curr = pose_curr.translation();

        double dt = (trajectory.points[i].time_from_start - trajectory.points[i - 1].time_from_start).toSec();
        if (dt <= 0.0) dt = 1.0 / frequency_hz;

        double speed = (p_curr - p_prev).norm() / dt;  // m/s
        tcp_speeds.push_back(speed * 100.0); // convert to cm/s
    }

    if (tcp_speeds.empty()) return 0.0;

    double mean = std::accumulate(tcp_speeds.begin(), tcp_speeds.end(), 0.0) / tcp_speeds.size();
    double variance = 0.0;
    for (double s : tcp_speeds) {
        variance += (s - mean) * (s - mean);
    }
    return variance / tcp_speeds.size();
}

void logMetrics(std::ofstream& log_file, const std::string& pipeline, const std::string& planner_ID,
                double planning_time, double ee_distance, double avg_joint_movement, double frechet,
                double energy ,double njs, double tcp_speed_var, int successful, int total) 
                {
if (log_file.is_open()) 
{
    log_file << "----------------------------------------\n"
             << "Planning Time: " << std::fixed << std::setprecision(4) << planning_time << " s\n"
             << "Avg Joint Movement: " << avg_joint_movement << " rad\n"
             << "NJS: " << njs << "\n"
             // << "Energy: " << energy << "\n"  // 如未启用可注释
             << "Frechet Distance: " << frechet << "\n"
             << "TCP Speed Variance: " << tcp_speed_var << " cm^2/s^2\n"
             << "TCP traveled Distance: " << ee_distance << " m\n"
             // << "Success Rate: " << (100.0 * successful / total) << "%\n" // 如未启用可注释
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

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(kinematic_model));

    std::string planner_ID, planning_pipeline;
    ros::param::param<std::string>("planning_pipeline", planning_pipeline, "ompl");
    ros::param::param<std::string>("default_planner_config", planner_ID, "RRT");
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
                double frechet = compareTrajectoryWithStraightLine(my_plan.trajectory_.joint_trajectory);
                double energy = calculateEnergyConsumption(my_plan.trajectory_.joint_trajectory);
                double njs = calculateNormalizedJerkScoreCartesian(move_group, my_plan.trajectory_.joint_trajectory);
                double ee_distance = calculateEndEffectorDistance(move_group, my_plan.trajectory_.joint_trajectory);
                double tcp_speed_var = calculateVarianceOfTCPSpeed(my_plan.trajectory_.joint_trajectory, robot_state, joint_model_group);
                ROS_INFO("----------------------------------------");
                ROS_INFO("Planning Time: %.4f s", planning_time);
                ROS_INFO("Avg Joint Movement: %.4f rad", avg_joint_movement);
                ROS_INFO("NJS: %.4f", njs);
                //ROS_INFO("[Metrics] Energy: %.4f", energy);
                ROS_INFO("Frechet: %.4f", frechet);
                ROS_INFO("TCP Speed Variance: %.4f cm^2/s^2", tcp_speed_var);
                ROS_INFO("TCP traveled Distance: %.4f m", ee_distance);
                // ROS_INFO("[Metrics] Success Rate: %.2f%%", 100.0 * successful_plans / total_attempts);
                ROS_INFO("----------------------------------------");

                if (log_file.is_open()) 
{
    std::string planner_id_for_log = (planning_pipeline == "ompl") ? move_group.getPlannerId() : planning_pipeline;

    log_file << "----------------------------------------\n"
             << "Planner ID: " << planner_id_for_log << "\n"
             << "Planning Time: " << std::fixed << std::setprecision(4) << planning_time << " s\n"
             << "Avg Joint Movement: " << avg_joint_movement << " rad\n"
             << "NJS: " << njs << "\n"
             //<< "Energy: " << energy << "\n"
             //<< "Frechet Distance: " << frechet << "\n"
             << "TCP Speed Variance: " << tcp_speed_var << " cm^2/s^2\n"
             << "TCP traveled Distance: " << ee_distance << " m\n"
             //<< "Success Rate: " << (100.0 * successful / total) << "%\n"
             << "----------------------------------------\n";
    log_file.flush();
    
}
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
