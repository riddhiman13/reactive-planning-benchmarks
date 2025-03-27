#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <fstream>
#include <vector>
#include <cmath>
#include <numeric>
#include <functional>
#include <sys/stat.h>
#include <Eigen/Dense>

geometry_msgs::Point goal_pose;
bool goal_received = false;  
bool collecting_data = false;   
ros::Time start_collect_time;    
double g_distance_to_goal = 999.0;  
trajectory_msgs::JointTrajectory global_traj;

std::vector<Eigen::Vector3d> tcp_positions; 

void goalCallback(const geometry_msgs::Point::ConstPtr& msg) 
{
    goal_pose = *msg;
    //ROS_INFO("[goalCallback] Received goal (%.3f, %.3f, %.3f)", goal_pose.x, goal_pose.y, goal_pose.z);
    collecting_data = true;
    goal_received = true;
    start_collect_time = ros::Time::now();
    //ROS_INFO("[goalCallback] => Start collecting joint states now!");
}

void distCallback(const std_msgs::Float64::ConstPtr& msg) 
{
    g_distance_to_goal = msg->data;
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) 
{
    if (!collecting_data) return;

    if (global_traj.joint_names.empty()) 
    {
        global_traj.joint_names = msg->name;
    }
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions  = msg->position;
    pt.velocities = msg->velocity;
    pt.effort     = msg->effort;
    pt.time_from_start = ros::Time::now() - start_collect_time;

    global_traj.points.push_back(pt);

    static int counter = 0;
    if (++counter % 50 == 0) 
    { 
        ROS_INFO("global_traj.size()=%zu", global_traj.points.size());
    }
}

void agentPositionCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    if (!collecting_data) return;

    Eigen::Vector3d p(msg->x, msg->y, msg->z);

    tcp_positions.push_back(p);
}

double computeTCPSpeedVarianceCM(const std::vector<Eigen::Vector3d>& positions, double freq_hz = 10.0)
{
    if (positions.size() < 2) return 0.0;

    std::vector<double> speeds;
    speeds.reserve(positions.size()-1);

    for (size_t i = 1; i < positions.size(); ++i)
    {
        Eigen::Vector3d delta = positions[i] - positions[i - 1];
        double speed_m_s = delta.norm() * freq_hz;
        // cm/s
        double speed_cm_s = speed_m_s * 100.0;
        ROS_INFO("speed_cm_s = %.3f", speed_cm_s);

        // outlier_threshold
        if (speed_cm_s > 8)
        {
            continue;
        }

                
        speeds.push_back(speed_cm_s);
    }
    if (speeds.empty()) return 0.0;

    double mean = std::accumulate(speeds.begin(), speeds.end(), 0.0) / speeds.size();
    double var  = 0.0;
    for (double s : speeds)
    {
        double diff = s - mean;
        var += diff * diff;
    }
    var /= speeds.size(); // population variance

    return var; // (cm/s)^2
}


double calculateAvgJointMovement(const trajectory_msgs::JointTrajectory& trajectory);
double calculateNJSFromAgentPositions(const std::vector<Eigen::Vector3d>& positions, double freq_hz, double outlier_threshold);
double calculateEnergyConsumption(const trajectory_msgs::JointTrajectory& trajectory);
double compareTrajectoryWithStraightLine(const trajectory_msgs::JointTrajectory& traj);
double calculateTrajectoryLength(const trajectory_msgs::JointTrajectory& trajectory);
double calculateEndEffectorDistance(const std::vector<Eigen::Vector3d>& agent_positions) ;
void printTrajectory(const trajectory_msgs::JointTrajectory& traj);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collect_and_stop");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber goal_sub = nh.subscribe("/goal_position",     1, goalCallback);
    ros::Subscriber dist_sub = nh.subscribe("/distance_to_goal", 1, distCallback);
    ros::Subscriber js_sub   = nh.subscribe("/joint_states",     10, jointStateCallback);
    ros::Subscriber agent_sub   = nh.subscribe("/agent_position",    10, agentPositionCallback);

    std::string home_dir = std::getenv("HOME");
    std::string log_path = home_dir + "/benchmark_ws/planning_metrics.txt";
    std::ofstream log_file(log_path, std::ios::app);
    if (!log_file.is_open()) 
    {
        ROS_WARN("Cannot open log file: %s", log_path.c_str());
    }
    double freq_hz = 10.0; // 
    ros::Rate loop(10);
    while (ros::ok()) 
    {
        ros::spinOnce();

        // stop collecting data when distance < 0.006 
        if (collecting_data && (g_distance_to_goal < 0.006)) {
            double planning_time = (ros::Time::now() - start_collect_time).toSec();

            size_t traj_size = global_traj.points.size();
            ROS_INFO("---------------------------------");
            ROS_INFO("Collected trajectory size=%zu (points)", traj_size);
            if (traj_size < 2) 
            {
                ROS_WARN("Not enough data => Metrics will be 0 or invalid!");
            }
            //printTrajectory(global_traj);

            double avg_movement = calculateAvgJointMovement(global_traj);
            double njs          = calculateNJSFromAgentPositions(tcp_positions,10.0, 0.08);
            double energy       = calculateEnergyConsumption(global_traj);
            double frechet      = compareTrajectoryWithStraightLine(global_traj);
            double tcp_var = computeTCPSpeedVarianceCM(tcp_positions, freq_hz);
            double distance_m = calculateEndEffectorDistance(tcp_positions);

            ROS_INFO("---------------------------------");
            ROS_INFO("PlanningTime: %.3f s", planning_time);
            ROS_INFO("AvgJointMovement: %.4f", avg_movement);
            ROS_INFO("NJS: %.4f", njs);
            //ROS_INFO("Energy: %.4f", energy);
            ROS_INFO("Frechet: %.4f", frechet);
            ROS_INFO("TCP Speed Variance: %.4f", tcp_var);
            ROS_INFO("TCP traveled distance = %.3f m", distance_m);
            ROS_INFO("---------------------------------");

            if (log_file.is_open()) 
            {
                log_file << "----------------------------------------\n"
                         << "Planner ID: Multi_agent\n"
                         << "Planning Time: " << std::fixed << std::setprecision(4) << planning_time << " s\n"
                         << "Avg Joint Movement: " << std::fixed << std::setprecision(4) << avg_movement << " rad\n"
                         << "NJS: " << std::fixed << std::setprecision(4) << njs << "\n"
                         //<< "Energy: " << std::fixed << std::setprecision(4) << energy << "\n"
                         //<< "Frechet Distance: " << std::fixed << std::setprecision(4) << frechet << "\n"
                         << "TCP Speed Variance: " << std::fixed << std::setprecision(4) << tcp_var << " cm^2/s^2\n"
                         << "TCP traveled Distance: " << std::fixed << std::setprecision(4) << distance_m << " m\n"
                         << "----------------------------------------\n";
                log_file.flush();
            }


            collecting_data = false;
            goal_received = false;
            g_distance_to_goal = 999.0; 
        }

        loop.sleep();
    }

    if (log_file.is_open()) {
        log_file.close();
    }

    ros::shutdown();
    return 0;
}

double calculateAvgJointMovement(const trajectory_msgs::JointTrajectory& trajectory) 
{
    if (trajectory.points.empty()) return 0.0;

    const int T = trajectory.points.size();            
    const int n = trajectory.joint_names.size();        
    double total_movement = 0.0;

    for (size_t t = 0; t < trajectory.points.size(); ++t) 
    {
        double sum_angles = 0.0;
        for (size_t j = 0; j < n; ++j) {
            double theta_j = trajectory.points[t].positions[j];
            sum_angles += std::fabs(theta_j);
        }
        double mean_this_t = sum_angles / n;

        total_movement += mean_this_t;
    }

    return total_movement / T;
}


double calculateTrajectoryLength(const trajectory_msgs::JointTrajectory& trajectory) 
{
    if (trajectory.points.size() < 2) return 0.0;
    double length = 0.0;
    for (size_t i = 1; i < trajectory.points.size(); ++i) {
        double seg_len = 0.0;
        for (size_t j = 0; j < trajectory.points[i].positions.size(); ++j) 
        {
            double delta = trajectory.points[i].positions[j] - trajectory.points[i-1].positions[j];
            seg_len += delta * delta;
        }
        length += std::sqrt(seg_len);
    }
    return length;
}

double calculateNJSFromAgentPositions(const std::vector<Eigen::Vector3d>& positions, double freq_hz, double outlier_threshold = 0.08)
{
    if (positions.size() < 4)
    {
        ROS_WARN("Not enough points => NJS=0");
        return 0.0;
    }

    double T = (positions.size() - 1) / freq_hz;
    if (T < 1e-9) return 0.0;

    double L = 0.0;
    for (size_t i = 1; i < positions.size(); ++i)
    {
        L += (positions[i] - positions[i - 1]).norm();
    }
    if (L < 1e-9)
    {
        ROS_WARN("Trajectory length ~0 => NJS=0");
        return 0.0;
    }

    std::vector<Eigen::Vector3d> filtered_positions;
    filtered_positions.reserve(positions.size());

    filtered_positions.push_back(positions.front());

    for (size_t i = 1; i < positions.size(); ++i)
    {
        double dt = 1.0 / freq_hz; 
        double dist = (positions[i] - positions[i-1]).norm();
        double speed_m_s = dist / dt; // m/s
        ROS_WARN("speed_m_s = %.3f", speed_m_s);

        if (outlier_threshold > 1e-9 && speed_m_s > outlier_threshold)
        {
            ROS_WARN("Skipping outlier speed=%.2f m/s > threshold=%.2f", speed_m_s, outlier_threshold);
            continue;
        }
        filtered_positions.push_back(positions[i]);
    }

    if (filtered_positions.size() < 4)
    {
        ROS_WARN("After filtering, not enough points => NJS=0");
        return 0.0;
    }
    double T2 = (filtered_positions.size() - 1) / freq_hz;
    double L2 = 0.0;
    for (size_t i = 1; i < filtered_positions.size(); ++i)
    {
        L2 += (filtered_positions[i] - filtered_positions[i-1]).norm();
    }
    if (L2 < 1e-9)
    {
        ROS_WARN("Filtered path length ~0 => NJS=0");
        return 0.0;
    }

    ROS_WARN("T2=%.3f, L2=%.3f", T2, L2);

    std::vector<Eigen::Vector3d> velocities(filtered_positions.size());
    std::vector<Eigen::Vector3d> accelerations(filtered_positions.size());

    double dt = 1.0 / freq_hz;

    for (size_t i = 0; i + 1 < filtered_positions.size(); ++i)
    {
        velocities[i] = (filtered_positions[i+1] - filtered_positions[i]) / dt;
    }
    velocities.back() = velocities[velocities.size()-2];

    // 2) acceleration
    for (size_t i = 0; i + 1 < velocities.size(); ++i)
    {
        accelerations[i] = (velocities[i+1] - velocities[i]) / dt;
    }
    accelerations.back() = accelerations[accelerations.size()-2];

    // 3) jerk
    double jerk_sum = 0.0;
    for (size_t i = 0; i + 1 < accelerations.size(); ++i)
    {
        Eigen::Vector3d jerk = (accelerations[i+1] - accelerations[i]) / dt;
        jerk_sum += jerk.squaredNorm();
    }
    double njs = std::sqrt(std::pow(T2, 5) / (L2 * L2) * jerk_sum);


    return njs;
}


double calculateEnergyConsumption(const trajectory_msgs::JointTrajectory& trajectory) 
{
    double energy = 0.0;
    for (size_t i = 1; i < trajectory.points.size(); ++i) {
        double dt = trajectory.points[i].time_from_start.toSec() - 
                    trajectory.points[i-1].time_from_start.toSec();
        if (dt <= 0.0) continue;

        for (size_t j = 0; j < trajectory.points[i].positions.size(); ++j) {
            double dq = trajectory.points[i].positions[j] - trajectory.points[i-1].positions[j];
            double v = dq/dt;
            double dv = trajectory.points[i].velocities[j] - trajectory.points[i-1].velocities[j];
            double a = dv/dt;
            energy += std::fabs(v * a);
        }
    }
    return energy;
}

// 离散Frechet
Eigen::VectorXd toEigen(const trajectory_msgs::JointTrajectoryPoint& pt) 
{
    Eigen::VectorXd v(pt.positions.size());
    for (size_t i = 0; i < pt.positions.size(); ++i) {
        v[i] = pt.positions[i];
    }
    return v;
}

std::vector<Eigen::VectorXd> extractJointPoints(const trajectory_msgs::JointTrajectory& traj) 
{
    std::vector<Eigen::VectorXd> path;
    path.reserve(traj.points.size());
    for (auto &p : traj.points) {
        path.push_back(toEigen(p));
    }
    return path;
}

std::vector<Eigen::VectorXd> generateStraightLinePath(const Eigen::VectorXd& start,
                                                      const Eigen::VectorXd& end,
                                                      int N) 
{
    std::vector<Eigen::VectorXd> path;
    path.reserve(N);
    for (int i = 0; i < N; ++i) 
    {
        double alpha = (double)i/(N-1);
        path.push_back((1.0 - alpha)*start + alpha*end);
    }
    return path;
}

double euclidean(const Eigen::VectorXd& a, const Eigen::VectorXd& b) 
{
    return (a - b).norm();
}

double discreteFrechetDistance(const std::vector<Eigen::VectorXd>& path1,
                               const std::vector<Eigen::VectorXd>& path2) 
{
    int m = path1.size(), n = path2.size();
    std::vector<std::vector<double>> dp(m, std::vector<double>(n, -1.0));

    std::function<double(int,int)> rec = [&](int i, int j) -> double 
    {
        if (dp[i][j] > -0.5) return dp[i][j];
        double dist = euclidean(path1[i], path2[j]);
        if (i==0 && j==0) {
            dp[i][j] = dist;
        } else if (i==0) {
            dp[i][j] = std::max(rec(0,j-1), dist);
        } else if (j==0) {
            dp[i][j] = std::max(rec(i-1,0), dist);
        } else {
            dp[i][j] = std::max(
                std::min({rec(i-1,j), rec(i-1,j-1), rec(i,j-1)}),
                dist
            );
        }
        return dp[i][j];
    };
    return rec(m-1, n-1);
}

double compareTrajectoryWithStraightLine(const trajectory_msgs::JointTrajectory& traj) 
{
    if (traj.points.size() < 2) return 0.0;
    auto real_path = extractJointPoints(traj);
    Eigen::VectorXd start = real_path.front();
    Eigen::VectorXd end   = real_path.back();
    auto straight_path     = generateStraightLinePath(start, end, real_path.size());
    return discreteFrechetDistance(real_path, straight_path);
}

double calculateEndEffectorDistance(const std::vector<Eigen::Vector3d>& agent_positions) 
{
    if (agent_positions.size() < 2) 
        return 0.0;

    double total_dist = 0.0; 
    for (size_t i = 1; i < agent_positions.size(); ++i)
    {
        double seg = (agent_positions[i] - agent_positions[i - 1]).norm();
        total_dist += seg; 
    }

    return total_dist;
}


void printTrajectory(const trajectory_msgs::JointTrajectory& traj)
{
    ROS_INFO("=== Printing global_traj ===");

    ROS_INFO_STREAM("Joint names: ");
    for (size_t i = 0; i < traj.joint_names.size(); ++i)
    {
        ROS_INFO_STREAM("  [" << i << "] " << traj.joint_names[i]);
    }

    ROS_INFO_STREAM("Number of trajectory points: " << traj.points.size());
    for (size_t i = 0; i < traj.points.size(); ++i)
    {
        const auto& pt = traj.points[i];
        ROS_INFO_STREAM("Point " << i 
            << ": time_from_start = " << pt.time_from_start.toSec() << " (s)");

        // 2.1 positions
        if (!pt.positions.empty())
        {
            std::stringstream pos_ss;
            pos_ss << "  positions = [";
            for (size_t j = 0; j < pt.positions.size(); ++j)
            {
                pos_ss << pt.positions[j];
                if (j + 1 < pt.positions.size()) pos_ss << ", ";
            }
            pos_ss << "]";
            ROS_INFO_STREAM(pos_ss.str());
        }

        // 2.2 velocities
        if (!pt.velocities.empty())
        {
            std::stringstream vel_ss;
            vel_ss << "  velocities = [";
            for (size_t j = 0; j < pt.velocities.size(); ++j)
            {
                vel_ss << pt.velocities[j];
                if (j + 1 < pt.velocities.size()) vel_ss << ", ";
            }
            vel_ss << "]";
            ROS_INFO_STREAM(vel_ss.str());
        }

        // 2.3 accelerations
        if (!pt.accelerations.empty())
        {
            std::stringstream acc_ss;
            acc_ss << "  accelerations = [";
            for (size_t j = 0; j < pt.accelerations.size(); ++j)
            {
                acc_ss << pt.accelerations[j];
                if (j + 1 < pt.accelerations.size()) acc_ss << ", ";
            }
            acc_ss << "]";
            ROS_INFO_STREAM(acc_ss.str());
        }

        // 2.4 efforts
        if (!pt.effort.empty())
        {
            std::stringstream eff_ss;
            eff_ss << "  effort = [";
            for (size_t j = 0; j < pt.effort.size(); ++j)
            {
                eff_ss << pt.effort[j];
                if (j + 1 < pt.effort.size()) eff_ss << ", ";
            }
            eff_ss << "]";
            ROS_INFO_STREAM(eff_ss.str());
        }
    }

    ROS_INFO("=== End of global_traj ===\n");
}
