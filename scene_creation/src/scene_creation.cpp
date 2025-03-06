#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/CollisionObject.h>
#include <yaml-cpp/yaml.h>

std::vector<moveit_msgs::CollisionObject> collision_objects;

// Function to print the Scene Objects details
void printCollisionObjects(const std::vector<moveit_msgs::CollisionObject>& collision_objects) {
    for (size_t i = 0; i < collision_objects.size(); ++i) {
        const auto& obj = collision_objects[i];
        ROS_INFO("Collision Object [%lu]:", i + 1);
        ROS_INFO("  ID: %s", obj.id.c_str());
        ROS_INFO("  Frame ID: %s", obj.header.frame_id.c_str());

        for (size_t j = 0; j < obj.primitives.size(); ++j) {
            const auto& primitive = obj.primitives[j];
            ROS_INFO("  Primitive [%lu]:", j + 1);
            switch (primitive.type) {
                case shape_msgs::SolidPrimitive::BOX:
                    ROS_INFO("    Type: BOX");
                    ROS_INFO("    Dimensions: [x: %.3f, y: %.3f, z: %.3f]",
                             primitive.dimensions[0], primitive.dimensions[1], primitive.dimensions[2]);
                    break;
                case shape_msgs::SolidPrimitive::SPHERE:
                    ROS_INFO("    Type: SPHERE");
                    ROS_INFO("    Radius: %.3f", primitive.dimensions[0]);
                    break;
                case shape_msgs::SolidPrimitive::CYLINDER:
                    ROS_INFO("    Type: CYLINDER");
                    ROS_INFO("    Height: %.3f, Radius: %.3f",
                             primitive.dimensions[0], primitive.dimensions[1]);
                    break;
                default:
                    ROS_WARN("    Unknown primitive type.");
            }
        }

        for (size_t j = 0; j < obj.primitive_poses.size(); ++j) {
            const auto& pose = obj.primitive_poses[j];
            ROS_INFO("  Pose [%lu]:", j + 1);
            ROS_INFO("    Position: [x: %.3f, y: %.3f, z: %.3f]",
                     pose.position.x, pose.position.y, pose.position.z);
            ROS_INFO("    Orientation: [x: %.3f, y: %.3f, z: %.3f, w: %.3f]",
                     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        }
    }
}

// Function to load obstacles and start/goal poses from a YAML file
void loadConfig(const std::string& yaml_file, const std::string planning_frame) {
    
    YAML::Node config = YAML::LoadFile(yaml_file);

    // Load obstacles
    for (const auto& obstacle : config["obstacles"]) {
        moveit_msgs::CollisionObject collision_object;

        collision_object.header.frame_id =  planning_frame;
        collision_object.id = "obstacle_" + std::to_string(obstacle["id"].as<int>());


        // Define the obstacle's shape
        shape_msgs::SolidPrimitive primitive;
        if (obstacle["type"].as<int>() == 2) { // Sphere
            primitive.type = primitive.SPHERE;
            primitive.dimensions.resize(1);
            primitive.dimensions[0] = obstacle["scale"]["x"].as<double>() * 0.5; // Radius
        } else if (obstacle["type"].as<int>() == 3) { // Cylinder
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[0] = obstacle["scale"]["z"].as<double>(); // Height
            primitive.dimensions[1] = obstacle["scale"]["x"].as<double>() * 0.5; // Radius
        } else if (obstacle["type"].as<int>() == 1) { // Cuboid
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = obstacle["scale"]["z"].as<double>(); // Height
            primitive.dimensions[1] = obstacle["scale"]["x"].as<double>(); // Radius
            primitive.dimensions[2] = obstacle["scale"]["x"].as<double>(); // Radius
        }

        // Define the obstacle's initial pose
        geometry_msgs::Pose pose;
        pose.position.x = obstacle["pose"]["position"]["x"].as<double>();
        pose.position.y = obstacle["pose"]["position"]["y"].as<double>();
        pose.position.z = obstacle["pose"]["position"]["z"].as<double>();
        pose.orientation.x = obstacle["pose"]["orientation"]["x"].as<double>();
        pose.orientation.y = obstacle["pose"]["orientation"]["y"].as<double>();
        pose.orientation.z = obstacle["pose"]["orientation"]["z"].as<double>();
        pose.orientation.w = obstacle["pose"]["orientation"]["w"].as<double>();

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "panda_obstacle_avoidance");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm"; //change this to a rosparam

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    std::string planning_frame;
    std::string yaml_file;

    if (ros::param::get("scene_file", yaml_file)) {
        ROS_INFO("Scene File: %s", yaml_file.c_str());
    } else {
        ROS_WARN("Failed to get 'Scene_file'.");
    }
    
    // Load obstacles and poses from the YAML file
    planning_frame = move_group.getPlanningFrame();
    loadConfig(yaml_file, planning_frame);
    printCollisionObjects(collision_objects);

    // Add initial obstacles to the planning scene
    planning_scene_interface.addCollisionObjects(collision_objects);
    
    ros::shutdown();
    return 0;
}
