#ifndef IIWA_CONTROLLER_NODE_HPP
#define IIWA_CONTROLLER_NODE_HPP

#include <ros/ros.h>
#include <ros/topic.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <iiwa_ros.h>
#include <random>
#include "iiwa_controller/SetTarget.h"


class KUKAController 
{
    public:
        KUKAController();
        ~KUKAController();
        bool goToInitialPose(moveit::planning_interface::MoveGroupInterface &group, 
                             moveit::planning_interface::MoveGroupInterface::Plan &plan);
        bool goToPose(moveit::planning_interface::MoveGroupInterface &group, 
                      moveit::planning_interface::MoveGroupInterface::Plan &plan, 
                      const geometry_msgs::PoseStamped &pose);
        bool goToPoses(moveit::planning_interface::MoveGroupInterface &group, 
                       moveit::planning_interface::MoveGroupInterface::Plan &plan, 
                       const std::vector<geometry_msgs::Pose> &poses);
        bool rotateFlange(moveit::planning_interface::MoveGroupInterface &group, 
                          moveit::planning_interface::MoveGroupInterface::Plan &plan, 
                          const double a7_rad);
        void getWaypointsToGoal(const geometry_msgs::PoseStamped &start, 
                                const geometry_msgs::PoseStamped &goal, 
                                std::vector<geometry_msgs::Pose> &waypoints);
        void jiggle(const geometry_msgs::PoseStamped &move_origin, 
                    const int updown_num, 
                    std::vector<geometry_msgs::Pose> &waypoints);
        bool setTarget(iiwa_controller::SetTarget::Request &req, 
                       iiwa_controller::SetTarget::Response &res);
        bool graspObject();
        bool graspObject(double &position);
        bool forceGraspObject(const double position);
        bool releaseObject();
        bool checkObject(std_srvs::Empty::Request &req, 
                         std_srvs::Empty::Response &res);
        void mainLoop();

    private:
        enum class OperationState
        {
            DoNothing,
            Unactiveted, 
            Initialized, 
            Selected, 
            Picked, 
            Placed
        };

        ros::NodeHandle nh, private_nh;
        actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_action_client;
        ros::ServiceServer set_target_srv, check_object_srv;
        ros::ServiceClient reset_camera;
        std::mutex target_mutex;
        std::vector<std::pair<std::string, bool>> target_objects;
        boost::shared_ptr<ros::Rate> loop_rate;
        std::vector<double> initial_joint_value;
        std::string movegroup_name, ee_link_name;
        double ros_rate;
        iiwa_ros::iiwaRos my_iiwa;
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;
        ros::Publisher iiwa_pub;
        bool use_simulation;
        bool use_random_pose;
        bool do_nothing;
        double ee_offset_x, ee_offset_y, ee_offset_z, min_ee_height;
        int jiggle_num;
        int max_sleep_time;//sec
        double gripper_position;
};

#endif // IIWA_CONTROLLER_NODE_HPP