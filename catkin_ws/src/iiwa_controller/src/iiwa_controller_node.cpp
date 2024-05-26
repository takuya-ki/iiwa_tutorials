#include "iiwa_controller_node.hpp"

KUKAController::KUKAController() :
    private_nh("~"), 
    gripper_action_client("/sake/sgs_action", true),
    movegroup_name("manipulator"), 
    ee_link_name("tool_link_ee"), 
    ros_rate(10), 
    use_simulation(true), 
    use_random_pose(false), 
    do_nothing(false), 
    ee_offset_x(0.0), 
    ee_offset_y(0.0), 
    ee_offset_z(0.5), 
    min_ee_height(0.95), 
    jiggle_num(2), 
    max_sleep_time(180), 
    gripper_position(50), 
    initial_joint_value(7, 0.0), 
    tf_listener(tf_buffer)
{
    this->set_target_srv = this->nh.advertiseService("set_target", &KUKAController::setTarget, this);
    this->check_object_srv = this->nh.advertiseService("check_object", &KUKAController::checkObject, this);
    this->reset_camera = this->nh.serviceClient<std_srvs::Empty>("reset_status");
    this->iiwa_pub = this->nh.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1);
    ROS_INFO("Waiting for gripper action server");
    this->gripper_action_client.waitForServer(ros::Duration(60.0));
    ROS_INFO("Started gripper action server");
    this->private_nh.getParam("initial_joint_value", this->initial_joint_value);
    this->private_nh.getParam("move_group", this->movegroup_name);
    this->private_nh.getParam("ee_link_name", this->ee_link_name);
    this->private_nh.getParam("use_simulation", this->use_simulation);
    this->private_nh.getParam("use_random_pose", this->use_random_pose);
    this->private_nh.getParam("do_nothing", this->do_nothing);
    this->private_nh.getParam("ee_offset_x", this->ee_offset_x);
    this->private_nh.getParam("ee_offset_y", this->ee_offset_y);
    this->private_nh.getParam("ee_offset_z", this->ee_offset_z);
    this->private_nh.getParam("min_ee_height", this->min_ee_height);
    this->private_nh.getParam("jiggle_num", this->jiggle_num);
    this->private_nh.getParam("max_sleep_time", this->max_sleep_time);
    this->private_nh.getParam("gripper_position", this->gripper_position);
    this->private_nh.getParam("ros_rate", this->ros_rate);
    this->loop_rate.reset(new ros::Rate(ros_rate));

    this->my_iiwa.init();
}

KUKAController::~KUKAController() 
{
}

bool KUKAController::goToInitialPose(moveit::planning_interface::MoveGroupInterface &group, 
                                     moveit::planning_interface::MoveGroupInterface::Plan &plan) 
{
    bool success_plan, motion_done;

    group.setStartStateToCurrentState();
    group.setJointValueTarget(this->initial_joint_value);
    success_plan = static_cast<bool>(group.plan(plan));

    if (!success_plan) 
    {
        ROS_WARN("Motion planning failed...");
        return success_plan;
    }

    motion_done = static_cast<bool>(group.execute(plan));
    
    if (!motion_done) 
    {
        ROS_WARN("Moving failed...");
        return motion_done;
    }

    return success_plan & motion_done;
}

bool KUKAController::goToPose(moveit::planning_interface::MoveGroupInterface &group, 
                              moveit::planning_interface::MoveGroupInterface::Plan &plan, 
                              const geometry_msgs::PoseStamped &pose)
{
    bool success_plan, motion_done;

    group.setStartStateToCurrentState();
    group.setPoseTarget(pose, this->ee_link_name);
    success_plan = static_cast<bool>(group.plan(plan));

    if (!success_plan) 
    {
        ROS_WARN("Motion planning failed...");
        return success_plan;
    }

    motion_done = static_cast<bool>(group.execute(plan));
    
    if (!motion_done) 
    {
        ROS_WARN("Moving failed...");
        return motion_done;
    }

    return success_plan & motion_done;
}

bool KUKAController::goToPoses(moveit::planning_interface::MoveGroupInterface &group, 
                               moveit::planning_interface::MoveGroupInterface::Plan &plan, 
                               const std::vector<geometry_msgs::Pose> &poses)
{
    bool success_plan, motion_done;
    moveit_msgs::RobotTrajectory trajectory_msg;
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;

    group.setStartStateToCurrentState();
    double fraction = group.computeCartesianPath(poses, eef_step, jump_threshold, trajectory_msg);
    plan.trajectory_ = trajectory_msg;
    success_plan = fraction < 1 ? false : true;

    if (!success_plan) 
    {
        ROS_WARN("Motion planning failed...");
        return success_plan;
    }

    motion_done = static_cast<bool>(group.execute(plan));
    
    if (!motion_done) 
    {
        ROS_WARN("Moving failed...");
        return motion_done;
    }

    return success_plan & motion_done;
}

bool KUKAController::rotateFlange(moveit::planning_interface::MoveGroupInterface &group, 
                                  moveit::planning_interface::MoveGroupInterface::Plan &plan, 
                                  const double a7_rad) 
{
    bool success_plan, motion_done;

    std::vector<double> joint_value;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), joint_value);
    joint_value[6] = a7_rad;

    group.setStartStateToCurrentState();
    group.setJointValueTarget(joint_value);
    success_plan = static_cast<bool>(group.plan(plan));

    if (!success_plan) 
    {
        ROS_WARN("Motion planning failed...");
        return success_plan;
    }

    motion_done = static_cast<bool>(group.execute(plan));
    
    if (!motion_done) 
    {
        ROS_WARN("Moving failed...");
        return motion_done;
    }

    return success_plan & motion_done;
}

void KUKAController::getWaypointsToGoal(const geometry_msgs::PoseStamped &start, 
                                        const geometry_msgs::PoseStamped &goal, 
                                        std::vector<geometry_msgs::Pose> &waypoints)
{
    geometry_msgs::PoseStamped command_cartesian_position;
    command_cartesian_position = start;
    command_cartesian_position.pose.position.x = goal.pose.position.x;
    command_cartesian_position.pose.position.y = goal.pose.position.y;
    waypoints.push_back(command_cartesian_position.pose);
    command_cartesian_position.pose.position.z = goal.pose.position.z;
    waypoints.push_back(command_cartesian_position.pose);

    return;
}

void KUKAController::jiggle(const geometry_msgs::PoseStamped &move_origin, 
                            const int updown_num, 
                            std::vector<geometry_msgs::Pose> &waypoints)
{
    geometry_msgs::PoseStamped command_cartesian_position;
    command_cartesian_position = move_origin;
    waypoints.push_back(command_cartesian_position.pose);
    for (int i = 0; i < updown_num; ++i) 
    {
        command_cartesian_position.pose.position.y -= 0.05;
        waypoints.push_back(command_cartesian_position.pose);
        command_cartesian_position.pose.position.y += 0.05;
        waypoints.push_back(command_cartesian_position.pose);
        command_cartesian_position.pose.position.y += 0.05;
        waypoints.push_back(command_cartesian_position.pose);
        command_cartesian_position.pose.position.y -= 0.05;
        waypoints.push_back(command_cartesian_position.pose);
    }

    return;
}

bool KUKAController::setTarget(iiwa_controller::SetTarget::Request &req, 
                               iiwa_controller::SetTarget::Response &res) 
{
    std::unique_lock<std::mutex> lock(this->target_mutex);

    res.success = false;
    bool is_new = true;
    for (auto &target_object : this->target_objects) 
    {
        is_new |= (target_object.first != req.object_name);
    }

    try 
    {
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped = this->tf_buffer.lookupTransform("world", req.object_name, ros::Time(0));
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s", ex.what());
        return true;
    }

    if (is_new) 
    {
        this->target_objects.push_back(std::pair<std::string, bool>(req.object_name, false));
        res.success = true;
    }

    return true;
}

bool KUKAController::graspObject()
{
    control_msgs::GripperCommandGoal goal;
    // initial position - 0..100, where 0 is fully closed, 100 is fully open.
    // grasping effort  - 0..100, where 100 is the maximum speed/force.
    goal.command.position = this->gripper_position;
    goal.command.max_effort = 100.0;
    this->gripper_action_client.sendGoal(goal);
    bool result = this->gripper_action_client.waitForResult(ros::Duration(60.0));

    if (result) 
    {
        actionlib::SimpleClientGoalState state = this->gripper_action_client.getState();
        ROS_INFO("Gripper action finished: %s", state.toString().c_str());
    }
    else 
    {
        ROS_WARN("Gripper action did not finish before the time out.");
    }
    return result;
}

bool KUKAController::graspObject(double &position)
{
    control_msgs::GripperCommandGoal goal;
    control_msgs::GripperCommandResult command_result;
    // initial position - 0..100, where 0 is fully closed, 100 is fully open.
    // grasping effort  - 0..100, where 100 is the maximum speed/force.
    goal.command.position = this->gripper_position;
    goal.command.max_effort = 100.0;
    this->gripper_action_client.sendGoal(goal);
    bool result = this->gripper_action_client.waitForResult(ros::Duration(60.0));

    if (result) 
    {
        command_result = *(this->gripper_action_client.getResult());
        position = command_result.position;
        ROS_INFO_STREAM("Gripper was reached to : " << position);
    }
    else 
    {
        ROS_WARN("Gripper action did not finish before the time out.");
        position = 0;
    }
    return result;
}

bool KUKAController::forceGraspObject(const double position)
{
    control_msgs::GripperCommandGoal goal;
    goal.command.position = position;
    goal.command.max_effort = 0.0;
    this->gripper_action_client.sendGoal(goal);
    bool result = this->gripper_action_client.waitForResult(ros::Duration(5.0));

    if (result) 
    {
        actionlib::SimpleClientGoalState state = this->gripper_action_client.getState();
        ROS_INFO("Gripper action finished: %s", state.toString().c_str());
    }
    else 
    {
        ROS_WARN("Gripper action did not finish before the time out.");
    }
    return result;
}

bool KUKAController::releaseObject()
{
    control_msgs::GripperCommandGoal goal;
    // initial position - 0..100, where 0 is fully closed, 100 is fully open.
    // grasping effort  - 1..100, where 100 is the maximum speed/force, and 0 means without sensing
    goal.command.position = this->gripper_position;
    goal.command.max_effort = 0.0;
    this->gripper_action_client.sendGoal(goal);
    bool result = this->gripper_action_client.waitForResult(ros::Duration(5.0));

    if (result) 
    {
        actionlib::SimpleClientGoalState state = this->gripper_action_client.getState();
        ROS_INFO("Gripper action finished: %s", state.toString().c_str());
    }
    else 
    {
        ROS_WARN("Gripper action did not finish before the time out.");
    }
    return result;
}

bool KUKAController::checkObject(std_srvs::Empty::Request &req, 
                                 std_srvs::Empty::Response &res)
{
    this->graspObject();

    geometry_msgs::PoseStamped::ConstPtr initial_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/iiwa/state/CartesianPose", this->nh);
    geometry_msgs::PoseStamped pose_stmp;

    for (int i = 0; i < 6; ++i) 
    {
        if (i == 5) 
        {
            try 
            {
                geometry_msgs::PoseStamped prev_pose_stmp;
                geometry_msgs::TransformStamped tf_stmp;
                prev_pose_stmp = pose_stmp;
                tf_stmp = this->tf_buffer.lookupTransform(initial_pose->header.frame_id, "ee_goal", ros::Time(0));
                pose_stmp.pose.position.x = tf_stmp.transform.translation.x;
                pose_stmp.pose.position.y = tf_stmp.transform.translation.y;
                pose_stmp.pose.position.z = tf_stmp.transform.translation.z;
                pose_stmp.pose.orientation = tf_stmp.transform.rotation;
                pose_stmp.header.stamp = ros::Time();

                this->iiwa_pub.publish(pose_stmp);

                ros::Duration(10.0).sleep();
                ros::spinOnce();

                this->iiwa_pub.publish(prev_pose_stmp);

                ros::Duration(5.0).sleep();
                ros::spinOnce();
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s", ex.what());
            }
        }

        pose_stmp.pose.position.x = initial_pose->pose.position.x;
        pose_stmp.pose.position.y = initial_pose->pose.position.y;
        pose_stmp.pose.position.z = initial_pose->pose.position.z + 0.05 - (0.05 * (i % 2));
        pose_stmp.pose.orientation = initial_pose->pose.orientation;
        pose_stmp.header.stamp = ros::Time();
        this->iiwa_pub.publish(pose_stmp);

        ros::Duration(3.0).sleep();
        ros::spinOnce();
    }

    this->releaseObject();
    return true;
}

void KUKAController::mainLoop()
{
    OperationState iiwa_state = OperationState::Unactiveted;

    if (this->do_nothing) 
    {
        iiwa_state = OperationState::DoNothing;
    }

    // Create MoveGroup
    moveit::planning_interface::MoveGroupInterface group(this->movegroup_name);
    moveit::planning_interface::MoveGroupInterface::Plan myplan;
    // Configure planner 
    group.setPlanningTime(10.0);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setEndEffectorLink(this->ee_link_name);

    geometry_msgs::PoseStamped initial_pose;
    geometry_msgs::PoseStamped object_pose;
    bool motion_done = false;

    bool is_first = true;
    ros::Time start_time;
    ros::Duration duration;

    bool is_jammed = false;
    double current_position = this->gripper_position;

    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::uniform_real_distribution<> rad_rand(this->initial_joint_value[6], this->initial_joint_value[6] + M_PI);

    while (ros::ok()) 
    {
        if (!this->use_simulation && !this->my_iiwa.getRobotIsConnected()) 
        {
            ROS_WARN("Robot is not connected...");
            this->loop_rate->sleep();
            continue;
        }
        
        switch (iiwa_state)
        {
            case OperationState::Unactiveted:
            {
                ROS_INFO("Go to initial pose");

                motion_done = this->goToInitialPose(group, myplan);
                initial_pose = group.getCurrentPose(this->ee_link_name);

                ros::Duration(1.0).sleep();

                if (!motion_done) 
                {
                    break;
                }

                motion_done = this->releaseObject();
                current_position = this->gripper_position;

                if (!motion_done) 
                {
                    break;
                }

                std_srvs::Empty srv;
                this->reset_camera.call(srv);

                iiwa_state = OperationState::Initialized;

                break;
            }
            case OperationState::Initialized:
            {
                ROS_INFO("Look forward to grasp object :)");

                std::unique_lock<std::mutex> lock(this->target_mutex);

                for (auto &target_object : this->target_objects) 
                {
                    if (target_object.second) 
                    {
                        continue;
                    }

                    geometry_msgs::TransformStamped transform_stamped;
                    try 
                    {
                        transform_stamped = this->tf_buffer.lookupTransform("world", target_object.first, ros::Time(0));
                        object_pose.pose.position.x = transform_stamped.transform.translation.x + this->ee_offset_x;
                        object_pose.pose.position.y = transform_stamped.transform.translation.y + this->ee_offset_y;
                        object_pose.pose.position.z = transform_stamped.transform.translation.z + this->ee_offset_z;
                        object_pose.pose.orientation = transform_stamped.transform.rotation;

                        if (this->min_ee_height > object_pose.pose.position.z) 
                        {
                            object_pose.pose.position.z = this->min_ee_height;
                        }

                        target_object.second = true;
                        iiwa_state = OperationState::Selected;
                    }
                    catch (tf2::TransformException &ex) 
                    {
                        ROS_WARN("%s", ex.what());
                    }

                    break;
                }

                if (is_first) 
                {
                    start_time = ros::Time::now();
                    is_first = false;
                }

                duration = ros::Time::now() - start_time;
                if (duration.toSec() > this->max_sleep_time) 
                {
                    iiwa_state = OperationState::Unactiveted;
                    is_first = true;
                }

                break;
            }
            case OperationState::Selected:
            {
                ROS_INFO("Move to object position");

                geometry_msgs::PoseStamped start_pose;
                double rot = this->initial_joint_value[6];

                if (this->use_random_pose) 
                {
                    rot = rad_rand(mt);
                }
                else 
                {
                    tf::Quaternion q(object_pose.pose.orientation.x, 
                                     object_pose.pose.orientation.y, 
                                     object_pose.pose.orientation.z, 
                                     object_pose.pose.orientation.w);
                    double roll, pitch, yaw;
                    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                    yaw = yaw >  M_PI / 2.0 ? yaw - M_PI //                   90 < yaw
                        : yaw < -M_PI / 2.0 ? yaw + M_PI // yaw < -90
                                            : yaw;       //       -90 < yaw < 90
                    rot += yaw > 0 ? M_PI - yaw : std::abs(yaw);
                }

                motion_done = this->rotateFlange(group, myplan, rot);

                if (!motion_done) 
                {
                    break;
                }

                start_pose = group.getCurrentPose(this->ee_link_name);
                std::vector<geometry_msgs::Pose> waypoints;
                this->getWaypointsToGoal(start_pose, object_pose, waypoints);
                motion_done = this->goToPoses(group, myplan, waypoints);

                ros::Duration(1.0).sleep();

                if (!motion_done) 
                {
                    break;
                }

                motion_done = this->graspObject(current_position);

                if (!motion_done) 
                {
                    break;
                }

                iiwa_state = OperationState::Picked;

                break;
            }
            case OperationState::Picked:
            {
                ROS_INFO("Check grasped object state");

                std::vector<geometry_msgs::Pose> waypoints;
                //object_pose = group.getCurrentPose(this->ee_link_name);

                if (this->jiggle_num > 0) 
                {
                    this->jiggle(initial_pose, this->jiggle_num, waypoints);
                }

                try 
                {
                    geometry_msgs::PoseStamped goal_pose;
                    geometry_msgs::TransformStamped tf_stmp;
                    tf_stmp = this->tf_buffer.lookupTransform("world", "storage_box", ros::Time(0));
                    goal_pose.pose.position.x = tf_stmp.transform.translation.x;
                    goal_pose.pose.position.y = tf_stmp.transform.translation.y;
                    goal_pose.pose.position.z = object_pose.pose.position.z;
                    goal_pose.pose.orientation = tf_stmp.transform.rotation;
                    goal_pose.header.stamp = ros::Time();

                    this->getWaypointsToGoal(initial_pose, goal_pose, waypoints);
                    motion_done = this->goToPoses(group, myplan, waypoints);
                }
                catch (tf2::TransformException &ex) 
                {
                    object_pose = group.getCurrentPose(this->ee_link_name);
                    motion_done = this->goToPoses(group, myplan, waypoints);
                    if (motion_done) 
                    {
                        ros::Duration(5.0).sleep();
                        motion_done = this->goToPose(group, myplan, object_pose);
                    }
                }

                ros::Duration(1.0).sleep();

                if (!motion_done) 
                {
                    break;
                }

                motion_done = this->releaseObject();
                current_position = this->gripper_position;

                if (!motion_done) 
                {
                    break;
                }

                iiwa_state = OperationState::Placed;

                break;
            }
            case OperationState::Placed:
            {
                ROS_INFO("Back to home position");

                geometry_msgs::PoseStamped goal_pose;
                goal_pose = group.getCurrentPose(this->ee_link_name);
                goal_pose.pose.position.z = initial_pose.pose.position.z;
                motion_done = this->goToPose(group, myplan, goal_pose);

                if (!motion_done) 
                {
                    break;
                }

                std::unique_lock<std::mutex> lock(this->target_mutex);

                bool grasped_all_object = true;
                for (auto &target_object : this->target_objects) 
                {
                    grasped_all_object &= target_object.second;
                }

                if (grasped_all_object) 
                {
                    this->target_objects.clear();
                }

                iiwa_state = OperationState::Unactiveted;

                break;
            }
            default:
            {
                // do nothing
                break;
            }
        }

        ros::spinOnce();
        this->loop_rate->sleep();
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "iiwa_controller_node");
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    KUKAController my_controller;
    my_controller.mainLoop();

    return 0;
}