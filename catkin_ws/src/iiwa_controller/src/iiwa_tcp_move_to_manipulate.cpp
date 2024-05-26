#include "ros/ros.h"
#include <iiwa_ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "iiwa_controller/SetTargetPose.h"

using moveit::planning_interface::MoveItErrorCode;


bool tcp_move_to_manipulate(iiwa_controller::SetTargetPose::Request &req,
                      iiwa_controller::SetTargetPose::Response &res) {
  ros::NodeHandle nh("~");

  iiwa_ros::iiwaRos my_iiwa;
  my_iiwa.init();

  std::string movegroup_name, ee_link;
  geometry_msgs::PoseStamped command_cartesian_position;
  
  // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  nh.param<std::string>("move_group", movegroup_name, "manipulator");
  nh.param<std::string>("ee_link", ee_link, "tool_link_ee");
  
  // Dynamic parameter to choose the rate at wich this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.1); // 0.1 Hz = 10 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);
  
  // Create MoveGroup
  moveit::planning_interface::MoveGroupInterface group(movegroup_name);
  moveit::planning_interface::MoveGroupInterface::Plan myplan;

  // Configure planner 
  group.setPlanningTime(0.5);
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setEndEffectorLink(ee_link);
  MoveItErrorCode motion_done = MoveItErrorCode::FAILURE;

  while (ros::ok()) {
    if (my_iiwa.getRobotIsConnected()) {
 
      group.setStartState(*group.getCurrentState());

      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back(req.target_pose);  // front

      group.setPoseReferenceFrame(ee_link); // added
      ROS_INFO_STREAM("Planning to move " 
                  << group.getEndEffectorLink() 
                  << " to a target pose expressed in " 
                  << group.getPlanningFrame()
                  << ", and the pose reference frame is in " 
                  << group.getPoseReferenceFrame());

      group.setStartStateToCurrentState();

      moveit_msgs::RobotTrajectory trajectory;
      double fraction = group.computeCartesianPath(waypoints,
                                                   0.01,  // eef_step
                                                   0.0,   // jump_threshold
                                                   trajectory);

      myplan.trajectory_ = trajectory;

      ROS_INFO_STREAM("Planning to move " << group.getEndEffectorLink());
      ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
            fraction * 100.0);
      /* Sleep to give Rviz time to visualize the plan. */
      // sleep(5.0);

      motion_done = group.execute(myplan);
      ros::spinOnce();
      loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
      res.success = true;
      break;
      // if (motion_done == MoveItErrorCode::SUCCESS) {
      //   direction *= -1; // In the next iteration the motion will be on the opposite direction
      //   loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
      // }
    }
    else {
      ROS_WARN_STREAM("Robot is not connected...");
      ros::Duration(5.0).sleep(); // 5 seconds
    }
  }

  return true;
}

int main (int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "iiwa_tcp_move_to_manipulate");
  ros::NodeHandle nh("~");
  
  ros::ServiceServer service = nh.advertiseService("tcp_move_to_manipulate", tcp_move_to_manipulate);
  ROS_INFO("Ready for tcp move.");

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
}; 
