#include <csignal>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

extern "C" void signal_handler(int sig) {
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sendmotion_loop_lin");
    ros::NodeHandle nh;

    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGHUP, signal_handler);

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPoseLin", 1);

    geometry_msgs::PoseStamped my_cartesian_pose;
    my_cartesian_pose.header.frame_id = "iiwa_link_0";
    my_cartesian_pose.pose.position.x = 0.7;
    my_cartesian_pose.pose.position.y = 0.0;
    my_cartesian_pose.pose.position.z = 0.23;
    my_cartesian_pose.pose.orientation.x = 0.0;
    my_cartesian_pose.pose.orientation.y = 1.0;
    my_cartesian_pose.pose.orientation.z = 0.0;
    my_cartesian_pose.pose.orientation.w = 0.0;

    ros::Rate loop_rate(0.5);
    int count = 0;

    while (ros::ok()) {
        my_cartesian_pose.pose.position.x = count % 2 == 0 ? 0.7 : 0.55;

        pose_pub.publish(my_cartesian_pose);
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}