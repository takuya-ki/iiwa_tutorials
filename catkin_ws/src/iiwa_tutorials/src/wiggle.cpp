#include <csignal>
#include <cmath>
#include <ros/ros.h>
#include <iiwa_msgs/JointPosition.h>

extern "C" void signal_handler(int sig) {
    ros::shutdown();
}

int main(int argc, char **argv) {
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGHUP, signal_handler);

    ros::init(argc, argv, "wiggle");

    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1);

    iiwa_msgs::JointPosition my_joint_position;

    ros::Rate loop_rate(10);
    int count = 0;
    const double speed = 0.1;

    while (ros::ok()) {
        ROS_INFO("count %d", count);

        my_joint_position.position.a1 = 0.3 * std::sin(count * speed * 1.0);
        my_joint_position.position.a2 = 0.3 * std::sin(count * speed * 1.1);
        my_joint_position.position.a3 = 0.3 * std::sin(count * speed * 1.2);
        my_joint_position.position.a4 = 0.3 * std::sin(count * speed * 1.3);
        my_joint_position.position.a5 = 0.3 * std::sin(count * speed * 1.4);
        my_joint_position.position.a6 = 0.3 * std::sin(count * speed * 1.5);
        my_joint_position.position.a7 = 0.3 * std::sin(count * speed * 1.6);

        joint_pub.publish(my_joint_position);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}