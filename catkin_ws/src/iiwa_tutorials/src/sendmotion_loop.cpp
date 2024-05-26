#include <csignal>
#include <ros/ros.h>
#include <iiwa_msgs/JointPosition.h>

extern "C" void signal_handler(int sig) {
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sendmotion_loop");
    ros::NodeHandle nh;

    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGHUP, signal_handler);

    ros::Publisher joint_pub = nh.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1);

    iiwa_msgs::JointPosition my_joint_position1;
    my_joint_position1.position.a1 = 0.0;
    my_joint_position1.position.a2 = 1.044;
    my_joint_position1.position.a3 = 0.0;
    my_joint_position1.position.a4 = -1.096;
    my_joint_position1.position.a5 = 0.0;
    my_joint_position1.position.a6 = 1.002;
    my_joint_position1.position.a7 = 0.0;

    iiwa_msgs::JointPosition my_joint_position2;
    my_joint_position2.position.a1 = 0.0;
    my_joint_position2.position.a2 = 0.769;
    my_joint_position2.position.a3 = 0.0;
    my_joint_position2.position.a4 = -1.672;
    my_joint_position2.position.a5 = 0.0;
    my_joint_position2.position.a6 = 0.701;
    my_joint_position2.position.a7 = 0.0;

    ros::Rate loop_rate(0.5);
    int count = 0;

    while (ros::ok()) {
        // Prepare next pose
        const iiwa_msgs::JointPosition& next_joint_position = count % 2 == 0 ? my_joint_position1 : my_joint_position2;

        joint_pub.publish(next_joint_position);
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}