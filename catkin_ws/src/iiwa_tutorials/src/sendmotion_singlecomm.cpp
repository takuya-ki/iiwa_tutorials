#include <ros/ros.h>
#include <iiwa_msgs/JointPosition.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "sendmotion");

    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1);

    iiwa_msgs::JointPosition my_joint_position;
    my_joint_position.position.a1 = 0.0;
    my_joint_position.position.a2 = 1.044;
    my_joint_position.position.a3 = 0.0;
    my_joint_position.position.a4 = -1.096;
    my_joint_position.position.a5 = 0.0;
    my_joint_position.position.a6 = 1.002;
    my_joint_position.position.a7 = 0.0;

    // wait until the connection is completed
    ROS_INFO("wait until the connection is completed");
    ros::Rate loop_rate(10);
    while (joint_pub.getNumSubscribers() == 0) {
        loop_rate.sleep();
    }

    // once connection is ok, send the command
    ROS_INFO("publishing single command");
    if (ros::ok()) {
        joint_pub.publish(my_joint_position);
        ros::spinOnce();
    }
    else {
        ROS_INFO("ros not ok");
    }

    ROS_INFO("done");

    return 0;
}