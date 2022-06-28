#include "slave.hpp"

void BilateralController::force_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    double master_x = pose->pose.position.x;
    // double master_y = pose->pose.position.y;
    // double master_z = pose->pose.position.z;

    omni_msgs::OmniFeedback force_msg;
    force_msg.force.x = master_x * (-5.);
    force_msg.force.y = 0.0;
    force_msg.force.z = 0.0;
    force_msg.position.x = 0.0;
    force_msg.position.y = 0.0;
    force_msg.position.z = 0.0;
    m_pub.publish(force_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bilateral_slave");
    ROS_INFO("Start bilateral slave node ...");
    BilateralController bilateral_controller;
    ros::spin();
    ROS_INFO("End bilateral slave node ...");
    return EXIT_SUCCESS;
}
