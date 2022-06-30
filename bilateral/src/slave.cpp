#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "omni_msgs/OmniFeedback.h"

#include "bilateral.hpp"

void BilateralController::forceControl()
{
    geometry_msgs::Point master_pos = this->getMasterPose().position;
    geometry_msgs::Point slave_pos = this->getSlavePose().position;
    std::vector<double> ktheta = this->getPosParams();
    omni_msgs::OmniFeedback force_msg;
    // phantomの場合、forceをかける方向はencの向きと逆
    // A0Bにおいては各軸について符号あわせる
    std::array<double, 3> tauref = this->positionIIRController(master_pos, slave_pos, ktheta);
    force_msg.force.x = tauref.at(0);
    force_msg.force.y = tauref.at(1);
    force_msg.force.z = tauref.at(2);
    force_msg.position.x = 0.0;
    force_msg.position.y = 0.0;
    force_msg.position.z = 0.0;
    m_pub.publish(force_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bilateral_slave");
    sleep(5);
    ROS_INFO("Start bilateral slave node ...");
    BilateralController bilateral_controller(BilateralController::MS::Slave);
    ros::spin();
    ROS_INFO("End bilateral slave node ...");
    return EXIT_SUCCESS;
}
