#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "omni_msgs/OmniFeedback.h"

#include "bilateral.hpp"

double BilateralController::positionController(
    double ref, double x, double k)

{
    return k * (ref - x);  // TODO: ディジタル制御器かく
}


void BilateralController::forceControl()
{
    geometry_msgs::Point master_pos = this->getMasterPose().position;
    geometry_msgs::Point slave_pos = this->getSlavePose().position;
    std::vector<double> params = this->getParams();
    omni_msgs::OmniFeedback force_msg;
    // phantomの場合、forceをかける方向はencの向きと逆
    // A0Bにおいては各軸について符号あわせる
    force_msg.force.x = this->positionController(slave_pos.x, master_pos.x, params.at(0));
    force_msg.force.y = this->positionController(slave_pos.y, master_pos.y, params.at(1));
    force_msg.force.z = this->positionController(slave_pos.z, master_pos.z, params.at(2));
    force_msg.position.x = 0.0;
    force_msg.position.y = 0.0;
    force_msg.position.z = 0.0;
    m_pub.publish(force_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bilateral_master");
    ROS_INFO("Start bilateral master node ...");
    BilateralController bilateral_controller(BilateralController::MS::Master);
    ros::spin();
    ROS_INFO("End bilateral master node ...");
    return EXIT_SUCCESS;
}
