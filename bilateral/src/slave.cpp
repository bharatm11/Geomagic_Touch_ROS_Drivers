#include "bilateral.hpp"

std::array<double, 3> BilateralController::positionController(
    geometry_msgs::Point& ref, geometry_msgs::Point& th, std::vector<double>& k)

{
    const double a0 = 1121;
    const double a1 = -1115;
    const double b1 = 0.9876;
    std::array<double, 3> thi;
    thi.at(0) = ref.x - th.x;
    thi.at(1) = ref.y - th.y;
    thi.at(2) = ref.z - th.z;
    std::array<double, 3> ret;
    for (int i = 0; i < 3; i++) {
        ret.at(i) = k.at(i) * (a0 * thi.at(i) + a1 * m_th_pi.at(i) + b1 * m_th_po.at(i));
        m_th_po.at(i) = ret.at(i);
        m_th_pi.at(i) = thi.at(i);
    }
    return ret;
}


void BilateralController::forceControl()
{
    geometry_msgs::Point master_pos = this->getMasterPose().position;
    geometry_msgs::Point slave_pos = this->getSlavePose().position;
    std::vector<double> params = this->getParams();
    omni_msgs::OmniFeedback force_msg;
    // phantomの場合、forceをかける方向はencの向きと逆
    // A0Bにおいては各軸について符号あわせる
    std::array<double, 3> tauref = this->positionController(master_pos, slave_pos, params);
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
