#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Wrench.h>
// #include <geometry_msgs/WrenchStamped.h>
// #include <urdf/model.h>

#include <string.h>
// #include <stdio.h>
// #include <math.h>
// #include <assert.h>
// #include <sstream>

//#include "omni_msgs/OmniButtonEvent.h"
#include "omni_msgs/OmniFeedback.h"
//#include "omni_msgs/OmniState.h"

// float prev_time;
// int calibrationStyle;

class BilateralController
{
private:
    std::string omni_name_master;
    std::string omni_name_slave;

    ros::NodeHandle m_nh;
    ros::NodeHandle m_pnh;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;

public:
    BilateralController() : m_pnh("~") {
        if (!m_pnh.getParam("/omni_name_master", omni_name_master)) {
          ROS_FATAL("'omni_name_master' is not set");
        }
        if (!m_pnh.getParam("omni_name_slave", omni_name_slave)) {
          ROS_FATAL("'omni_name_slave' is not set");
        }
        m_pub = m_nh.advertise<omni_msgs::OmniFeedback>(omni_name_slave + "/force_feedback", 1);
        m_sub = m_nh.subscribe(omni_name_master + "/pose", 1, &BilateralController::force_callback, this);
    }

    void force_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
    {
        ROS_INFO("hoge");

        ///////////////////////////////////////////////////////////
        // Some people might not like this extra damping, but it
        // helps to stabilize the overall force feedback. It isn't
        // like we are getting direct impedance matching from the
        // omni anyway
        ///////////////////////////////////////////////////////////

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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bilateral_slave");
    ROS_INFO("Start bilateral slave node ...");
    BilateralController bilateral_controller;
    ros::spin();
    ROS_INFO("End bilateral slave node ...");
    return EXIT_SUCCESS;
}
