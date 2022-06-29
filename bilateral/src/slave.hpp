#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include "omni_msgs/OmniFeedback.h"

class BilateralController
{
private:
    std::string m_omni_name_master;
    std::string m_omni_name_slave;

    ros::NodeHandle m_nh;
    ros::NodeHandle m_pnh;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;

public:
    BilateralController() : m_pnh("~")
    {
        if (!m_pnh.getParam("/omni_name_master", m_omni_name_master)) {
            ROS_FATAL("'omni_name_master' is not set");
        }
        if (!m_pnh.getParam("omni_name_slave", m_omni_name_slave)) {
            ROS_FATAL("'omni_name_slave' is not set");
        }
        m_pub = m_nh.advertise<omni_msgs::OmniFeedback>(m_omni_name_slave + "/force_feedback", 1);
        m_sub = m_nh.subscribe(m_omni_name_master + "/pose", 1, &BilateralController::force_callback, this);
    }

    void force_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
};
