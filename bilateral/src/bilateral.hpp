#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "omni_msgs/OmniFeedback.h"

#include <array>
#include <string>

class BilateralController
{
public:
    enum class MS
    {
        Master,
        Slave
    };

private:
    std::string topic_name_master;
    std::string topic_name_slave;
    std::vector<double> ktheta_list;
    BilateralController::MS master_or_slave;

    ros::NodeHandle m_nh;
    ros::NodeHandle m_pnh;
    ros::Publisher m_pub;
    ros::Subscriber m_sub_master;
    ros::Subscriber m_sub_slave;

    geometry_msgs::Pose m_master_pose;
    geometry_msgs::Pose m_slave_pose;

    std::array<double, 3> positionController(geometry_msgs::Point& ref, geometry_msgs::Point& th, std::vector<double>& k);
    std::array<double, 3> m_th_pi;  // prev_input
    std::array<double, 3> m_th_po;  // prev_output

public:
    BilateralController(BilateralController::MS ms) : master_or_slave(ms), m_pnh("~")
    {
        if (!m_pnh.getParam("/topic_master", topic_name_master)) {
            ROS_FATAL("'topic_master' is not set");
        } else {
            ROS_INFO("topic_master: %s", topic_name_master.c_str());
        }
        if (!m_pnh.getParam("/topic_slave", topic_name_slave)) {
            ROS_FATAL("'topic_slave' is not set");
        } else {
            ROS_INFO("topic_slave: %s", topic_name_slave.c_str());
        }
        if (!m_pnh.getParam("ktheta_list", ktheta_list)) {
            ROS_FATAL("'ktheta_list' is not set");
        } else {
            ROS_INFO("ktheta_list: [%lf, %lf, %lf]", ktheta_list.at(0), ktheta_list.at(1), ktheta_list.at(2));
        }
        if (master_or_slave == BilateralController::MS::Master) {
            m_pub = m_nh.advertise<omni_msgs::OmniFeedback>(topic_name_master + "/force_feedback", 1);
        } else {
            m_pub = m_nh.advertise<omni_msgs::OmniFeedback>(topic_name_slave + "/force_feedback", 1);
        }
        m_sub_master = m_nh.subscribe(topic_name_master + "/pose", 1, &BilateralController::masterCallback, this);
        m_sub_slave = m_nh.subscribe(topic_name_slave + "/pose", 1, &BilateralController::slaveCallback, this);
    }

    void updateMasterPose(const geometry_msgs::Pose& pose)
    {
        m_master_pose = pose;
    }
    void updateSlavePose(const geometry_msgs::Pose& pose)
    {
        m_slave_pose = pose;
    }
    geometry_msgs::Pose& getMasterPose() { return m_master_pose; }
    geometry_msgs::Pose& getSlavePose() { return m_slave_pose; }
    std::vector<double>& getParams() { return ktheta_list; }
    void forceControl();
    void masterCallback(const geometry_msgs::PoseStamped::ConstPtr& master_pose)
    {
        this->updateMasterPose(master_pose->pose);
        // 1kHzにするためにslaveからsubしたときのみcontrol
        // this->forceControl();
    }
    void slaveCallback(const geometry_msgs::PoseStamped::ConstPtr& slave_pose)
    {
        this->updateSlavePose(slave_pose->pose);
        this->forceControl();
    }
};
