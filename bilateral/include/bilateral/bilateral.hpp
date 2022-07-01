#pragma once

#include <array>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "omni_msgs/OmniFeedback.h"

template <typename T, std::size_t N>
std::array<T, N> operator+(const std::array<T, N>& a, const std::array<T, N>& b) noexcept
{
    std::array<T, N> ret;
    for (std::size_t i = 0; i < N; i++) {
        ret.at(i) = a.at(i) + b.at(i);
    }
    return ret;
}
template <typename T, std::size_t N>
std::array<T, N> operator-(const std::array<T, N>& a, const std::array<T, N>& b) noexcept
{
    std::array<T, N> ret;
    for (std::size_t i = 0; i < N; i++) {
        ret.at(i) = a.at(i) - b.at(i);
    }
    return ret;
}
template <typename T, std::size_t N>
std::array<T, N> operator*(const double a, const std::array<T, N>& b) noexcept
{
    std::array<T, N> ret;
    for (std::size_t i = 0; i < N; i++) {
        ret.at(i) = a * b.at(i);
    }
    return ret;
}
template <typename T, std::size_t N>
std::array<T, N> operator-(const std::array<T, N>& a) noexcept
{
    std::array<T, N> ret;
    for (std::size_t i = 0; i < N; i++) {
        ret.at(i) = -a.at(i);
    }
    return ret;
}

class BilateralController
{
public:
    enum class MS
    {
        Master,
        Slave
    };

private:
    std::string m_topic_name_master;
    std::string m_topic_name_slave;
    std::vector<double> m_joint_gain_list;
    std::vector<double> m_position_scale_gain;
    std::vector<double> m_force_scale_gain;
    BilateralController::MS m_master_or_slave;

    ros::NodeHandle m_nh;
    ros::NodeHandle m_pnh;
    ros::Publisher m_pub;
    ros::Subscriber m_sub_master;
    ros::Subscriber m_sub_slave;

    geometry_msgs::Pose m_master_pose;
    geometry_msgs::Pose m_slave_pose;

    std::array<double, 3> m_th_pi;  // prev_input
    std::array<double, 3> m_th_po;  // prev_output

    // 位置にもとづくディジタル制御器
    // tustin変換 (双一次z変換) によりIIR型フィルタとして構成している
    std::array<double, 3> positionIIRController(
        geometry_msgs::Point& master, geometry_msgs::Point& slave, std::vector<double>& joint_gain)
    {
        const double a0 = 157.8;
        const double a1 = -157.7;
        const double b1 = 0.9704;
        std::array<double, 3> pos_feedback_diff;  // theta_input_diff
        // x, y, zでしかaccessできないので仕方なく...
        pos_feedback_diff.at(0) = master.x - this->m_position_scale_gain.at(0) * slave.x;
        pos_feedback_diff.at(1) = master.y - this->m_position_scale_gain.at(1) * slave.y;
        pos_feedback_diff.at(2) = master.z - this->m_position_scale_gain.at(2) * slave.z;
        std::array<double, 3> ret;
        for (int i = 0; i < 3; i++) {
            ret.at(i) = joint_gain.at(i) * (a0 * pos_feedback_diff.at(i) + a1 * m_th_pi.at(i))  // m_th_pi: prev_input
                        + b1 * m_th_po.at(i);                                                   //m_th_po: prev_output
            // update variables
            m_th_po.at(i) = ret.at(i);
            m_th_pi.at(i) = pos_feedback_diff.at(i);
        }
        return ret;
    }

    // TODO: 今はとりあえず定数だが、モータパラメータを使ってDOB、RFOBを構成する
    std::array<double, 3> forceIIRController(
        geometry_msgs::Point& master, geometry_msgs::Point& slave)  //, std::vector<double>& k)
    {
        static int cnt = 0;
        constexpr double theta_threshold = 0.05;
        constexpr int time_threshold_ms = 100;
        const double pos_feedback_diff = master.x - this->m_position_scale_gain.at(0) * slave.x;
        ROS_INFO("pos_feedback_diff: %lf", pos_feedback_diff);
        if (std::abs(pos_feedback_diff) > theta_threshold) {
            cnt++;
        } else {
            cnt = 0;
        }
        ROS_INFO("cnt: %d", cnt);

        const double f = 1.;
        if (cnt > time_threshold_ms) {
            return std::array<double, 3>{(pos_feedback_diff > 0.0 ? -1.0 : 1.0) * f, 0.0, 0.0};
        } else {
            return std::array<double, 3>{0.0, 0.0, 0.0};
        }
    }

public:
    BilateralController(BilateralController::MS ms) : m_master_or_slave(ms), m_pnh("~")
    {
        // get parameters
        if (!m_pnh.getParam("/topic_master", m_topic_name_master)) {
            ROS_FATAL("'topic_master' is not set");
        }
        ROS_INFO("topic_master: %s", m_topic_name_master.c_str());
        if (!m_pnh.getParam("/topic_slave", m_topic_name_slave)) {
            ROS_FATAL("'topic_slave' is not set");
        }
        ROS_INFO("topic_slave: %s", m_topic_name_slave.c_str());
        if (!m_pnh.getParam("joint_gain_list", m_joint_gain_list)) {
            ROS_FATAL("'joint_gain_list' is not set");
        }
        ROS_INFO("joint_gain_list: [%lf, %lf, %lf]", m_joint_gain_list.at(0), m_joint_gain_list.at(1), m_joint_gain_list.at(2));
        if (!m_pnh.getParam("/position_scale_gain", m_position_scale_gain)) {
            ROS_FATAL("'position_scale_gain' is not set");
        }
        ROS_INFO("position_scale_gain: [%lf, %lf, %lf]", m_position_scale_gain.at(0), m_position_scale_gain.at(1), m_position_scale_gain.at(2));
        if (!m_pnh.getParam("/force_scale_gain", m_force_scale_gain)) {
            ROS_FATAL("'force_scale_gain' is not set");
        }
        ROS_INFO("force_scale_gain: [%lf, %lf, %lf]", m_force_scale_gain.at(0), m_force_scale_gain.at(1), m_force_scale_gain.at(2));

        // set publisher/subscriber
        if (m_master_or_slave == BilateralController::MS::Master) {
            m_pub = m_nh.advertise<omni_msgs::OmniFeedback>(m_topic_name_master + "/force_feedback", 1);
        } else {
            m_pub = m_nh.advertise<omni_msgs::OmniFeedback>(m_topic_name_slave + "/force_feedback", 1);
        }
        m_sub_master = m_nh.subscribe(m_topic_name_master + "/pose", 1, &BilateralController::masterCallback, this);
        m_sub_slave = m_nh.subscribe(m_topic_name_slave + "/pose", 1, &BilateralController::slaveCallback, this);
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
    std::vector<double>& getPosGains() { return m_joint_gain_list; }
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
