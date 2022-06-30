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
    std::string m_topic_name_master;
    std::string m_topic_name_slave;
    std::vector<double> m_ktheta_list;
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
    // ref: reference
    // th: controlled obj.
    //
    std::array<double, 3> positionIIRController(
        geometry_msgs::Point& ref, geometry_msgs::Point& th, std::vector<double>& k)
    {
        const double a0 = 34.94;
        const double a1 = -34.89;
        const double b1 = 0.995;
        std::array<double, 3> thi;  // theta_input
        // x, y, zでしかaccessできないので仕方なく...
        thi.at(0) = ref.x - th.x;
        thi.at(1) = ref.y - th.y;
        thi.at(2) = ref.z - th.z;
        std::array<double, 3> ret;
        for (int i = 0; i < 3; i++) {
            ret.at(i) = k.at(i) * (a0 * thi.at(i) + a1 * m_th_pi.at(i))  // m_th_pi: prev_input
                        + b1 * m_th_po.at(i);                            //m_th_po: prev_output
            // update variables
            m_th_po.at(i) = ret.at(i);
            m_th_pi.at(i) = thi.at(i);
        }
        return ret;
    }

    // IIR (大嘘)
    std::array<double, 3> forceIIRController(
        geometry_msgs::Point& master, geometry_msgs::Point& slave)  //, std::vector<double>& k)
    {
        static int p_cnt = 0;
        static int m_cnt = 0;
        const double theta_threshold = 0.05;
        const int time_threshold_ms = 100;
        ROS_INFO("diff: %lf", master.x - slave.x);
        if (master.x - slave.x > theta_threshold) {
            p_cnt++;
            m_cnt = 0;
        } else if (master.x - slave.x < -theta_threshold) {
            p_cnt = 0;
            m_cnt++;
        } else {
            p_cnt = 0;
            m_cnt = 0;
        }
        ROS_INFO("p: %d, m: %d", p_cnt, m_cnt);

        const double f = 1.;
        if (p_cnt > time_threshold_ms) {
            return std::array<double, 3>{f, 0.0, 0.0};
        } else if (m_cnt > time_threshold_ms) {
            return std::array<double, 3>{-f, 0.0, 0.0};
        } else {
            return std::array<double, 3>{0.0, 0.0, 0.0};
        }
    }

public:
    BilateralController(BilateralController::MS ms) : m_master_or_slave(ms), m_pnh("~")
    {
        if (!m_pnh.getParam("/topic_master", m_topic_name_master)) {
            ROS_FATAL("'topic_master' is not set");
        } else {
            ROS_INFO("topic_master: %s", m_topic_name_master.c_str());
        }
        if (!m_pnh.getParam("/topic_slave", m_topic_name_slave)) {
            ROS_FATAL("'topic_slave' is not set");
        } else {
            ROS_INFO("topic_slave: %s", m_topic_name_slave.c_str());
        }
        if (!m_pnh.getParam("ktheta_list", m_ktheta_list)) {
            ROS_FATAL("'ktheta_list' is not set");
        } else {
            ROS_INFO("ktheta_list: [%lf, %lf, %lf]", m_ktheta_list.at(0), m_ktheta_list.at(1), m_ktheta_list.at(2));
        }
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
    std::vector<double>& getPosGains() { return m_ktheta_list; }
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
