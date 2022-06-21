#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

//#include "omni_msgs/OmniButtonEvent.h"
#include "omni_msgs/OmniFeedback.h"
//#include "omni_msgs/OmniState.h"
#include "geometry_msgs/PoseStamped.h"

float prev_time;
int calibrationStyle;

class BilateralController
{
private:
    ros::NodeHandle m_nh;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;

public:
    std::string omni_name;
    BilateralController()
    {
        ros::param::param(std::string("~omni_name"), omni_name, std::string("phantom"));
        std::ostringstream stream1;
        stream1 << omni_name << "/force_feedback";
        std::string state_topic_name = std::string(stream1.str());
        m_pub = m_nh.advertise<omni_msgs::OmniFeedback>(state_topic_name.c_str(), 1);

        //Subscribe to NAME/force_feedback
        std::ostringstream stream2;
        stream2 << omni_name << "/pose";
        std::string force_feedback_topic = std::string(stream2.str());
        m_sub = m_nh.subscribe(force_feedback_topic.c_str(), 1, &BilateralController::force_callback, this);
    }

    void force_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
    {
        ////////////////////Some people might not like this extra damping, but it
        ////////////////////helps to stabilize the overall force feedback. It isn't
        ////////////////////like we are getting direct impedance matching from the
        ////////////////////omni anyway

        double master_x = pose->pose.position.x;
        double master_y = pose->pose.position.y;
        double master_z = pose->pose.position.z;

        omni_msgs::OmniFeedback force_msg;
        force_msg.force.x = master_x;
        force_msg.force.y = 0.0;
        force_msg.force.z = 0.0;
        force_msg.position.x = 0.0;
        force_msg.position.y = 0.0;
        force_msg.position.z = 0.0;
    }
};

int main(int argc, char** argv)
{
    ////////////////////////////////////////////////////////////////
    // Init ROS
    ////////////////////////////////////////////////////////////////
    ros::init(argc, argv, "bilateral_slave");
    BilateralController bi;

    ros::spin();

    ROS_INFO("Ending Session....");

    return 0;
}
