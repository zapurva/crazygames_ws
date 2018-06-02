#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

#include "pid.hpp"

#define MAX_PITCH 7.0
#define MAX_ROLL 7.0

//***************************************
//Tuning parameter for switching function
#define c_x 0.1375  
#define c_y 0.1375
//***************************************

double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}
	
class Follower
{
public:

    Follower(
        const std::string& worldFrame,
        const std::string& leaderFrame,
        const std::string& followerFrame,
        const float& xOffset,
        const float& yOffset,
        const ros::NodeHandle& n)
        : m_worldFrame(worldFrame)
        , m_frame_leader(leaderFrame)
        , m_frame_follower(followerFrame)
        , m_xOffset(xOffset)
        , m_yOffset(yOffset)
        , m_pubNav()
        , m_listener_follower()
        , m_pidX(
            get(n, "PIDs/X/kp"),
            get(n, "PIDs/X/kd"),
            get(n, "PIDs/X/ki"),
            get(n, "PIDs/X/minOutput"),
            get(n, "PIDs/X/maxOutput"),
            get(n, "PIDs/X/integratorMin"),
            get(n, "PIDs/X/integratorMax"),
            "x")
        , m_pidY(
            get(n, "PIDs/Y/kp"),
            get(n, "PIDs/Y/kd"),
            get(n, "PIDs/Y/ki"),
            get(n, "PIDs/Y/minOutput"),
            get(n, "PIDs/Y/maxOutput"),
            get(n, "PIDs/Y/integratorMin"),
            get(n, "PIDs/Y/integratorMax"),
            "y")
        , m_pidZ(
            get(n, "PIDs/Z/kp"),
            get(n, "PIDs/Z/kd"),
            get(n, "PIDs/Z/ki"),
            get(n, "PIDs/Z/minOutput"),
            get(n, "PIDs/Z/maxOutput"),
            get(n, "PIDs/Z/integratorMin"),
            get(n, "PIDs/Z/integratorMax"),
            "z")
        , m_pidYaw(
            get(n, "PIDs/Yaw/kp"),
            get(n, "PIDs/Yaw/kd"),
            get(n, "PIDs/Yaw/ki"),
            get(n, "PIDs/Yaw/minOutput"),
            get(n, "PIDs/Yaw/maxOutput"),
            get(n, "PIDs/Yaw/integratorMin"),
            get(n, "PIDs/Yaw/integratorMax"),
            "yaw")
        , m_state(Idle)
        , m_goal()
        , m_subscribeGoal()
        , m_subscribeTwist()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_thrust(0)
        , m_startZ(0)
    {
        ros::NodeHandle nh;
        m_listener_leader.waitForTransform(m_worldFrame, m_frame_leader, ros::Time(0), ros::Duration(10.0)); 
        m_listener_follower.waitForTransform(m_worldFrame, m_frame_follower, ros::Time(0), ros::Duration(10.0)); 
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_subscribeGoal = nh.subscribe("goal", 1, &Follower::goalChanged, this);
        m_subscribeTwist = nh.subscribe(m_getTwistData, 1, &Follower::getTwistData, this);
        m_serviceTakeoff = nh.advertiseService("takeoff", &Follower::takeoff, this);
        m_serviceLand = nh.advertiseService("land", &Follower::land, this);
        initVelComputation();
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        ros::spin();
    }
    
    int stabilizeCheck = 0;

private:
    void goalChanged(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        m_goal = *msg;
    }

    void getTwistData(
        const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        m_twistData = *msg;
    }

    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;

        tf::StampedTransform transform;
        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

        /*ROS_INFO("Goal1 x = %f", m_goal.pose.position.x);
        ROS_INFO("Goal1 y = %f", m_goal.pose.position.y);
        ROS_INFO("Goal1 z = %f", m_goal.pose.position.z);*/

        m_startZ = transform.getOrigin().z();

        m_startX = transform.getOrigin().x();
        m_startY = transform.getOrigin().y();

        return true;
    }

    bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;

        return true;
    }

    void pidReset()
    {
        m_pidX.reset();
        m_pidY.reset();
        m_pidZ.reset();
        m_pidYaw.reset();
    }

    void initVelComputation()
    {
        for (int thisReading = 0; thisReading < 5; thisReading++) 
        {
            velocityX[thisReading] = 0.0;
            velocityY[thisReading] = 0.0;
            //ROS_INFO("velocityX = %f", velocityX[thisReading]);
        }
    }