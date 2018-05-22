#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>


#include "pid.hpp"

#define MAX_PITCH 5.0
#define MAX_ROLL 5.0


double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}

class Controller
{
public:

    Controller(
        const std::string& worldFrame,
        const std::string& frame,
        const ros::NodeHandle& n)
        : m_worldFrame(worldFrame)
        , m_frame(frame)
        , m_getTwistData()
        , m_pubNav()
        , m_listener()
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
        m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0)); 
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_subscribeGoal = nh.subscribe("goal", 1, &Controller::goalChanged, this);
        m_getTwistData = m_frame + "/twist";
        m_subscribeTwist = nh.subscribe(m_getTwistData, 1, &Controller::getTwistData, this);
        m_serviceTakeoff = nh.advertiseService("takeoff", &Controller::takeoff, this);
        m_serviceLand = nh.advertiseService("land", &Controller::land, this);
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        ros::spin();
    }

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

    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
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
            velocityX[thisReading] = 0;
            velocityY[thisReading] = 0;
        }
    }

    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();
        initVelComputation();

        switch(m_state)
        {
        case TakingOff:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

                if (transform.getOrigin().z() > m_startZ + 0.05 || m_thrust > 50000)
                {
                    pidReset();
                    m_pidZ.setIntegral(m_thrust / m_pidZ.ki());

                    m_goal_temp = m_goal;

                    m_goal_temp.pose.position.x = m_startX;
                    m_goal_temp.pose.position.y = m_startY;

                    m_state = GoToZDesired;
                    m_thrust = 0;
                }
                else
                {
                    m_thrust += 14500 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    m_pubNav.publish(msg);
                }

            }
            break;
        case GoToZDesired:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

                geometry_msgs::PoseStamped targetWorld;
                targetWorld.header.stamp = transform.stamp_;
                targetWorld.header.frame_id = m_worldFrame;
                targetWorld.pose = m_goal_temp.pose;

                geometry_msgs::PoseStamped targetDrone;
                m_listener.transformPose(m_frame, targetWorld, targetDrone);

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);

                geometry_msgs::Twist msg;
                msg.linear.x = m_pidX.update(0, targetDrone.pose.position.x);
                msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y);
                msg.linear.z = m_pidZ.update(0.0, targetDrone.pose.position.z);
                msg.angular.z = m_pidYaw.update(0.0, yaw);
                m_pubNav.publish(msg);

                m_HoverRMSEX = targetDrone.pose.position.x * targetDrone.pose.position.x;
                m_HoverRMSEY = targetDrone.pose.position.y * targetDrone.pose.position.y;
                m_HoverRMSEZ = targetDrone.pose.position.z * targetDrone.pose.position.z;

                //ROS_INFO("Error x = %f, y = %f, z = %f", m_HoverRMSEX, m_HoverRMSEY, m_HoverRMSEZ);

                if (m_HoverRMSEX < 0.05 && m_HoverRMSEY < 0.05 && m_HoverRMSEZ < 0.05)
                    m_state = Automatic;
                
                /*tf::StampedTransform transform1;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform1);

                geometry_msgs::PoseStamped targetWorld1;
                targetWorld1.header.stamp = transform1.stamp_;
                targetWorld1.header.frame_id = m_worldFrame;
                targetWorld1.pose = m_goal.pose;

                geometry_msgs::PoseStamped targetDrone1;
                m_listener.transformPose(m_frame, targetWorld1, targetDrone1);

                tfScalar roll1, pitch1, yaw1;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone1.pose.orientation.x,
                        targetDrone1.pose.orientation.y,
                        targetDrone1.pose.orientation.z,
                        targetDrone1.pose.orientation.w
                    )).getRPY(roll1, pitch1, yaw1);

                float s_x = targetDrone1.pose.position.x + (1/2*0.1744)*m_twistData.twist.linear.x*fabs(m_twistData.twist.linear.x);
                float s_y = targetDrone1.pose.position.y + (1/2*0.1744)*m_twistData.twist.linear.y*fabs(m_twistData.twist.linear.y);

                ROS_INFO("%f %f %f %f %f %f", s_x, s_y, targetDrone1.pose.position.x, m_twistData.twist.linear.x, targetDrone1.pose.position.y, m_twistData.twist.linear.y);*/
            }
            break;
        case Automatic:
            {
                //ROS_INFO("Automatic mode initiated");
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

                geometry_msgs::PoseStamped targetWorld;
                targetWorld.header.stamp = transform.stamp_;
                targetWorld.header.frame_id = m_worldFrame;
                targetWorld.pose = m_goal.pose;

                geometry_msgs::PoseStamped targetDrone;
                m_listener.transformPose(m_frame, targetWorld, targetDrone);

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);

                geometry_msgs::Twist msg;

                //********************VELOCITY COMPUTATION*******************************
                totalX = totalX - velocityX[readIndex];
                totalY = totalY - velocityY[readIndex];
                
                currX = -targetDrone.pose.position.x;
                currY = -targetDrone.pose.position.y;
                
                velocityX[readIndex] = (currX - prevX)/dt;
                velocityY[readIndex] = (currY - prevY)/dt;
                
                totalX = totalX + velocityX[readIndex];
                totalY = totalY + velocityY[readIndex];

                readIndex = readIndex + 1;

                if (readIndex >= 5)
                {
                    readIndex = 0;
                } 

                avgVelX = totalX / 5;
                avgVelY = totalY / 5;

                //************************************************************************

                //********* TIME OPTIMAL CONTROLLER START**************
                //float s_x = targetDrone.pose.position.x + (1/2*0.174)*m_twistData.twist.linear.x*fabs(m_twistData.twist.linear.x);
                //float s_y = targetDrone.pose.position.y + (1/2*0.174)*m_twistData.twist.linear.y*fabs(m_twistData.twist.linear.y);

                float s_x = currX + ((1/(2*10*0.0175))*avgVelX*fabs(avgVelX));
                float s_y = currY + ((1/(2*10*0.0175))*avgVelY*fabs(avgVelY));
                //ROS_INFO("%f %f %f %f %f %f", s_x, s_y, currX, currY, avgVelX, avgVelY);
                //ROS_INFO("s_x = %f, s_y = %f", s_x, s_y);

                if (fabs(targetDrone.pose.position.x) > 0.25)  
                {
                    //ROS_INFO("Error present");
                    //msg.linear.x = m_pidX.update(0, targetDrone.pose.position.x);
                    //msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y);
                    ROS_INFO("%f %f %f %f %f %f %f %f %f", s_x, s_y, currX, currY, avgVelX, avgVelY, prevX, prevY, dt);
                    if (s_x > 0.1)
                    {
                        //ROS_INFO("%f", s_x);
                        msg.linear.x = -MAX_PITCH;
                    }
                    else if (s_x < -0.1)
                    {
                        //ROS_INFO("%f", s_x);
                        msg.linear.x = +MAX_PITCH;
                    }
                    else if (s_x < 0.1 && s_x > -0.1)
                    {
                        if (avgVelX > 0.1)
                            msg.linear.x = -MAX_PITCH;
                        else if (avgVelX < -0.1)
                            msg.linear.x = MAX_PITCH;
                        else
                            msg.linear.x = 0;
                    }
                    else
                        msg.linear.x = 0.0;
                }
                else
                {
                    msg.linear.x = m_pidX.update(0, targetDrone.pose.position.x);
                }

                if (fabs(targetDrone.pose.position.y) > 0.25)
                {
                    ROS_INFO("%f %f %f %f %f %f %f %f %f", s_x, s_y, currX, currY, avgVelX, avgVelY, prevX, prevY, dt);
                    if (s_y > 0.1)
                    {
                        //ROS_INFO("%f", s_x);
                        msg.linear.y = +MAX_ROLL;
                    }
                    else if (s_y < -0.1)
                    {
                        //ROS_INFO("%f", s_x);
                        msg.linear.y = -MAX_ROLL;
                    }
                    else if (s_y < 0.1 && s_y > -0.1)
                    {
                        if (avgVelY > 0.1)
                            msg.linear.y = MAX_ROLL;
                        else if (avgVelY < -0.1)
                            msg.linear.y = -MAX_ROLL;
                        else
                            msg.linear.y = 0.0;
                    }
                    else
                    {
                        //ROS_INFO("Case 5");
                        msg.linear.y = 0.0;
                    }
                }
                else
                {
                    msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y);
                }
                    
                msg.linear.z = m_pidZ.update(0.0, targetDrone.pose.position.z);
                msg.angular.z = m_pidYaw.update(0.0, yaw);
                m_pubNav.publish(msg);
            
                prevX = currX;
                prevY = currY;
                //ROS_INFO("%f %f %f %f %f %f", s_x, s_y, targetDrone.pose.position.x, m_twistData.twist.linear.x, targetDrone.pose.position.y, m_twistData.twist.linear.y);
                //********* TIME OPTIMAL CONTROLLER END*************/

                /**************PID POSITION CONTROLLER***********************

                msg.linear.x = m_pidX.update(0, targetDrone.pose.position.x);
                msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y);
                msg.linear.z = m_pidZ.update(0.0, targetDrone.pose.position.z);
                msg.angular.z = m_pidYaw.update(0.0, yaw);
                m_pubNav.publish(msg);
                /*************************************************************/
            }
            break;
        case Landing:
            {
                m_goal.pose.position.z = m_startZ + 0.05;
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() <= m_startZ + 0.05) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    m_pubNav.publish(msg);
                }

                geometry_msgs::PoseStamped targetWorld;
                targetWorld.header.stamp = transform.stamp_;
                targetWorld.header.frame_id = m_worldFrame;
                targetWorld.pose = m_goal.pose;

                geometry_msgs::PoseStamped targetDrone;
                m_listener.transformPose(m_frame, targetWorld, targetDrone);

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);

                geometry_msgs::Twist msg;
                msg.linear.x = m_pidX.update(0, targetDrone.pose.position.x);
                msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y);
                msg.linear.z = m_pidZ.update(0.0, targetDrone.pose.position.z);
                msg.angular.z = m_pidYaw.update(0.0, yaw);
                m_pubNav.publish(msg);
            }
            break;
        case Idle:
            {
                geometry_msgs::Twist msg;
                m_pubNav.publish(msg);
            }
            break;
        }
    }

private:

    enum State
    {
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing = 4,
        GoToZDesired = 3,
    };

private:
    std::string m_worldFrame;
    std::string m_frame;
    std::string m_getTwistData;

    ros::Publisher m_pubNav;
    tf::TransformListener m_listener;
    PID m_pidX;
    PID m_pidY;
    PID m_pidZ;
    PID m_pidYaw;
    State m_state;
    geometry_msgs::PoseStamped m_goal;
    geometry_msgs::PoseStamped m_goal_temp;
    geometry_msgs::TwistStamped m_twistData;
    ros::Subscriber m_subscribeGoal;
    ros::Subscriber m_subscribeTwist;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    float m_thrust;
    float m_startZ, m_startX, m_startY;
    float m_HoverRMSEX, m_HoverRMSEY, m_HoverRMSEZ;

    float totalX = 0.0, totalY = 0.0;
    float velocityX[5], velocityY[5];
    int readIndex;
    float currX, currY, prevX, prevY, avgVelX, avgVelY;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  // Read parameters
  ros::NodeHandle n("~");
  std::string worldFrame;
  n.param<std::string>("worldFrame", worldFrame, "/world");
  std::string frame;
  n.getParam("frame", frame);
  double frequency;
  n.param("frequency", frequency, 50.0);

  Controller controller(worldFrame, frame, n);
  controller.run(frequency);

  return 0;
}
