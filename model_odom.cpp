#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <szelectricity_common/VehicleParameters.hpp>
#include <szelectricity_common/MathGeometry.hpp>

const std::string DEFAULT_BASE_FRAME = "base_footprint";
const std::string DEFAULT_ODOM_FRAME = "odom";

struct joints {
  double p1;
  double p2;
  double velo1;
} olvasas;

sensor_msgs::JointState msg_joint;

nav_msgs::Odometry current_state;         // Current odometry state (pose, twist, covariance)
double theta = 0.0;                       // Current yaw related to COG
double dtheta = 0.0;                      // Change in theta
double dx = 0.0;                          // Change in x
double dy = 0.0;                          // Change in y

ros::Time prev_time, curr_time;

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  //std::cout << *msg << std::endl;
  msg_joint.position = msg->position;
  msg_joint.velocity = msg->velocity;
  //std::cout << msg_joint.position[0] << std::endl;
  //std::cout << msg_joint.velocity[2] << std::endl;
  olvasas.p1 = msg_joint.position[0];
  //std::cout << olvasas.p1 << std::endl;
  olvasas.p2 = msg_joint.position[1];
  olvasas.velo1 = msg_joint.velocity[2];

}

void UpdateStep(double dt, const ros::Time stamp)
    {

        current_state.header.stamp = stamp;
        // Update the pose estimation
        theta = theta+dtheta*dt;
        current_state.twist.twist.linear.x = dx;
        current_state.twist.twist.linear.y = dy;
        // Angular speed is defined as RPY, only YAW is considered
        current_state.twist.twist.angular.z = dtheta*dt;
        // Pose update
        current_state.pose.pose.position.x += dx*dt;
        current_state.pose.pose.position.y += dy*dt;        
        
        current_state.pose.pose.orientation = szenergy::YawToQuaternion(theta);    
    }

int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "joint_listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/joint_states", 1000, jointCallback);
    ros::Publisher odom = n.advertise <nav_msgs::Odometry> ("odometry", 1000);
    ros::Rate loop_rate(20);

    
    szenergy::VehicleParameters params("modelcar",
      0.045,      
      0.267,
      0.188, 
      0.147);

    current_state.pose.pose.position.z = 0.0;
    current_state.twist.twist.linear.z = 0.0;
    current_state.pose.pose.orientation.w = 1.0;
        
    current_state.twist.twist.angular.x = 0.0;
    current_state.twist.twist.angular.y = 0.0;

    current_state.header.frame_id = "odom";
    current_state.child_frame_id = "base_link";

    while(ros::ok())
    
    {
        olvasas.p1;
        olvasas.p2;
        olvasas.velo1;
        //std::cout << olvasas.p1 << '\t' << olvasas.p2 << '\t' << olvasas.velo1 << std::endl;
        
        ros::Duration dt;
        const ros::Time stamp;
        curr_time = ros::Time::now();

        dt = curr_time - prev_time;


        double linvel = olvasas.velo1*params.wheelradius;
        //std::cout << linvel << '\t';
        dtheta = linvel/(params.wheelbase)*tan(olvasas.p1);
        //std::cout << dtheta << '\t';
        dx = linvel*cos(theta);
        //std::cout << dx << '\t';
        dy = linvel*sin(theta);
        //std::cout << dt << std::endl;
        UpdateStep(dt.toSec(), stamp);

        odom.publish(current_state);

        prev_time = curr_time;

		ros::spinOnce();

		loop_rate.sleep();
    }
  }
