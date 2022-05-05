
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <iostream>

class GazeboRosInterface
{
    private:
        gazebo::transport::SubscriberPtr sub_gazebo_wrench;
        ros::NodeHandle ros_node; 
        ros::Publisher pub_ros_wrench;
        ros::Subscriber sub_;

    public :
        GazeboRosInterface()
        {
            // Create Gazebo node and init
            gazebo::transport::NodePtr gazebo_node(new gazebo::transport::Node());
            gazebo_node->Init();

            pub_ros_wrench = ros_node.advertise<geometry_msgs::WrenchStamped>("/ur/ft_sensor/raw", 1);

            // Listen to Gazebo wrench topic
            sub_gazebo_wrench = gazebo_node->Subscribe(
            "/gazebo/default/robot/wrist_3_joint/wrist_3_joint_force_torque/wrench",
            &GazeboRosInterface::WrenchCallback, this);
        }
        // Forces callback function
        void WrenchCallback(ConstWrenchStampedPtr &_gazebo_wrench_msg)
        {
            geometry_msgs::WrenchStamped _ros_wrench_msg;
            
            _ros_wrench_msg.wrench.force.x = _gazebo_wrench_msg->wrench().force().x();
            _ros_wrench_msg.wrench.force.y = _gazebo_wrench_msg->wrench().force().y();
            _ros_wrench_msg.wrench.force.z = _gazebo_wrench_msg->wrench().force().z();
            _ros_wrench_msg.wrench.torque.z = _gazebo_wrench_msg->wrench().torque().x();
            _ros_wrench_msg.wrench.torque.y = _gazebo_wrench_msg->wrench().torque().y();
            _ros_wrench_msg.wrench.torque.z = _gazebo_wrench_msg->wrench().torque().z();

            pub_ros_wrench.publish(_ros_wrench_msg);
        }
};


int main(int argc, char **argv)
{
    // Load Gazebo & ROS
    gazebo::client::setup(argc, argv);
    ros::init(argc, argv, "GazeboRosInterfaceNode");

    GazeboRosInterface Wrist3JointObject;

    ros::spin();
    return 0;
}