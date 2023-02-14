// Client side implementation of UDP client-server model
#ifndef HPE2UDP_H
#define HPE2UDP_H

/* UDP socket include*/
#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>


/* ROS include*/
#include <ros/ros.h>
#include "hpe_ros_msgs/JointArmCmd.h"
#include "hpe_ros_msgs/CartesianArmCmd.h"

/* LICAS arms struct*/
#include "StructureInterface.h"

/* Define constants */
#define PORT 22008 
#define MAXLINE 1024
#define NUM_ARM_JOINTS 4

class HpeToUdp
{
    public:
        HpeToUdp(ros::NodeHandle nh);
        ~HpeToUdp();

        void init();
        void run();

        int socket_publisher_ = -1;
        /* UDP socket stuff*/
        struct sockaddr_in addr_host_;
        struct hostent * host;
        bool ready_ = false;
        bool pub_ready_ = false;
        bool recv_ready_ = false; 

        // Initialize armsCtl data packet
        ARMS_CONTROL_REFERENCES_DATA_PACKET armsCtl={
            .mode                           = 0, 
            .leftArmGripperPositionRef      = 0.0,  
            .leftArmJointPositionRef        = {0.0, 0.0, 0.0, 0.0}, 
            .leftArmCartesianPositionRef    = {0.0, 0.0, 0.0}, 
            .leftArmJointTorqueRef          = {0.0, 0.0, 0.0, 0.0}, 
            .leftArmForce                   = {0.0, 0.0, 0.0},
            .rightArmGripperPositionRef     = 0.0, 
            .rightArmJointPositionRef       = {0.0, 0.0, 0.0, 0.0}, 
            .rightArmCartesianPositionRef   = {0.0, 0.0, 0.0}, 
            .rightArmJointTorqueRef         = {0.0, 0.0, 0.0, 0.0},
            .rightArmForce                  = {0.0, 0.0, 0.0} 
        }; 



    private: 

        /* ROS stuff */
        ros::NodeHandle nodeHandle_;
        ros::Subscriber rightArmJoints; 
        ros::Subscriber leftArmJoints;
        ros::Subscriber rightArmCartesian;
        ros::Subscriber leftArmCartesian;

        void rightArmJointsCallback(const hpe_ros_msgs::JointArmCmd::ConstPtr& msg);
        void leftArmJointsCallback(const hpe_ros_msgs::JointArmCmd::ConstPtr& msg);
        void rightArmCartesianCallback(const hpe_ros_msgs::CartesianArmCmd::ConstPtr& msg);
        void leftArmCartesianCallback(const hpe_ros_msgs::CartesianArmCmd::ConstPtr& msg);

        double rightArmJointPositions[4]; 
        double leftArmJointPositions[4];
        double rightArmCartesianPositions[3];
        double leftArmCartesianPositions[3];

        bool init_pub_socket();
        bool init_sub_socket(); 



};

#endif // HPE2UDP_H
