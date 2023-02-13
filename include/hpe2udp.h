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

/* ROS include*/
#include <ros/ros.h>
#include "hpe_ros_msgs/JointArmCmd.h"
#include "hpe_ros_msgs/CartesianArmCmd.h"

/* Define constants */
#define PORT 8080
#define MAXLINE 1024

class HpeToUdp
{
    public:
        HpeToUdp(ros::NodeHandle nh);
        ~HpeToUdp();

        void init();
        void run();

    private: 

        /* ROS stuff */
        ros::NodeHandle nodeHandle_;
        ros::Subscriber rightArmJoints; 
        ros::Subscriber leftArmJoints;
        ros::Subscriber rightArmCartesian;
        ros::Subscriber leftArmCartesian;

        void rightArmJointsCallback(const hpe_ros_msgs::JointArmCmd::ConstPtr& msg);
        void leftArmJointsCallback(const hpe_ros_msgs::JointArmCmd& msg);
        void rightArmCartesianCallback(const hpe_ros_msgs::CartesianArmCmd& msg);
        void leftArmCartesianCallback(const hpe_ros_msgs::CartesianArmCmd& msg);

        /* UDP socket stuff*/
        int sockfd;
        char buffer[MAXLINE];
        struct sockaddr_in servaddr;
        int n;
        socklen_t len;

};

#endif // HPE2UDP_H