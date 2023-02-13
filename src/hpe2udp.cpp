#include "hpe2udp.h"
   

HpeToUdp::HpeToUdp(ros::NodeHandle nh) : nodeHandle_(nh)
{
    ROS_INFO("[HpeToUdp] Initializing node");

    rightArmJoints      = nodeHandle_.subscribe<hpe_ros_msgs::JointArmCmd>(std::string("right_arm_joints"), 1, &HpeToUdp::rightArmJointsCallback, this);
    leftArmJoints       = nodeHandle_.subscribe<hpe_ros_msgs::JointArmCmd>(std::string("left_arm_joints"), 1, &HpeToUdp::leftArmJointsCallback, this);
    rightArmCartesian   = nodeHandle_.subscribe<hpe_ros_msgs::CartesianArmCmd>(std::string("right_arm_cartesian"), 1, &HpeToUdp::rightArmCartesianCallback, this);
    leftArmCartesian    = nodeHandle_.subscribe<hpe_ros_msgs::CartesianArmCmd>(std::string("left_arm_cartesian"), 1, &HpeToUdp::leftArmCartesianCallback, this);
}

HpeToUdp::~HpeToUdp()
{
    close(sockfd);
}

void HpeToUdp::init()
{
    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        ROS_ERROR("socket creation failed");
        exit(EXIT_FAILURE);
    }
   
    memset(&servaddr, 0, sizeof(servaddr));
       
    // Filling server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = INADDR_ANY;
}

void HpeToUdp::rightArmJointsCallback(const hpe_ros_msgs::JointArmCmd::ConstPtr& msg)
{
    std::cout<<"Right arm joints callback"<<std::endl;
    armsControlReferencesDataPacket.rightArmJointPositionRef[0] = static_cast<double>(msg->shoulder_pitch.data); 
    armsControlReferencesDataPacket.rightArmJointPositionRef[1] = static_cast<double>(msg->shoulder_roll.data);
    armsControlReferencesDataPacket.rightArmJointPositionRef[2] = static_cast<double>(msg->shoulder_yaw.data);
    armsControlReferencesDataPacket.rightArmJointPositionRef[3] = static_cast<double>(msg->elbow.data);
}

void HpeToUdp::leftArmJointsCallback(const hpe_ros_msgs::JointArmCmd::ConstPtr& msg)
{
    armsControlReferencesDataPacket.leftArmCartesianPositionRef[0] = static_cast<double>(msg->shoulder_pitch.data);
    armsControlReferencesDataPacket.leftArmCartesianPositionRef[1] = static_cast<double>(msg->shoulder_roll.data);
    armsControlReferencesDataPacket.leftArmCartesianPositionRef[2] = static_cast<double>(msg->shoulder_yaw.data);
    armsControlReferencesDataPacket.leftArmCartesianPositionRef[3] = static_cast<double>(msg->elbow.data);
}

void HpeToUdp::rightArmCartesianCallback(const hpe_ros_msgs::CartesianArmCmd::ConstPtr& msg)
{
    std::cout<<"Right arm joints callback"<<std::endl;
}

void HpeToUdp::leftArmCartesianCallback(const hpe_ros_msgs::CartesianArmCmd::ConstPtr& msg)
{
    std::cout<<"Right arm joints callback"<<std::endl;
}



void HpeToUdp::run()
{   
    ros::Rate r(10);
    while(ros::ok())
    {   
        std::cout<< "Node is running!" << std::endl; 
        r.sleep(); 
        //sendto(sockfd, (const char *)hello, strlen(hello),
        //    MSG_CONFIRM, (const struct sockaddr *) &servaddr, 
        //        sizeof(servaddr));
        //std::cout<<"Hello message sent."<<std::endl;
        //n = recvfrom(sockfd, (char *)buffer, MAXLINE, 
        //            MSG_WAITALL, (struct sockaddr *) &servaddr,
        //            &len);
        //buffer[n] = '\0';
        //std::cout<<"Server :"<<buffer<<std::endl;
    }
}



