#include "hpe2udp.h"
   

HpeToUdp::HpeToUdp(ros::NodeHandle nh) : nodeHandle_(nh)
{
    ROS_INFO("[HpeToUdp] Initializing node");

    rightArmJoints      = nodeHandle_.subscribe<hpe_ros_msgs::JointArmCmd>(std::string("right_arm"), 1, &HpeToUdp::rightArmJointsCallback, this);
    leftArmJoints       = nodeHandle_.subscribe<hpe_ros_msgs::JointArmCmd>(std::string("left_arm"), 1, &HpeToUdp::leftArmJointsCallback, this);
    rightArmCartesian   = nodeHandle_.subscribe<hpe_ros_msgs::CartesianArmCmd>(std::string("cart_right_arm"), 1, &HpeToUdp::rightArmCartesianCallback, this);
    leftArmCartesian    = nodeHandle_.subscribe<hpe_ros_msgs::CartesianArmCmd>(std::string("cart_left_arm"), 1, &HpeToUdp::leftArmCartesianCallback, this);

    // Initialize UDP socket
    init();
}

HpeToUdp::~HpeToUdp()
{
    close(sockfd);
}

void HpeToUdp::init()
{
    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        ROS_ERROR("Socket creation failed");
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
    ROS_INFO_ONCE("Recieved first message from right arm joints"); 
    double spitch, sroll, syaw, elbow; 
    spitch  = static_cast<double>(msg->shoulder_pitch.data);
    sroll   = static_cast<double>(msg->shoulder_roll.data);
    syaw    = static_cast<double>(msg->shoulder_yaw.data);
    elbow   = static_cast<double>(msg->elbow.data);

    armsControlReferencesDataPacket.rightArmJointPositionRef[0] = spitch;
    armsControlReferencesDataPacket.rightArmJointPositionRef[1] = sroll; 
    armsControlReferencesDataPacket.rightArmJointPositionRef[2] = syaw; 
    armsControlReferencesDataPacket.rightArmJointPositionRef[3] = elbow; 

    //ROS_INFO_STREAM("Right arm joints callback: " << spitch << " " << sroll << " " << syaw << " " << elbow << ""); 
}

void HpeToUdp::leftArmJointsCallback(const hpe_ros_msgs::JointArmCmd::ConstPtr& msg)
{   
    ROS_INFO_ONCE("Recieved first message from left arm joints");
    double spitch, sroll, syaw, elbow; 
    spitch  = static_cast<double>(msg->shoulder_pitch.data);
    sroll   = static_cast<double>(msg->shoulder_roll.data);
    syaw    = static_cast<double>(msg->shoulder_yaw.data);
    elbow   = static_cast<double>(msg->elbow.data);

    armsControlReferencesDataPacket.leftArmCartesianPositionRef[0] = spitch; 
    armsControlReferencesDataPacket.leftArmCartesianPositionRef[1] = sroll; 
    armsControlReferencesDataPacket.leftArmCartesianPositionRef[2] = syaw; 
    armsControlReferencesDataPacket.leftArmCartesianPositionRef[3] = elbow; 

    //ROS_INFO_STREAM("Left arm joints callback: " << spitch << " " << sroll << " " << syaw << " " << elbow << "");
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
    ros::Rate r(1);
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



