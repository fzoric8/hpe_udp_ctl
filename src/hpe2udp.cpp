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
    close(socket_publisher_);
}

void HpeToUdp::init()
{
    ros::Time rosTime; 

    pub_ready_ = init_pub_socket();
    recv_ready_ = init_sub_socket();     
    
}

bool HpeToUdp::init_pub_socket(){
    // Open the socket in datagram mode
    socket_publisher_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (socket_publisher_ < 0) {
        ROS_ERROR("ArmsReferencesSender: could not open socket!");
        //close(socket_publisher_);
        
        return false;
    }

    host = gethostbyname("192.168.43.20");
    if (host == NULL) {
        ROS_ERROR("ArmsReferencesSender: could not get host by name!");
        close(socket_publisher_);
        return false;
    }

    bzero((char*)&addr_host_, sizeof(struct sockaddr_in));
    addr_host_.sin_family = AF_INET;
    bcopy((char*)host->h_addr, (char*)&addr_host_.sin_addr.s_addr, host->h_length);
    addr_host_.sin_port = htons(22008);

    ready_ = true;

    if (ready_){
        ROS_INFO_STREAM("Succesfuly initialized UDP socket!");
        ros::Duration(2.0).sleep(); 
    }
    else{
        ROS_ERROR("Failed to initialize UDP socket!");
        close(socket_publisher_); 
    }

    return ready_; 
}

bool HpeToUdp::init_sub_socket()
{
    return false; 
}

void HpeToUdp::rightArmJointsCallback(const hpe_ros_msgs::JointArmCmd::ConstPtr& msg)
{   
    ROS_INFO_ONCE("Recieved first message from right arm joints"); 
    double spitch, sroll, syaw, elbow; 
    spitch  = static_cast<double>(msg->shoulder_pitch.data);
    sroll   = static_cast<double>(msg->shoulder_roll.data);
    syaw    = static_cast<double>(msg->shoulder_yaw.data);
    elbow   = static_cast<double>(msg->elbow.data);
    
    // Looked arms from wrong direction
    double c = -1.0;  
    double t = static_cast<double>(ros::Time::now().toSec());
    armsCtl.mode = 1; 
    armsCtl.rightArmJointPositionRef[0] = c*spitch; 
    armsCtl.rightArmJointPositionRef[1] = c*sroll;  
    armsCtl.rightArmJointPositionRef[2] = syaw;  
    armsCtl.rightArmJointPositionRef[3] = c*elbow; 
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

    // Looked arms from wrong direction
    double c = -1.0; 
    double t = static_cast<double>(ros::Time::now().toSec());
    armsCtl.leftArmJointPositionRef[0] = c*spitch; 
    armsCtl.leftArmJointPositionRef[1] = c*sroll;  
    armsCtl.leftArmJointPositionRef[2] = syaw; 
    armsCtl.leftArmJointPositionRef[3] = c*elbow;  
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
    ros::Rate r(50);

    while(ros::ok())
    {   
        std::cout<< "Node is running!" << std::endl; 
        if(pub_ready_)
        {   

            //ROS_INFO_STREAM("Socket descriptor is: " << socket_publisher_); 
            
            double t = static_cast<double>(ros::Time::now().toSec());

            int s = sendto(socket_publisher_, (const char *)&armsCtl, sizeof(ARMS_CONTROL_REFERENCES_DATA_PACKET), 0, (const struct sockaddr *) &addr_host_, sizeof(addr_host_));
            
            if (s < 0)
            {
                ROS_ERROR("ArmsReferencesSender: could not send message!");
                close(socket_publisher_);
            }
            //ROS_INFO_STREAM("Socket send return value is: " << s); 

            //n = recvfrom(socket_publisher_, (char *)buffer, MAXLINE, MSG_WAITALL, (struct sockaddr *) &addr_host_, &len);

        }
        r.sleep(); 
    }
}



