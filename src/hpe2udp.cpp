#include "hpe2udp.h"
   

HpeToUdp::HpeToUdp(ros::NodeHandle nh) : nodeHandle_(nh)
{
    ROS_INFO("[HpeToUdp] Initializing node");

    bool kalman = true; 
    std::string cartLeftSubName, cartRightSubName; 
    if(kalman){
        cartLeftSubName  = "/kalman_cart_left_arm";
        cartRightSubName = "/kalman_cart_right_arm";
    }
    else{
        cartLeftSubName  = "/cart_left_arm";
        cartRightSubName = "/cart_right_arm";
    }

    rightArmJointsSub      = nodeHandle_.subscribe<hpe_ros_msgs::JointArmCmd>(std::string("/right_arm"), 1, &HpeToUdp::rightArmJointsCallback, this);
    leftArmJointsSub       = nodeHandle_.subscribe<hpe_ros_msgs::JointArmCmd>(std::string("/left_arm"), 1, &HpeToUdp::leftArmJointsCallback, this);
    rightArmCartesianSub   = nodeHandle_.subscribe<hpe_ros_msgs::CartesianArmCmd>(cartRightSubName, 1, &HpeToUdp::rightArmCartesianCallback, this);
    leftArmCartesianSub    = nodeHandle_.subscribe<hpe_ros_msgs::CartesianArmCmd>(cartLeftSubName, 1, &HpeToUdp::leftArmCartesianCallback, this); 

    // Initialize UDP socket
    init();

    ROS_INFO("[HpeToUdp] Node initialized");
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

    if (pub_ready_){
        ROS_INFO_STREAM("Succesfuly initialized UDP pub socket!");
        ros::Duration(1.0).sleep(); 
    }
    else{
        ROS_ERROR("Failed to initialize UDP pub socket!");
        close(socket_publisher_); 
    }

    if (recv_ready_){
        ROS_INFO_STREAM("Succesfuly initialized UDP sub socket!");
        ros::Duration(1.0).sleep();
    }
    else{
        ROS_ERROR("Failed to successfully initialize UDP sub socket!");
        close(socket_receiver_);
    }

    
}

bool HpeToUdp::init_pub_socket(){
    // Open the socket in datagram mode
    socket_publisher_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (socket_publisher_ < 0) {
        ROS_ERROR("ArmsReferencesSender: could not open pub socket!");
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

    return true; 
}

bool HpeToUdp::init_sub_socket()
{
    socket_receiver_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (socket_receiver_ < 0) {
        ROS_ERROR("ArmsReferencesSender: could not open sub socket!");
        
        return false;
    }

    recv_addr_.sin_family = AF_INET; 
    recv_addr_.sin_addr.s_addr = INADDR_ANY; 
    recv_addr_.sin_port = htons(22009);

    if (bind(socket_receiver_, (struct sockaddr*)&recv_addr_, sizeof(recv_addr_)) < 0){
        ROS_ERROR("ArmsStateReciever: could not bind sub socket!");
        close(socket_receiver_); 
        return false; 
    } 

    return true; 
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
    ROS_INFO_STREAM("Left arm joints callback: " << spitch << " " << sroll << " " << syaw << " " << elbow << "");
}

void HpeToUdp::rightArmCartesianCallback(const hpe_ros_msgs::CartesianArmCmd::ConstPtr& msg)
{
    ROS_INFO_ONCE("Recieved first cartesian message from right arm");
    float x, y, z, vx, vy, vz;
    x = msg->positionEE.x; y = msg->positionEE.y; z = msg->positionEE.z;
    vx = msg->velocityEE.x; vy = msg->velocityEE.y; vz = msg->velocityEE.z;
    armsCtl.rightArmCartesianPositionRef[0] = x; 
    armsCtl.rightArmCartesianPositionRef[1] = y;
    armsCtl.rightArmCartesianPositionRef[2] = z;
    ROS_INFO_STREAM_THROTTLE(1.0, "Right arm cartesian callback: " << x << " " << y << " " << z); 
}

void HpeToUdp::leftArmCartesianCallback(const hpe_ros_msgs::CartesianArmCmd::ConstPtr& msg)
{
    ROS_INFO_ONCE("Recieved first cartesian message from left arm"); 
    float x, y, z, vx, vy, vz;
    x = msg->positionEE.x; y = msg->positionEE.y; z = msg->positionEE.z;
    vx = msg->velocityEE.x; vy = msg->velocityEE.y; vz = msg->velocityEE.z;
    armsCtl.leftArmCartesianPositionRef[0] = x;
    armsCtl.leftArmCartesianPositionRef[1] = y;
    armsCtl.leftArmCartesianPositionRef[2] = z;
    ROS_INFO_STREAM_THROTTLE(1.0, "Left arm cartesian callback: " << x << " " << y << " " << z);
}

void HpeToUdp::run()
{   
    ros::Rate r(50);

    while(ros::ok())
    {   
        
        if(pub_ready_)
        {   

            ROS_INFO_STREAM_THROTTLE(1.0, "UDP: sending messages");
            //ROS_INFO_STREAM("Socket descriptor is: " << socket_publisher_); 
            
            double t = static_cast<double>(ros::Time::now().toSec());

            if (pub_ready_){
                 int s = sendto(socket_publisher_, (const char *)&armsCtl, sizeof(ARMS_CONTROL_REFERENCES_DATA_PACKET), 0, (const struct sockaddr *) &addr_host_, sizeof(addr_host_));
            
                if (s < 0)
                {
                    ROS_ERROR("ArmsReferencesSender: could not send message!");
                    close(socket_publisher_);
                    pub_ready_ = false; 
                }

            }else{
                ROS_ERROR("ArmsReferencesSender: could not send message!");
            }
        }

        /*
        if (recv_ready_){

            
            ROS_INFO_STREAM_THROTTLE(1.0, "UDP: recv messages");

            
            int r = recvfrom(socket_receiver_, buffer, 1024, 0, (const struct sockaddr *) &recv_addr_, sizeof(recv_addr_));

            if (r == sizeof(bla)){

            }

        }

        */
        
        
        r.sleep(); 
    }
}



