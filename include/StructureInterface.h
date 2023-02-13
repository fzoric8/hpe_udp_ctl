
typedef struct
{
	uint8_t mode;
	double leftArmGripperPositionRef;
	double leftArmJointPositionRef[4];
	double leftArmCartesianPositionRef[3];
	double leftArmJointTorqueRef[4];
	double leftArmForce[3];
	double rightArmGripperPositionRef;
	double rightArmJointPositionRef[4];
	double rightArmCartesianPositionRef[3];
	double rightArmJointTorqueRef[4];
	double rightArmForce[3];
	double timeStamp;
} ARMS_CONTROL_REFERENCES_DATA_PACKET;

