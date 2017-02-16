//
// Simple demo program that calls the youBot ROS wrapper
//

#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <fstream>

// Positions of the gripper
#define OPENED_GRIPPER    0.011
#define CLOSED_GRIPPER  0.0015

using namespace std;

//ros::Publisher platformPublisher;
ros::Publisher armPublisher;
ros::Publisher gripperPublisher;

// Load angles from a file into a std::vector 2 dimensions
// Each line is a 'frame' in the movement of the robot
// Each value in the lines correspond to a specific joint angle
vector< vector<double> > loadAnglesValues(char* filename) {
    vector< vector<double> > outputAngles;
    string value;
    ifstream f;
    f.open(filename);
    if (f.is_open())
    {
        while (!f.eof())
        {
            vector<double> currentPoint;
            // push back the 5 angles of the robot
            for (int i=0; i<5; i++)
            {
                f >> value;
                currentPoint.push_back(atof(value.c_str()));
            }
            // push back the created robot position
            outputAngles.push_back(currentPoint);
        }
        f.close();
    }
    else
    {
        cerr << "Cannot open the file " << filename << endl;
    }
    return outputAngles;
}

// create a brics actuator message with the given joint position values
brics_actuator::JointPositions createArmPositionCommand(vector<double>& newPositions) {
    int numberOfJoints = 5;
    brics_actuator::JointPositions msg;

    if (newPositions.size() < numberOfJoints)
        return msg; // return empty message if not enough values provided

    for (int i = 0; i < numberOfJoints; i++) {
        // Set all values for one joint, i.e. time, name, value and unit
        brics_actuator::JointValue joint;
        joint.timeStamp = ros::Time::now();
        joint.value = newPositions[i];
        joint.unit = boost::units::to_string(boost::units::si::radian);

        // create joint names: "arm_joint_1" to "arm_joint_5" (for 5 DoF)
        stringstream jointName;
        jointName << "arm_joint_" << (i + 1);
        joint.joint_uri = jointName.str();

        // add joint to message
        msg.positions.push_back(joint);
    }

    return msg;
}

// create a brics actuator message for the gripper using the same position for both fingers
brics_actuator::JointPositions createGripperPositionCommand(double newPosition) {
    brics_actuator::JointPositions msg;

    brics_actuator::JointValue joint;
    joint.timeStamp = ros::Time::now();
    joint.unit = boost::units::to_string(boost::units::si::meter); // = "m"
    joint.value = newPosition;
    joint.joint_uri = "gripper_finger_joint_l";
    msg.positions.push_back(joint);
    joint.joint_uri = "gripper_finger_joint_r";
    msg.positions.push_back(joint);

    return msg;
}


// move platform a little bit back- and forward and to the left and right
/*void movePlatform() {
    geometry_msgs::Twist twist;

    // forward
    twist.linear.x = 0.05;  // with 0.05 m per sec
    platformPublisher.publish(twist);
    ros::Duration(2).sleep();

    // backward
    twist.linear.x = -0.05;
    platformPublisher.publish(twist);
    ros::Duration(2).sleep();

    // to the left
    twist.linear.x = 0;
    twist.linear.y = 0.05;
    platformPublisher.publish(twist);
    ros::Duration(2).sleep();

    // to the right
    twist.linear.y = -0.05;
    platformPublisher.publish(twist);
    ros::Duration(2).sleep();

    // stop
    twist.linear.y = 0;
    platformPublisher.publish(twist);
}*/

// move arm once up and down
/*void moveArm() {
    brics_actuator::JointPositions msg;
    vector<double> jointvalues(5);


    // move arm straight up. values were determined empirically
    jointvalues[0] = -3.147;
    jointvalues[1] = 0.56874;
    jointvalues[2] = 0.695891;
    jointvalues[3] = -1.94531;
    jointvalues[4] = 0;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();

    // move arm back close to calibration position
    jointvalues[0] = -2.94961;
    jointvalues[1] = -0.0335571;
    jointvalues[2] = 0.65143;
    jointvalues[3] = -1.29855;
    jointvalues[4] = 0;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();

    jointvalues[0] = -2.16421;
    jointvalues[1] = -1.37512 ;
    jointvalues[2] = 0.528414;
    jointvalues[3] = 0.166033;
    jointvalues[4] = 0;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();

    jointvalues[0] = -2.44251;
    jointvalues[1] = -2.52041 ;
    jointvalues[2] = 1.76995;
    jointvalues[3] = -0.97741;
    jointvalues[4] = 0;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();

    jointvalues[0] = -2.94961;
    jointvalues[1] = -1.22819 ;
    jointvalues[2] = 1.06134;
    jointvalues[3] = -1.56102;
    jointvalues[4] = 0;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();

    jointvalues[0] = -2.94961;
    jointvalues[1] = -0.84809 ;
    jointvalues[2] = 1.49785;
    jointvalues[3] = -2.34624;
    jointvalues[4] = 0;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();

    jointvalues[0] = -2.94961;
    jointvalues[1] = -0.846735 ;
    jointvalues[2] = 2.17819;
    jointvalues[3] = -3.02793;
    jointvalues[4] = 0;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();

    jointvalues[0] = -3.89309;
    jointvalues[1] = -1.27728 ;
    jointvalues[2] = 1.8776;
    jointvalues[3] = -2.2968;
    jointvalues[4] = 0;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(2).sleep();
}*/

// Initialize the robot at a starting position (completely folded on itself)
void initializeArm()
{
    brics_actuator::JointPositions msg;
    vector<double> jointvalues(5);

    jointvalues[0] = 0.012;
    jointvalues[1] = 0.012;
    jointvalues[2] = -0.158;
    jointvalues[3] = 0.023;
    jointvalues[4] = 0.111;
    msg = createArmPositionCommand(jointvalues);
    armPublisher.publish(msg);

    ros::Duration(5).sleep();
}

// Move the gripper to a new position
void moveGripper(double position)
{
    brics_actuator::JointPositions msg;
    msg = createGripperPositionCommand(position);
    gripperPublisher.publish(msg);

    ros::Duration(3).sleep();
}

int main(int argc, char **argv) {
    cout << "Loading values from " << argv[1] << "..." << endl;
    // Load the requested action
    vector< vector<double> > values = loadAnglesValues(argv[1]);
    cout << "Initialize..." << endl;

    // Initialize ros environement
    ros::init(argc, argv, "object_displacer");
    ros::NodeHandle n;

    //platformPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    armPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);
    gripperPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);
    sleep(1);

    // Initialize robot at constant starting position
    moveGripper(CLOSED_GRIPPER);
    initializeArm();
    brics_actuator::JointPositions msg;
    cout << "Starting" << endl;
    // Open the gripper
    moveGripper(OPENED_GRIPPER);
    // Achieve the 2 first positions
    for (int i=0; i<2; i++)
    {
        msg = createArmPositionCommand(values[i]);
        cout << "Point n°" << i << ":\t";
        for (int j=0; j<5; j++)
        {
            cout << values[0][j] << "\t";
        }
        cout << endl;
        armPublisher.publish(msg);

        ros::Duration(2).sleep();
    }
    // Close the gripper (taking the object)
    moveGripper(CLOSED_GRIPPER);
    // Achieve the remaining positions in the list (move with the object)
    for (int i=2; i<values.size(); i++)
    {
        msg = createArmPositionCommand(values[i]);
        cout << "Point n°" << i << ":\t";
        for (int j=0; j<5; j++)
        {
            cout << values[i][j] << "\t";
        }
        cout << endl;
        armPublisher.publish(msg);

        ros::Duration(1).sleep();
    }
    // Open the gripper (release the object)
    moveGripper(OPENED_GRIPPER);

    //movePlatform();
    //moveArm();

    sleep(1);
    // Close ros environement
    ros::shutdown();

    return 0;
}
