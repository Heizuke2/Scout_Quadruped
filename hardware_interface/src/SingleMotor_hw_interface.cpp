#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

#include <iostream>
#include <stdio.h>
#include <thread>         
#include <vector>         
#include <unistd.h>
#include <string>
#include <string.h>

#include <std_msgs/String.h>
#include "../include/CANCommLib/CAN_comm.h"
#include "../include/MathOpsLib/math_ops.h"

// Define these constants at the top of your hw_interface.cpp file
const float P_MIN = -12.5f; // Adjust based on your motor's range
const float P_MAX = 12.5f;  // Adjust based on your motor's range

/*
    Subscribe to trajectory_msgs/JointTrajectory . This ROS message contains the joint angles (12DOF) that the actuators can use to move the robot.

    Publish all the actuators' current angle using sensor_msgs/JointState to 'joint_states' topic.

    Control the actuators and read its angle (optional) programmatically.
*/


/*
struct MotorStatusStruct
{
    std::string name;
    int motor_id;
    int p_des, v_des, kp,kd,t_ff;
};
*/

std::vector<MotorStatusStruct> motorStatus;

/* may need it later 
float radToDeg(float rad){
    float pi = 3.1415926535;

    return float(rad * (180/pi));
}
*/


//callback from subscriber
// void receivedMsg(const trajectory_msgs::JointTrajectory::ConstPtr& receivedMsg) {
//     if (receivedMsg->points.empty()) {
//         ROS_ERROR("Received an empty trajectory point.");
//         return;
//     }

//     size_t num_motors = 3; // Adjust this based on how many motors you're controlling
//     if (receivedMsg->points[0].positions.size() >= num_motors) {
//         for (size_t i = 0; i < num_motors; ++i) {
//             float desired_position = receivedMsg->points[0].positions[i]/ 4.0 ;
//             ROS_INFO("Updating motor %lu to position %f", i, desired_position);

//             // Convert the desired position to the format expected by your CAN communication
//             int pos_int = float_to_uint(desired_position, P_MIN, P_MAX, 16);

//             // Control parameters - adjust these as necessary
//             int KP = 5; // Placeholder - adjust to your needs
//             int KD = 3; // Placeholder - adjust to your needs
//             int FF = 1; // Placeholder - adjust to your needs

//             // Determine which interface name and CAN ID to use for each motor
//             const char* interface_name = "can0"; // Example, adjust as needed
//             int id_can = motorStatus[i].motor_id; // Assuming motor_id is the CAN ID

//             // Send the command to the motor
//             // Corrected function call with dummy values for velocity and FF for demonstration
//             int vel = 0; // Assuming you're not setting velocity here
//             write_can_frame(interface_name, id_can, pos_int, vel, KP, KD, FF, motorStatus[i]);

//             // write_can_frame(interface_name, id_can, pos_int, KP, KD, FF, motorStatus[i]);
//             // write_can_frame(interface_name, id_can, pos, vel, KP, KD, FF, motorStatus[i]);

//             // Debugging: log the converted position and control parameters being sent
//             ROS_INFO("Converted pos for motor %lu: %d, KP: %d, KD: %d, FF: %d", i, pos_int, KP, KD, FF);
//         }
//     } else {
//         ROS_ERROR("Received trajectory point does not contain enough position elements for the motors.");
//     }
// }

void receivedMsg(const trajectory_msgs::JointTrajectory::ConstPtr& receivedMsg) {
    if (receivedMsg->points.empty()) {
        ROS_ERROR("Received an empty trajectory point.");
        return;
    }

    // Assuming we are only interested in the first position for the single motor
    if (!receivedMsg->points[0].positions.empty()) {
        float desired_position = receivedMsg->points[0].positions[0];
        ROS_INFO("Updating motor to position %f", desired_position);

        // Convert the desired position to the format expected by your CAN communication
        int pos_int = float_to_uint(desired_position, P_MIN, P_MAX, 16);

        // Control parameters - adjust these as necessary
        int KP = 5; // Placeholder - adjust to your needs
        int KD = 3; // Placeholder - adjust to your needs
        int FF = 1; // Placeholder - adjust to your needs

        // Specify the interface name and CAN ID for the single motor
        const char* interface_name = "can0"; // Example, adjust as needed
        int id_can = 1; // Example CAN ID for the single motor, adjust as needed

        // Send the command to the motor
        //write_can_frame(interface_name, id_can, pos_int, KP, KD, FF, motorStatus[0]);
        int vel = 0; // Assuming you're not setting velocity here
        write_can_frame(interface_name, id_can, pos_int, vel, KP, KD, FF, motorStatus[0]);

        // Debugging: log the converted position and control parameters being sent
        ROS_INFO("Converted pos for motor: %d, KP: %d, KD: %d, FF: %d", pos_int, KP, KD, FF);
    } else {
        ROS_ERROR("Received trajectory point does not contain position elements.");
    }
}





//initialize all motors - assigns CAN interface, motor id, and enters motor mode
void setupMotors(){
    for (int i = 0; i < 12; ++i){
        motorStatus.push_back(MotorStatusStruct());
        motorStatus[i].name = "M"+ std::to_string(i); // set name of motor
        
        const char *interf_name;
        // Each leg uses a different CAN interface
        switch(i) {
            case 0 ... 2:
                interf_name = "can0";
                break;
            case 3 ... 5:
                interf_name = "can1";
                break;
            case 6 ... 8:
                interf_name = "can2";
                break;
            case 9 ... 11:
                interf_name = "can3";
                break;
        }
        
        motorStatus[i].motor_id = (i % 3) + 1;
        
        open_port(interf_name, 1000000);
        
        // Initialize motors with default values
        enter_motor_mode(interf_name, motorStatus[i].motor_id, motorStatus[i]);
    
    
    
    // ROS_INFO("Received trajectory message.");
    // // Inside your loop or critical sections
    // ROS_INFO("Processing motor index: %d", i);


        //zero_position_sensor(interf_name, motorStatus[i].motor_id, motorStatus[i]);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "hw_interface");
    ros::NodeHandle nh;
    
    std::vector<std::string> CAN_interfs = {"can0", "can1", "can2", "can3"};
    
    sensor_msgs::JointState joint_state;
    joint_state.position.resize(12);
    joint_state.velocity.resize(12);
    joint_state.effort.resize(12);              //same as t_ff? 
    joint_state.name = {"motor_front_left_hip", "motor_front_left_upper_leg", "motor_front_left_lower_leg", "motor_front_right_hip", "motor_front_right_upper_leg", "motor_front_right_lower_leg", "motor_back_left_hip", "motor_back_left_upper_leg", "motor_back_left_lower_leg", "motor_back_right_hip", "motor_back_right_upper_leg", "motor_back_right_lower_leg"};
    
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
    ros::Subscriber sub = nh.subscribe<trajectory_msgs::JointTrajectory>("joint_group_position_controller/command", 100, &receivedMsg);
    
    setupMotors();


    while(ros::ok()){
        joint_state.header.stamp = ros::Time::now();
    // Now only update and publish the three motors you're working with
        // Update and publish the state for the single motor
        joint_state.position[0] = motorStatus[0].p_des; // Assuming motorStatus[0] is your single motor
        joint_state.velocity[0] = motorStatus[0].v_des;
        joint_state.effort[0] = motorStatus[0].t_ff;

    //pub.publish(joint_state);
    //ros::spinOnce(); // Handle callback functions
        //ros::spin();
    
    pub.publish(joint_state);
    ros::spinOnce(); // Ensure this is uncommented to process callbacks
    usleep(100000); // Adjust as necessary to control the publish rate
}
}
