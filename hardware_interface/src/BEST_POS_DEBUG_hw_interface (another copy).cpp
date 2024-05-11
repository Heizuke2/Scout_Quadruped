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
void receivedMsg(const trajectory_msgs::JointTrajectory::ConstPtr& receivedMsg){
    //assume position is in radians for now (9.27.2022)
    if (receivedMsg->points.empty()) {
        ROS_ERROR("Received an empty trajectory point.");
        return;
    }

    // Log the size of received trajectory points for debugging
    ROS_INFO("Received trajectory with %lu positions, %lu velocities, %lu efforts",
             receivedMsg->points[0].positions.size(),
             receivedMsg->points[0].velocities.size(),
             receivedMsg->points[0].effort.size());
             
    // Log each position value
    for (size_t i = 0; i < receivedMsg->points[0].positions.size(); ++i) {
        ROS_INFO("Position %lu: %f", i, receivedMsg->points[0].positions[i]);
    }


    size_t expected_elements = 3; // Now expecting 3 elements for testing
    if (receivedMsg->points[0].positions.size() < expected_elements ||
        receivedMsg->points[0].velocities.size() < expected_elements ||
        receivedMsg->points[0].effort.size() < expected_elements) {
        ROS_ERROR("Received trajectory point does not contain enough elements.");
        return;
    }

    //ROS_INFO("Received trajectory with %lu positions", receivedMsg->points[0].positions.size());


    // Loop over the elements you do have
    for (size_t i = 0; i < expected_elements; i++) {
        // Your existing logic to handle the motors
        motorStatus[i].p_des = receivedMsg->points[0].positions[i];
        motorStatus[i].v_des = receivedMsg->points[0].velocities[i];
        motorStatus[i].t_ff = receivedMsg->points[0].effort[i];
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
    for (size_t i = 0; i < 3; ++i) {
        joint_state.position[i] = motorStatus[i].p_des;
        joint_state.velocity[i] = motorStatus[i].v_des;
        joint_state.effort[i] = motorStatus[i].t_ff;
    }

    //pub.publish(joint_state);
    //ros::spinOnce(); // Handle callback functions
        //ros::spin();
    
    pub.publish(joint_state);
    ros::spinOnce(); // Ensure this is uncommented to process callbacks
    usleep(100000); // Adjust as necessary to control the publish rate
}
}
