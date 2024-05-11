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
void receivedMsg(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
    if (msg->points.empty()) return; // Ensure there's at least one point to read from
    
    for (size_t i = 0; i < motorStatus.size() && i < msg->points[0].positions.size(); ++i) {
        // Update desired positions, velocities, and efforts
        motorStatus[i].p_des = msg->points[0].positions[i]; // Assuming radians
        motorStatus[i].v_des = msg->points[0].velocities[i]; // Assuming rad/s
        motorStatus[i].t_ff = msg->points[0].effort[i]; // Assuming Nm or similar

        // You might need to convert from radians to the unit your actuators expect (e.g., degrees)
        // Similarly for velocity and effort, depending on your actuator requirements

        // Convert to appropriate units if necessary
        // Here we directly use the values assuming they are in correct units for simplicity
        int pos = static_cast<int>(motorStatus[i].p_des);
        int vel = static_cast<int>(motorStatus[i].v_des);
        int effort = static_cast<int>(motorStatus[i].t_ff);

        // Send CAN command to motor
        write_can_frame(motorStatus[i].interf_name.c_str(), motorStatus[i].motor_id, pos, vel, motorStatus[i].kp, motorStatus[i].kd, effort, motorStatus[i]);
    }
}




// initialize all motors - assigns CAN interface, motor id, and enters motor mode
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

        zero_position_sensor(interf_name, motorStatus[i].motor_id, motorStatus[i]);
    }
}









/////////////////////////////////////////////////////////////////
void setupMotors() {
    const int numberOfMotors = 12; // Assuming 12 motors
    std::vector<std::string> interfaces = {"can0", "can1", "can2", "can3"};
    
    for (int i = 0; i < numberOfMotors; ++i) {
        MotorStatusStruct motor;
        motor.name = "M" + std::to_string(i); // Set name of motor
        
        // Assign CAN interface based on motor index
        int interfaceIndex = i / 3; // This will group every 3 motors to the same interface (0-2, 3-5, 6-8, 9-11)
        motor.interf_name = interfaces[interfaceIndex];
        
        // Assign motor ID (1 to 3) within each group
        motor.motor_id = (i % 3) + 1;
        
        // Open port and enter motor mode for each motor
        // Assuming open_port and enter_motor_mode functions handle the opening and setting modes correctly
        open_port(motor.interf_name.c_str(), 1000000); // Example baud rate
        
        // The motor object is passed by reference, allowing enter_motor_mode to modify its properties if needed
        enter_motor_mode(motor.interf_name.c_str(), motor.motor_id, motor);
        
        // Add configured motor to the motorStatus vector
        motorStatus.push_back(motor);
    }
}

//////////////////////////////////////////////////////////////////////////



void shutdownMotors() {
    for (auto& motor : motorStatus) {
        exit_motor_mode(motor.interf_name.c_str(), motor.motor_id, motor);
        std::cout << "Exiting motor mode for motor ID: " << motor.motor_id << " on interface: " << motor.interf_name << std::endl;
    }
}




int main(int argc, char** argv) {
    ros::init(argc, argv, "hw_interface");
    ros::NodeHandle nh;

    // Motor names should match exactly what's expected in your robot's URDF
    std::vector<std::string> motor_names = {
        "motor_front_left_hip", "motor_front_left_upper_leg", "motor_front_left_lower_leg",
        "motor_front_right_hip", "motor_front_right_upper_leg", "motor_front_right_lower_leg",
        "motor_back_left_hip", "motor_back_left_upper_leg", "motor_back_left_lower_leg",
        "motor_back_right_hip", "motor_back_right_upper_leg", "motor_back_right_lower_leg"
    };

    setupMotors(); // Ensure motors are setup before entering the loop



    ros::on_shutdown(shutdownMotors); // Ensure motors are shutdown when the node is killed



    sensor_msgs::JointState joint_state;
    joint_state.name = motor_names;
    joint_state.position.resize(motorStatus.size());
    joint_state.velocity.resize(motorStatus.size());
    joint_state.effort.resize(motorStatus.size());

    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
    ros::Subscriber sub = nh.subscribe<trajectory_msgs::JointTrajectory>("joint_group_position_controller/command", 100, receivedMsg);

    ros::Rate loop_rate(50); // Loop at 50Hz, adjust as necessary

    //Control command using write_to_can

    //How many elements are in the motorStatus?

    //Works for simulation
    while (ros::ok()) {
        joint_state.header.stamp = ros::Time::now();
        for (size_t i = 0; i < motorStatus.size(); ++i) {

            joint_state.position[i] = motorStatus[i].p_des; // Assuming these are already in the correct units
            joint_state.velocity[i] = motorStatus[i].v_des;
            joint_state.effort[i] = motorStatus[i].t_ff;


            // // Convert to appropriate units if necessary
            // int pos = static_cast<int>(motorStatus[i].p_des);
            // int vel = static_cast<int>(motorStatus[i].v_des);
            // int effort = static_cast<int>(motorStatus[i].t_ff);

            // // Continuously send CAN commands to motors
            // write_can_frame(motorStatus[i].interf_name.c_str(), motorStatus[i].motor_id, pos, vel, motorStatus[i].kp, motorStatus[i].kd, effort, motorStatus[i]);
        


                //writing CAN frame
            //write_can_frame("can0", 0x02 , motorStatus[2].p_des, motorStatus[2].v_des, 20, 3, motorStatus[2].t_ff, motorStatus[2]);
        }

        pub.publish(joint_state);
        ros::spinOnce(); // Handle callback functions
        loop_rate.sleep(); // Sleep to maintain loop rate
    }

    return 0;
}