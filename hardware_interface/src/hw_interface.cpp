#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

#include <iostream>
#include <vector>
#include <unistd.h>
#include <string>
#include "../include/CANCommLib/CAN_comm.h"
#include "../include/MathOpsLib/math_ops.h"

// Define these constants at the top of your hw_interface.cpp file
const float P_MIN = -12.5f; // Adjust based on your motor's range
const float P_MAX = 12.5f;  // Adjust based on your motor's range

std::vector<MotorController> motorStatus;


// Callback from subscriber
void receivedMsg(const trajectory_msgs::JointTrajectory::ConstPtr& receivedMsg) {
    if (receivedMsg->points.empty()) {
        ROS_ERROR("Received an empty trajectory point.");
        return;
    }

    size_t num_motors = 12;
    if (receivedMsg->points[0].positions.size() >= num_motors) {
        for (size_t i = 0; i < num_motors; ++i) {
            float desired_position = receivedMsg->points[0].positions[i];

            // Log for debugging
            ROS_INFO("Updating motor %lu to position %f", i, desired_position);
            if( i == 0|i == 1|i == 2|i == 6|i == 8)  {
            // Assuming the MotorController class has a method for writing CAN frames
                motorStatus[i].write_can_frame(-desired_position, 0, 5, 0.27, 0); // Example values for velocity, KP, KD, FF
            }
            else if(i==7) {

                motorStatus[i].write_can_frame(desired_position, 0, 5, 0.27, 0); // Example values for velocity, KP, KD, FF


            }
            else {
                motorStatus[i].write_can_frame(-desired_position/10, 0, 5, 0.27, 0); // Example values for velocity, KP, KD, FF
            }
        }
    } else {
        ROS_ERROR("Received trajectory point does not contain enough position elements for the motors.");
    }
}

// // Initialize all motors - assigns CAN interface, motor id, and enters motor mode
// void setupMotors() {
//     const char* interfaces[] = {"can0", "can1", "can2", "can3"};
//     int motorsPerInterface = 3;
//     int bitrate = 1000000; // Example bitrate

//     for (int i = 0; i < 12; ++i) {
//         const char* interf_name = interfaces[i / motorsPerInterface];
//         int motor_id = (i % motorsPerInterface) + 1;

//         // Initialize MotorController with interface and motor ID, then enter motor mode
        
//         open_port(interf_name, bitrate); // Assuming open_port is a method of MotorController now

//         MotorController motor(interf_name, motor_id);
        
//         motor.enter_motor_mode();
//         motor.zero_position_sensor();
//         motorStatus.push_back(motor);

//         ROS_INFO("Setup motor on interface %s with ID %d", interf_name, motor_id);

//     }
// }


void setupMotors() {
    const char* interfaces[] = {"can0", "can1", "can2", "can3"};
    int motorsPerInterface = 3;
    int bitrate = 1000000; // Example bitrate
    int motorIdOffset[] = {1, 4, 7, 10};  // Motor IDs start at 1 for can0, 4 for can1, etc.

    for (int i = 0; i < 12; ++i) {
        const char* interf_name = interfaces[i / motorsPerInterface];
        int interfaceIndex = i / motorsPerInterface;
        int motor_id = motorIdOffset[interfaceIndex] + (i % motorsPerInterface);

        open_port(interf_name, bitrate); // Assuming open_port is a method of MotorController now


        ROS_INFO("Initializing motor %d on interface %s", motor_id, interf_name);
        MotorController motor(interf_name, motor_id);
        motor.enter_motor_mode();
        //motor.zero_position_sensor();
        motorStatus.push_back(motor);

        ROS_INFO("Setup motor on interface %s with ID %d", interf_name, motor_id);
    }
}





void shutdownMotors() {
    for (auto& motor : motorStatus) {
        motor.exit_motor_mode();
        std::cout << "Exiting motor mode for motor" << std::endl;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "hw_interface");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
    ros::Subscriber sub = nh.subscribe<trajectory_msgs::JointTrajectory>("joint_group_position_controller/command", 100, receivedMsg);

    setupMotors();
    
    ROS_INFO("==========Finished setup motors interface============================");

    sensor_msgs::JointState joint_state;
    joint_state.position.resize(12);
    joint_state.velocity.resize(12);
    joint_state.effort.resize(12);
    // joint_state.name = {"motor_front_left_hip", "motor_front_left_upper_leg", "motor_front_left_lower_leg",
    //                     "motor_front_right_hip", "motor_front_right_upper_leg", "motor_front_right_lower_leg",
    //                     "motor_back_left_hip", "motor_back_left_upper_leg", "motor_back_left_lower_leg",
    //                     "motor_back_right_hip", "motor_back_right_upper_leg", "motor_back_right_lower_leg"};

    joint_state.name = {"front_left_hip_joint", "front_left_upper_leg_joint", "front_left_lower_leg_joint",
                        "rear_left_hip_joint", "rear_left_upper_leg_joint", "rear_left_lower_leg_joint",
                        "front_right_hip_joint", "front_right_upper_leg_joint", "front_right_lower_leg_joint",
                        "rear_right_hip_joint", "rear_right_upper_leg_joint", "rear_right_lower_leg_link"};
    
    
    while (ros::ok()) {
        joint_state.header.stamp = ros::Time::now();

        for (size_t i = 0; i < 12; ++i) {
            joint_state.position[i] = motorStatus[i].getPDes();
            joint_state.velocity[i] = motorStatus[i].getVDes();
            joint_state.effort[i] = motorStatus[i].getTFF();
        }

        pub.publish(joint_state);
        ros::spinOnce();
        usleep(60000); // Adjust as necessary to control the publish rate
    }

    shutdownMotors();
    ros::shutdown();

    return 0;
}
