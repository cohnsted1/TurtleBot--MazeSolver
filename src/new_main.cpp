// ROS Libraries
#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // Motor Commands
#include "sensor_msgs/LaserScan.h" // Laser Data
#include "tf/transform_listener.h" // tf Tree

// C++ Libraries
#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>

using namespace std_msgs;

// ROS Publisher:Motor Commands, Subscriber:Laser Data, and Messages:Laser Messages & Motor Messages
ros::Publisher motor_command_publisher;
ros::Subscriber laser_subscriber;
sensor_msgs::LaserScan laser_msg;
geometry_msgs::Twist motor_command;

// Define the robot direction of movement
typedef enum _ROBOT_MOVEMENT {
    STOP = 0,
    FORWARD,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    GO_RIGHT,
    GO_LEFT

} ROBOT_MOVEMENT;

// The robot_move function will be called by the laser_callback function each time a laser scan data is received
// This function will accept robot movements and actuate the robot's wheels accordingly
// Keep a low speed for better results
bool robot_move(const ROBOT_MOVEMENT move_type)
{
    if (move_type == STOP) {
        ROS_INFO("HALT! \n");

        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.0;
    }

    else if (move_type == FORWARD) {
        ROS_INFO("Always FORWARD! \n");
        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.17;
    }

    else if (move_type == BACKWARD) {
        ROS_INFO("I'm going back! \n");
        motor_command.linear.x = -0.15;
        motor_command.angular.z = 0.75;
    }

    else if (move_type == TURN_LEFT) {
        ROS_INFO("I'm turning left! \n");
        motor_command.linear.x = 0.05;
        motor_command.angular.z = 0.5;
    }

    else if (move_type == TURN_RIGHT) {
        ROS_INFO("I'm turning right! \n");
        motor_command.linear.x = 0.05;
        motor_command.angular.z = -0.5;
    }
    else if (move_type == GO_RIGHT) {
        ROS_INFO("I'm goin right! \n");
        motor_command.linear.x = 0.15;
        motor_command.angular.z = -0.5;
    }
    else if (move_type == GO_LEFT) {
        ROS_INFO("I'm goin left! \n");
        motor_command.linear.x = 0.15;
        motor_command.angular.z = 0.5;
    }
    else {
        ROS_INFO("Move type wrong! \n");
        return false;
    }

    motor_command_publisher.publish(motor_command);
    usleep(10);
    return true;
}

bool crashed = false;

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    // Read and process laser scan values
    laser_msg = *scan_msg;
    std::vector<float> laser_ranges;
    laser_ranges = laser_msg.ranges;
    size_t range_size = laser_ranges.size();
    float left_side = 0.0, right_side = 0.0,
          fright_side = 0.0, bright_side = 0.0,
          front_side = 0.0;
    float range_min = laser_msg.range_max;

    for (size_t i = 0; i < range_size; i++) {
        if (laser_ranges[i] < range_min) {
            range_min = laser_ranges[i];
        }

        if (i > range_size / 2) {
            right_side += laser_ranges[i];
        }
        else {
            left_side += laser_ranges[i];
        }

        if (i>range_size*0.875) {
          if (laser_ranges[i] > 4) {
              front_side += 4;
          } else {
              front_side += laser_ranges[i]; //sums front section
          }
        }

        if (i < range_size*0.75 + 60 && i > range_size*0.75) {
          if (laser_ranges[i] > 4) {
              fright_side += 4;
          } else {
              fright_side += laser_ranges[i]; //sums front right section
          }

        } else if (i > range_size*0.75 - 60 && i < range_size*0.75) {
          if (laser_ranges[i] > 4) {
              bright_side += 4;
          } else {
              bright_side += laser_ranges[i]; //sums front right section
          }
        }
    }

    // Check if the robot has crashed into a wall
    if (laser_ranges[0] < 0.20) {
        crashed = true;
    }
    else {
        crashed = false;
    }

    // Assign movements to a robot that still did not crash
    if (!crashed) {

        if (range_min <= 0.20) {
            crashed = false;
            robot_move(STOP);

            if (left_side >= right_side) {
                robot_move(TURN_LEFT);
            }
            else {
                robot_move(TURN_RIGHT);
            }
        }
        else {

          if (front_side < 30) {
              robot_move(TURN_LEFT);
              ROS_INFO("Dead end \n");
          } else if (abs(fright_side - bright_side) > 12) {
              ROS_INFO("Not Parallel \n");

              if (fright_side > bright_side) {

                  robot_move(GO_RIGHT);
              } else {
                  robot_move(GO_LEFT);
              }
          } else if (laser_ranges[range_size*0.75] > 0.75) {
              ROS_INFO("Too far from wall \n");

              robot_move(GO_RIGHT);

          } else if (laser_ranges[range_size*0.75] < 0.55) {
              ROS_INFO("Too close to wall \n");

              robot_move(GO_LEFT);
          } else {
              robot_move(FORWARD);
          }
        }
    }
    // Robot should go backward since it crashed into a wall
    else {
        robot_move(BACKWARD);
    }
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "node");
    ros::NodeHandle n;
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    laser_subscriber = n.subscribe("/scan", 1000, laser_callback);
    ros::Duration time_between_ros_wakeups(0.001);

    while (ros::ok()) {
        ros::spinOnce();
        time_between_ros_wakeups.sleep();
    }

    return 0;
}
