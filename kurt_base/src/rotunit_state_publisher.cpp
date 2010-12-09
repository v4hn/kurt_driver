#include "kurt2can.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <signal.h>
#include <stdlib.h>

int k_can_init(void)
{
  int can_version;
  char *retext; // return text of CAN interface functions

  if ((retext = can_init(&can_version))) {
    ROS_INFO("k_can_init %s", retext);
    return 1;
  }
  ROS_INFO("CAN interface version %3d.%02d",
      can_version / 100, can_version % 100);
  return 0;
}

void k_can_close(void)
{
  char *retext; // return text of CAN interface functions
  can_rotunit_send(0);
  if ((retext = can_close())) {
    ROS_WARN("k_can_close %s", retext);
  }
}

void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  can_rotunit_send(msg->angular.x);
}

void quit(int sig)
{
  ROS_INFO("close");
  k_can_close();
  exit(0);
}

int main(int argc, char** argv)
{
  int speed;
  ros::init(argc, argv, "rotunit_state_publisher");
  ros::NodeHandle n;
  ros::NodeHandle nh_ns("~");
  nh_ns.param("InitialSpeed", speed, 42);

  ros::Rate loop_rate(100);
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 10, velCallback);
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  sensor_msgs::JointState joint_state;
  joint_state.name.resize(1);
  joint_state.position.resize(1);
  joint_state.name[0] ="laser_rot_joint";

  if (k_can_init() > 0) {
    ROS_ERROR("can not init can");
    return 0;
  }
  can_rotunit_send(speed);
  signal(SIGINT, quit);

  int rot = 0;

  while (ros::ok()) {
    can_getrotunit(&rot);

    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = rot * 2 * M_PI / 10240;

    joint_pub.publish(joint_state);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

