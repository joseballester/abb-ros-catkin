#ifndef _EGMCONTROL_ROSHELPER_HPP_
#define _EGMCONTROL_ROSHELPER_HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

#include <vector>

class ROSHelper {
public:
  ROSHelper(ros::NodeHandle n, std::string robotname_sl);

  ~ROSHelper();

  void load_command_pose(const geometry_msgs::PoseStamped& data);

  geometry_msgs::PoseStamped get_command_pose();

  void publish_joint_state(const sensor_msgs::JointState& joints);

  void publish_measured_pose(const geometry_msgs::PoseStamped& pose);

  void publish_sent_pose(const geometry_msgs::PoseStamped& pose);

private:
  ros::Subscriber command_pose_sub;
  ros::Publisher joint_state_pub;
  ros::Publisher measured_pose_pub;
  ros::Publisher sent_pose_pub;
  geometry_msgs::PoseStamped command_pose;
};

#endif
