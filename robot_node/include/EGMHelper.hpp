#ifndef _EGMCONTROL_EGMHELPER_HPP_
#define _EGMCONTROL_EGMHELPER_HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "egm.pb.h"

#include <vector>
#include <string>

uint32_t get_tick();

void Pose_to_PoseStamped(const geometry_msgs::Pose& pose, ros::Time time, geometry_msgs::PoseStamped& posestamped);

void EgmFeedBack_to_Pose(abb::egm::EgmFeedBack *fb, geometry_msgs::Pose& pose);

void EgmFeedBack_to_PoseStamped(abb::egm::EgmFeedBack *fb, geometry_msgs::PoseStamped& posestamped);

void EgmFeedBack_to_JointState(abb::egm::EgmFeedBack *fb, sensor_msgs::JointState& js);

void Position_to_EgmSensor(geometry_msgs::Pose pose, unsigned int seqno, uint32_t tick, abb::egm::EgmSensor* sensor);

void Velocity_to_EgmSensor(geometry_msgs::Pose vel, geometry_msgs::Pose pose, unsigned int seqno, uint32_t tick, abb::egm::EgmSensor* sensor);

#endif
