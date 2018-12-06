#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "egm.pb.h"

#pragma comment(lib, "libprotobuf.lib")

uint32_t get_tick()
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now))
    return 0;
  return now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

void Pose_to_PoseStamped(const geometry_msgs::Pose& pose, ros::Time time, geometry_msgs::PoseStamped& posestamped)
{
  posestamped.header.stamp = time;
  posestamped.header.frame_id = "world";
  posestamped.pose = pose;
}

void EgmFeedBack_to_Pose(abb::egm::EgmFeedBack *fb, geometry_msgs::Pose& pose)
{
  pose.position.x = fb->cartesian().pos().x();
  pose.position.y = fb->cartesian().pos().y();
  pose.position.z = fb->cartesian().pos().z();
  pose.orientation.x = fb->cartesian().orient().u1();
  pose.orientation.y = fb->cartesian().orient().u2();
  pose.orientation.z = fb->cartesian().orient().u3();
  pose.orientation.w = fb->cartesian().orient().u0();
}

void EgmFeedBack_to_PoseStamped(abb::egm::EgmFeedBack *fb, geometry_msgs::PoseStamped& posestamped)
{
  posestamped.header.stamp = ros::Time::now();
  posestamped.header.frame_id = "world";
  EgmFeedBack_to_Pose(fb, posestamped.pose);
}

void EgmFeedBack_to_JointState(abb::egm::EgmFeedBack *fb, sensor_msgs::JointState& js)
{
  js.header.stamp = ros::Time::now();
  js.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  js.position.resize(6);
  for (int i = 0; i < 6; i++)
    js.position[i] = fb->joints().joints(i);
  js.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  js.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

void Position_to_EgmSensor(geometry_msgs::Pose pose, unsigned int seqno, uint32_t tick, abb::egm::EgmSensor* sensor)
{
  sensor->mutable_header()->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
  sensor->mutable_header()->set_seqno(seqno);
  sensor->mutable_header()->set_tm(tick);

  sensor->clear_planned();
  sensor->clear_speedref();
  sensor->mutable_planned()->mutable_cartesian()->mutable_pos()->set_x(pose.position.x);
  sensor->mutable_planned()->mutable_cartesian()->mutable_pos()->set_y(pose.position.y);
  sensor->mutable_planned()->mutable_cartesian()->mutable_pos()->set_z(pose.position.z);
  sensor->mutable_planned()->mutable_cartesian()->mutable_orient()->set_u0(pose.orientation.w);
  sensor->mutable_planned()->mutable_cartesian()->mutable_orient()->set_u1(pose.orientation.x);
  sensor->mutable_planned()->mutable_cartesian()->mutable_orient()->set_u2(pose.orientation.y);
  sensor->mutable_planned()->mutable_cartesian()->mutable_orient()->set_u3(pose.orientation.z);
}

void Velocity_to_EgmSensor(geometry_msgs::Pose vel, geometry_msgs::Pose pose, unsigned int seqno, uint32_t tick, abb::egm::EgmSensor* sensor)
{
  Position_to_EgmSensor(pose, seqno, tick, sensor);
  sensor->mutable_speedref()->mutable_cartesians()->add_value(vel.position.x);
  sensor->mutable_speedref()->mutable_cartesians()->add_value(vel.position.y);
  sensor->mutable_speedref()->mutable_cartesians()->add_value(vel.position.z);
  sensor->mutable_speedref()->mutable_cartesians()->add_value(vel.orientation.x);
  sensor->mutable_speedref()->mutable_cartesians()->add_value(vel.orientation.y);
  sensor->mutable_speedref()->mutable_cartesians()->add_value(vel.orientation.z);
}
