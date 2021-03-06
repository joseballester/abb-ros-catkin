#ifndef _EGMCONTROL_RobotHelper_HPP_
#define _EGMCONTROL_RobotHelper_HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "PracticalSocket.h"
#include "egm.pb.h"

#include <vector>
#include <string>

typedef std::pair<double, double> limits;

class RobotHelper {
public:
  RobotHelper(ros::NodeHandle n, int udpPort);
  ~RobotHelper();

  void connect(std::string command_mode);
  void disconnect();
  void flush_robot_data();
  void get_measured_pose(geometry_msgs::PoseStamped& posestamped);
  void get_measured_js(sensor_msgs::JointState& js);

  abb::egm::EgmFeedBack get_robot_feedback();
  geometry_msgs::PoseStamped send_command(geometry_msgs::PoseStamped command_pose, std::string command_mode);

private:
  unsigned int seqno;
  uint32_t start_tick;
  UDPSocket* sock;
  int udpPort;
  const static int MAX_ROBOT_BUFFER = 1400;
  char inBuffer[MAX_ROBOT_BUFFER];
  std::string outBuffer;
  int messageSize;
  std::string sourceAddr;
  unsigned short sourcePort;
  limits x_limits;
  limits y_limits;
  limits z_limits;
  int last_sent_seq;
  bool egm_first;

  geometry_msgs::PoseStamped last_measured_ps;
  sensor_msgs::JointState last_measured_js;
  geometry_msgs::PoseStamped last_sent_ps;
  ros::Time new_sent_time;
  geometry_msgs::Pose target;
  abb::egm::EgmFeedBack last_fb;
  abb::egm::EgmRobot* last_egm_robot;
  abb::egm::EgmSensor* last_egm_sensor;
};

#endif
