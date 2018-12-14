#include "RobotHelper.hpp"
#include "EGMHelper.hpp"
#include "egm.pb.h"

RobotHelper::RobotHelper(ros::NodeHandle n, int udpPort)
  : seqno(0), start_tick(get_tick()), udpPort(udpPort), egm_first(true)
{
  sock = new UDPSocket(udpPort);
  sock->setBlocking();
  last_egm_robot = new abb::egm::EgmRobot();
  last_egm_sensor = new abb::egm::EgmSensor();
}

RobotHelper::~RobotHelper()
{
  delete sock;
}

void RobotHelper::connect(std::string command_mode)
{
  sock->setBlocking();
  last_measured_ps = geometry_msgs::PoseStamped();
  if(!egm_first) send_command(last_measured_ps, command_mode);
  egm_first = false;
  flush_robot_data();
  last_sent_ps = last_measured_ps;
  last_sent_seq = 0;
  sock->setTimeout(2, 500000);
}

void RobotHelper::disconnect() {
  sock->setBlocking();
}

void RobotHelper::flush_robot_data()
{
  messageSize = sock->recvFrom(inBuffer, MAX_ROBOT_BUFFER-1, sourceAddr, sourcePort);

  if (messageSize < 0) {
    // TODO: Should be improved
    ROS_INFO("ROBOT_CONTROLLER: Failed to receive EGM message from robot.");
    return;
  }
  last_egm_robot->ParseFromArray(inBuffer, messageSize);
  last_fb = last_egm_robot->feedback();
  EgmFeedBack_to_PoseStamped(&last_fb, last_measured_ps);
  EgmFeedBack_to_JointState(&last_fb, last_measured_js);
}

void RobotHelper::get_measured_pose(geometry_msgs::PoseStamped& posestamped)
{
  posestamped = last_measured_ps;
}

void RobotHelper::get_measured_js(sensor_msgs::JointState& js)
{
  js = last_measured_js;
}

geometry_msgs::PoseStamped RobotHelper::send_command(geometry_msgs::PoseStamped command_pose, std::string command_mode)
{
  new_sent_time = ros::Time::now();
  if (command_mode == "velocity") {
    if(command_pose.header.seq == last_sent_seq) {
      target = geometry_msgs::Pose();
    } else {
      target = command_pose.pose;
      last_sent_seq = command_pose.header.seq;
    }
    Velocity_to_EgmSensor(target, last_measured_ps.pose, seqno++, get_tick()-start_tick, last_egm_sensor);
  } else {
    if (command_pose.header.seq == last_sent_seq) {
      target = last_sent_ps.pose;
    } else {
      target = command_pose.pose;
      last_sent_seq = command_pose.header.seq;
    }
    Position_to_EgmSensor(target, seqno++, get_tick()-start_tick, last_egm_sensor);
  }
  Pose_to_PoseStamped(target, new_sent_time, last_sent_ps);
  last_egm_sensor->SerializeToString(&outBuffer);
  sock->sendTo(outBuffer.c_str(), outBuffer.length(), sourceAddr, sourcePort);
  return last_sent_ps;
}
