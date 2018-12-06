#include "ROSHelper.hpp"

ROSHelper::ROSHelper(ros::NodeHandle n, std::string robotname_sl)
{
  command_pose_sub = n.subscribe(robotname_sl + "_EGM/SetCartesian", 1, &ROSHelper::load_command_pose, this);
  joint_state_pub = n.advertise<sensor_msgs::JointState>(robotname_sl + "_EGM/GetJoints", 1);
  measured_pose_pub = n.advertise<geometry_msgs::PoseStamped>(robotname_sl + "_EGM/GetCartesian", 1);
  sent_pose_pub = n.advertise<geometry_msgs::PoseStamped>(robotname_sl + "_EGM/GetSentCartesian", 1);
}

ROSHelper::~ROSHelper()
{
}

void ROSHelper::load_command_pose(const geometry_msgs::PoseStamped& data)
{
  command_pose = data;
}

geometry_msgs::PoseStamped ROSHelper::get_command_pose()
{
  return command_pose;
}

void ROSHelper::publish_joint_state(const sensor_msgs::JointState& joints)
{
  joint_state_pub.publish(joints);
}

void ROSHelper::publish_measured_pose(const geometry_msgs::PoseStamped& pose)
{
  measured_pose_pub.publish(pose);
}

void ROSHelper::publish_sent_pose(const geometry_msgs::PoseStamped& pose)
{
  sent_pose_pub.publish(pose);
}
