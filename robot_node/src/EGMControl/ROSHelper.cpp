#include "ROSHelper.hpp"

ROSHelper::ROSHelper(ros::NodeHandle n, std::string robotname_sl)
{
  command_pose_sub = n.subscribe(robotname_sl + "_EGM/SetCartesian", 100, &ROSHelper::load_command_pose, this);
  joint_state_pub = n.advertise<sensor_msgs::JointState>(robotname_sl + "_EGM/GetJoints", 100);
  measured_pose_pub = n.advertise<geometry_msgs::PoseStamped>(robotname_sl + "_EGM/GetCartesian", 100);
  sent_pose_pub = n.advertise<geometry_msgs::PoseStamped>(robotname_sl + "_EGM/GetSentCartesian", 100);
  command_poses = std::vector<geometry_msgs::PoseStamped>();
  max_queued = 0;
}

ROSHelper::~ROSHelper()
{
  command_poses.clear();
}

void ROSHelper::load_command_pose(const geometry_msgs::PoseStamped& data)
{
  command_poses.push_back(data);
}

geometry_msgs::PoseStamped ROSHelper::get_command_pose()
{
  if(command_poses.size() > 0) {
    last_command_ps = command_poses[0];
    command_poses.erase(command_poses.begin());
    return last_command_ps;
  } else {
    return geometry_msgs::PoseStamped();
  }
}

void ROSHelper::publish_joint_state(const sensor_msgs::JointState joints)
{
  joint_state_pub.publish(joints);
}

void ROSHelper::publish_measured_pose(const geometry_msgs::PoseStamped pose)
{
  measured_pose_pub.publish(pose);
}

void ROSHelper::publish_sent_pose(const geometry_msgs::PoseStamped pose)
{
  sent_pose_pub.publish(pose);
}

int ROSHelper::get_max_queued()
{
  return max_queued;
}
