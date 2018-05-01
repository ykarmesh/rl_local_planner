#ifndef DDPG_PLANNER_H
#define DDPG_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
//#include <std_srvs/Empty.h>

#include <iostream>
#include <math.h>
// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>
 
namespace rl_local_planner
{
class DDPGPlanner : public nav_core::BaseLocalPlanner
{
public:
	DDPGPlanner();

	~DDPGPlanner();

	void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

	bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

	bool isGoalReached();

	bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

private:
	ros::NodeHandle private_nh_;

	costmap_2d::Costmap2DROS* costmap_ros_;

	std::vector<geometry_msgs::PoseStamped> global_plan_;

	bool initialized_ = false;

	geometry_msgs::PoseStamped goal_;

	double goal_limit_;

	tf::Stamped<tf::Pose> current_pose_;

	tf::TransformListener* tf_;
};
};
#endif
