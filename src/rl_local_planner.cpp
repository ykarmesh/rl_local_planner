#include "rl_local_planner/rl_local_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rl_local_planner::DDPGPlanner, nav_core::BaseLocalPlanner)

namespace rl_local_planner{

  	DDPGPlanner::DDPGPlanner() 
	:initialized_(false), costmap_ros_(NULL){}
	
	DDPGPlanner::~DDPGPlanner()
	{
		delete costmap_ros_;
	}

	void DDPGPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){	//initialize the local planner
		if(!initialized_){
			ros::NodeHandle private_nh("~/" + name);	
			private_nh_ = private_nh;
			costmap_ros_ = costmap_ros;
			initialized_ = true;
			tf_ = tf;
			costmap_ros_->getRobotPose(current_pose_);
			private_nh_.param("goal_limit", goal_limit_, 0.2);
			
		}			
		else{
			ROS_WARN("This planner has already been initialized... doing nothing");
		}
	}

	bool DDPGPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){ 	//brings the global planner to the local planner. get it and do nothing I suppose
		if (!initialized_) {
      			ROS_ERROR("[Set_Plan]This planner has not been initialized, please call initialize() before using this planner");
      			return false;
    		}
		global_plan_ = plan;
		
		if (global_plan_.empty())
		{
			ROS_ERROR("Received plan with zero length");
			return false;
		}
		goal_ = global_plan_.back();
		//Make sure that the global planner is not publishing intermediate poses otherwise goal based reward will be wrong
		return true;
	}

	bool DDPGPlanner::isGoalReached(){	//tells movebase that the goal has been reached and to stop the planner
		if (!initialized_) {
			ROS_ERROR("[Goal_Reached]This planner has not been initialized, please call initialize() before using this planner");
			return false;
		}
		if (!costmap_ros_->getRobotPose(current_pose_)) {
      			ROS_ERROR("Could not get robot pose");
      			return false;
		}
		geometry_msgs::PoseStamped current_pose_msg;
		tf::poseStampedTFToMsg(current_pose_, current_pose_msg);
		double distance = pow(pow(current_pose_msg.pose.position.x - goal_.pose.position.x,2) + pow(current_pose_msg.pose.position.y - goal_.pose.position.y,2),1/2);
		if (distance < goal_limit_) {
			ROS_INFO("Goal reached");
			return true;
		} 
			else {
			return false;
		}
	}

	bool DDPGPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){	//send the velocity command and publish the local costmap
		if(!initialized_){
      			ROS_ERROR("[Cmd_Vel]This planner has not been initialized, please call initialize() before using this planner");
      			return false;
		}
		geometry_msgs::TwistConstPtr msg = ros::topic::waitForMessage<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", ros::Duration(5));
		//2)use wait for message to check if something was published by gym_gazebo
		//could also check if published values are too fluctuating and stop that
		//How to handle node based goals and the final goal differently
		std::cout<<msg;
		if(msg != NULL){
			//ROS_INFO("cmd velocity was sent");
			return true;
		}
		else{
			//ROS_INFO("No cmd velocity was sent");
			return false;
		}
    	}
}
