#ifndef _TRAJECTORY_PLAN_HPP
#define _TRAJECTORY_PLAN_HPP
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// ROS
#include <ros/ros.h>
#include <math.h>
// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

using std::vector;

/* void allocateTCPPose
 * \brief
 * allocate recorded TCP poses to waypoints
 */
void allocateTCPPose(double *P, geometry_msgs::Pose &pose)
{
  pose.position.x = P[0];
  pose.position.y = P[1];
  pose.position.z = P[2];
  tf2::Quaternion orientation; 
  orientation.setRPY(P[3], P[4], P[5]); //TCP pose(tool0 pose)
  orientation.normalize(); //normalize the Quaternion
  pose.orientation = tf2::toMsg(orientation);
}

/* void TCP_To_eelink
 * \brief
 * convert TCP(tool0) pose to ee_link pose
 */
void TCP_To_eelink(vector<geometry_msgs::Pose> &waypoints)
{
  tf2::Quaternion TCP, rotation, eelink;
  rotation.setRPY(M_PI/2, 0, M_PI/2);
  //rotation.setRPY(-M_PI/2, M_PI/2, 0);

  for(auto & points : waypoints)
  {
    tf2::convert(points.orientation , TCP);
    eelink = rotation*TCP;
    eelink.normalize();
    tf2::convert(eelink, points.orientation);
  }
}

/* void planTraj
 * \brief
 * plan trajetory segment
 */
void planTraj(vector<geometry_msgs::Pose> &waypoints, 
              moveit::planning_interface::MoveGroupInterface &ur5)
{
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = 0.0;
  int maxtries = 10000;   //最大尝试规划次数
  int attempts = 0;     //已经尝试规划次数

  while(fraction < 1 && attempts < maxtries)
  {
      fraction = ur5.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      ++attempts;

      if(attempts % 10 == 0)
      { 
        ROS_INFO("fraction of the path achieved: %0.2f", fraction);
        ROS_INFO("Still trying after %d attempts...", attempts);
      }
      else ROS_INFO("fraction of the path achieved: %0.2f", fraction);
  }

  if(fraction == 1)
  {
      ROS_INFO("Path computed successfully. Moving the arm.");

      // 生成机械臂的运动规划数据
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;

      // 执行运动
      ur5.execute(plan);
      ros::WallDuration(1.0).sleep();
  }
  else
  {
      ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
  }
}

void planTrajDirect(vector<geometry_msgs::Pose> &waypoints, 
                    moveit::planning_interface::MoveGroupInterface &ur5)
{
    int maxtries = 10000;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    for(vector<geometry_msgs::Pose>::iterator waypointIter = waypoints.begin(); waypointIter!=waypoints.end(); ++waypointIter)
    {
        bool success = 0;
        ur5.setPoseTarget(*waypointIter);
        while(!success && attempts < maxtries)
        {
            success = (ur5.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            attempts++;

            if(attempts % 10 == 0)
                ROS_INFO("Still trying after %d attempts...", attempts);
        }
        
        if(success)
        {
            ROS_INFO("Path computed successfully. Moving the arm.");
            // 执行运动
            ur5.move();
            ros::WallDuration(1.0).sleep();
        }
        else
        {
            ROS_INFO("Path planning failed with only %d success after %d attempts.", int(success), maxtries);
        }
    }
}

#endif