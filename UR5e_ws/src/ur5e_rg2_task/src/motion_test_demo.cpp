#include <math.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<iostream>
#include<vector>
using namespace std;

/* void planTrajInterpolate
 * \brief
 * plan trajetory segment
 */
void planTrajInterpolate(vector<geometry_msgs::Pose> &waypoints, 
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
      sleep(1);
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
            ros::WallDuration(8.0).sleep();
        }
        else
        {
            ROS_INFO("Path planning failed with only %d success after %d attempts.", int(success), maxtries);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_test_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface ur5("manipulator");
    geometry_msgs::Pose init_pose = ur5.getCurrentPose().pose;

    string eef_link = ur5.getEndEffector();
    std::string reference_frame = "base_link";
    ur5.setPoseReferenceFrame(reference_frame);

    ur5.allowReplanning(true);

    ur5.setGoalPositionTolerance(0.01);
    ur5.setGoalOrientationTolerance(0.01);
    ur5.setMaxAccelerationScalingFactor(0.5);
    ur5.setMaxVelocityScalingFactor(0.3);
    double c=0.56;//
    
    geometry_msgs::Pose target_pose;
   
   // ur5.setPoseTarget(target_pose);
  //  ur5.move();

    vector<geometry_msgs::Pose> waypoints;
    tf2::Quaternion goal_orientation;

    double test_pose[6]={0.088, 0.422, 0.216, 2.4314, 1.5637, 2.512};

    goal_orientation.setRPY(test_pose[3], test_pose[4], test_pose[5]);
    goal_orientation.normalize();
    target_pose.orientation = tf2::toMsg(goal_orientation);
    // target_pose.orientation.w = 0.707;
    // target_pose.orientation.x = 0.0;
    // target_pose.orientation.y = 0.707;
    // target_pose.orientation.x = 0.0;

    //target_pose.position = init_pose.position;
    target_pose.position.x = test_pose[0];
    target_pose.position.y = test_pose[1];
    target_pose.position.z = test_pose[2];
    waypoints.push_back(init_pose);
    waypoints.push_back(target_pose);

    //planTrajInterpolate(waypoints, ur5);
    planTrajDirect(waypoints, ur5);

    /* 控制机械臂先回到初始化位置*/
    // ur5.setNamedTarget("up");
    // ur5.move();
    // sleep(1);

    ros::shutdown();
    return 0;
}