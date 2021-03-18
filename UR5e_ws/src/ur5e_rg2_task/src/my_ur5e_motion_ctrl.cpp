// ROS
#include <ros/ros.h>
#include <math.h>
// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//rg2_service
#include "ur5e_rg2_task/rg2_control.h"
//dashBoared play service
#include "std_srvs/Trigger.h"
//pick and place
#include "pick_place.hpp"
//trajectory planning
#include "trajectory_plan.hpp"

#define GRIP_OPEN 80
#define GRIP_CLOSE 32
#define GRIP_FORCE 30

using std::vector; 

// void allocateTCPPose(double *P, geometry_msgs::Pose &pose); //allocate recorded TCP poses to waypoints
// void TCP_To_eelink(vector<geometry_msgs::Pose> &waypoints); //convert TCP(tool0) pose to ee_link pose
// void planTraj(vector<geometry_msgs::Pose> &waypoints, 
//               moveit::planning_interface::MoveGroupInterface &ur5); //plan trajetory segment

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_ur5e_motion_ctrl");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(1.0).sleep();

  /*---------------move group API init---------------*/
  //init move group and planning_scene_interface
  static const std::string PLANNING_GROUP="manipulator";
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  //setup motion group
  move_group.setPlanningTime(200.0);
  std::string eef_link = move_group.getEndEffector();
  std::string reference_frame = "base_link";
  move_group.setPoseReferenceFrame(reference_frame);
  move_group.allowReplanning(true);
  move_group.setGoalPositionTolerance(0.01);
  move_group.setGoalOrientationTolerance(0.01);
  move_group.setMaxAccelerationScalingFactor(0.5);
  move_group.setMaxVelocityScalingFactor(0.3);

  //set motion planning constraint
  // moveit_msgs::OrientationConstraint ocm;
  // ocm.link_name = "ee_link";
  // ocm.header.frame_id = "base_link";
  // ocm.orientation.w = 0.0; //R=180, p=0
  // ocm.orientation.z = 0.0;
  // ocm.absolute_x_axis_tolerance = 0.1;
  // ocm.absolute_y_axis_tolerance = 0.1;
  // ocm.absolute_z_axis_tolerance = 0.1;
  // ocm.weight = 1.0;

  // moveit_msgs::Constraints constraints;
  // constraints.orientation_constraints.push_back(ocm);
  // move_group.setPathConstraints(constraints);

  /*-----------------init rg2 and dashboard-----------------*/ 
  //wait for service to be ready
  ros::service::waitForService("rg2_control");
  ros::service::waitForService("/ur_hardware_interface/dashboard/play");
  ros::WallDuration(2.0).sleep();

  //init rg2 gripper client
  ros::ServiceClient rg2_client = nh.serviceClient<ur5e_rg2_task::rg2_control>("rg2_control");
  ur5e_rg2_task::rg2_control rg2_srv;

  //init dashboard play client
  ros::ServiceClient dashBoardCtrl_client = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/play");
  std_srvs::Trigger dashBoardPlay_srv;

  //init dashboard external control
  dashBoardCtrl_client.call(dashBoardPlay_srv);
  if(dashBoardPlay_srv.response.success) 
  {
    ROS_INFO("enable external control success!");
  }
  ros::WallDuration(1.0).sleep();

  //init rg2 gripper status
  rg2_srv.request.width = GRIP_OPEN;
  rg2_srv.request.force = GRIP_FORCE;
  rg2_client.call(rg2_srv);
  ros::WallDuration(1.0).sleep();
  //reconnect robot(restart external control)
  dashBoardCtrl_client.call(dashBoardPlay_srv);
  ros::WallDuration(1.0).sleep();

  /*---------------set waypoints and identify pick and place position------------------*/
  //way points definition and record index to perform pick and place actions
  double pickPoints[][6] = {{0.088, 0.422, 0.216, 2.4314, 1.5637, 2.512},
                           {-0.0802, 0.425, 0.216, 2.464, 1.564, 2.537},
                           {-0.038, 0.291, 0.216, 2.1485, 1.563, 2.5187}
                           };
  double placePoints[][6] = {{0.0834, 0.3091, 0.216+0.001, -2.735, 1.561, -2.744},
                             {0.01809, 0.37664, 0.216+0.019, -0.3106, 1.537, -1.856},
                             {0.0834, 0.3091, 0.216+0.033, -2.735, 1.561, -2.744}};

  //init pose vector sequence
  std::vector<geometry_msgs::Pose> pose_sequence;
  geometry_msgs::Pose init_pose = move_group.getCurrentPose().pose;
  pose_sequence.push_back(init_pose); //start point
  vector<vector<geometry_msgs::Pose>::size_type> pickIndex, placeIndex; //define index to perform pick and place actions

  for(size_t i=0;i<3;++i)
  {
    geometry_msgs::Pose safe_Pick, pick_pose, safe_Place, place_pose;
    allocateTCPPose(pickPoints[i], pick_pose);
    allocateTCPPose(placePoints[i], place_pose);
    safe_Pick = pick_pose; 
    safe_Pick.position.z = safe_Pick.position.z + 0.15;
    safe_Place = place_pose;
    safe_Place.position.z = safe_Place.position.z + 0.15;

    pose_sequence.push_back(safe_Pick); 
    pose_sequence.push_back(pick_pose); 
    pickIndex.push_back(pose_sequence.size()-1); //record pose index to perform pick action
    pose_sequence.push_back(safe_Pick);

    pose_sequence.push_back(safe_Place); 
    pose_sequence.push_back(place_pose);
    placeIndex.push_back(pose_sequence.size()-1); //record pose index to perform place action
    pose_sequence.push_back(safe_Place);
  }

  /*------------------motion planning and execution-------------------*/
  vector<geometry_msgs::Pose>::iterator trajSegBegin = pose_sequence.begin();
  vector<vector<geometry_msgs::Pose>::size_type>::iterator pickIndexIter = pickIndex.begin();
  vector<vector<geometry_msgs::Pose>::size_type>::iterator placeIndexIter = placeIndex.begin();

  for(vector<geometry_msgs::Pose>::size_type i=0; i<pose_sequence.size(); ++i)
  {
    if(pickIndexIter!=pickIndex.end() && i==(*pickIndexIter))
    {
      vector<geometry_msgs::Pose>::iterator trajSegEnd = pose_sequence.begin()+i;
      vector<geometry_msgs::Pose> trajSeg(trajSegBegin, trajSegEnd+1); //create trajectory segment for picking a block
      trajSegBegin = trajSegEnd;
      ROS_INFO("pick action! start planning!");
      planTrajDirect(trajSeg, move_group);
      ros::WallDuration(1.0).sleep();

      /*pick the block*/
      rg2_srv.request.width = GRIP_CLOSE;
      rg2_srv.request.force = GRIP_FORCE;
      rg2_client.call(rg2_srv);
      ros::WallDuration(1.0).sleep();
      //reconnect robot(restart external control)
      dashBoardCtrl_client.call(dashBoardPlay_srv);
      ros::WallDuration(1.0).sleep();

      ++pickIndexIter;
    }
    
    if(placeIndexIter!=placeIndex.end() && i==(*placeIndexIter))
    {
      vector<geometry_msgs::Pose>::iterator trajSegEnd = pose_sequence.begin()+i;
      vector<geometry_msgs::Pose> trajSeg(trajSegBegin, trajSegEnd+1); //create trajectory segment for picking a block
      trajSegBegin = trajSegEnd;
      ROS_INFO("place action! start planning!");
      planTrajDirect(trajSeg, move_group);
      ros::WallDuration(1.0).sleep();

      /*place the block*/
      rg2_srv.request.width = GRIP_OPEN;
      rg2_srv.request.force = GRIP_FORCE;
      rg2_client.call(rg2_srv);
      ros::WallDuration(1.0).sleep();
      //reconnect robot(restart external control)
      dashBoardCtrl_client.call(dashBoardPlay_srv);
      ros::WallDuration(1.0).sleep();

      ++placeIndexIter;
    }
  }

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  ros::waitForShutdown();
  return 0;
}

