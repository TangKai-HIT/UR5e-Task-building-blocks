#include <math.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include<iostream>
#include<vector>
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_with_circle");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface ur5("manipulator");

    string eef_link = ur5.getEndEffector();
    std::string reference_frame = "base_link";
    ur5.setPoseReferenceFrame(reference_frame);

    ur5.allowReplanning(true);

    ur5.setGoalPositionTolerance(0.01);
    ur5.setGoalOrientationTolerance(0.01);
    ur5.setMaxAccelerationScalingFactor(0.8);
    ur5.setMaxVelocityScalingFactor(0.8);
    double c=0.56;//
    
   
//方向向量dab
    double d=1;//
    double a=1;
    double b=1;
    geometry_msgs::Pose target_pose;

   
   // ur5.setPoseTarget(target_pose);
  //  ur5.move();

    vector<geometry_msgs::Pose> waypoints;
   // waypoints.push_back(target_pose);
//位置设定，方向转换为四元数

        double m11,m12,m13;
        double m21,m22,m23;
        double m31,m32,m33;

    double w,x,y,z;//方向向量（d，a，b）
    double y_relative;
    double z_relative;

    y_relative= a;
    z_relative= b;
    double Norm1;
    m11= d;
    m21= y_relative;
    m31= z_relative;
    Norm1=pow(pow(m11,2)+pow(m21,2)+pow(m31,2),0.5);
//*******set matrix
    m11= d/Norm1;
    m21= y_relative/Norm1;
    m31= z_relative/Norm1;
//***
    double Norm2;
    Norm2=pow((pow(y_relative,2)+pow(d,2)),0.5);
    m12= y_relative/Norm2;
    m22= -d/Norm2;
    m32= 0;

    m13 = m21 * m32 - m31 * m22;  //计算三阶行列式
    m23 = m31 * m12 - m11 * m32;
    m33 = m11 * m22 - m21 * m12;

//探测四元数中最大的项
    double fourWSquaredMinusl = m11+m22+m33;
double fourXSquaredMinusl = m11-m22-m33;
double fourYSquaredMinusl = m22-m11-m33;
double fourZSquaredMinusl = m33-m11-m22;

int biggestIndex = 0;
double fourBiggestSqureMinus1 = fourWSquaredMinusl;
if(fourXSquaredMinusl>fourBiggestSqureMinus1){
	fourBiggestSqureMinus1 = fourXSquaredMinusl;
	biggestIndex =1;
} 
if(fourYSquaredMinusl>fourBiggestSqureMinus1){
	fourBiggestSqureMinus1 = fourYSquaredMinusl;
	biggestIndex =2;
} 
if(fourZSquaredMinusl>fourBiggestSqureMinus1){
	fourBiggestSqureMinus1 = fourZSquaredMinusl;
	biggestIndex =3;
} 

//计算平方根和除法 
double biggestVal = sqrt(fourBiggestSqureMinus1+1.0f)*0.5f;
double mult = 0.25f/biggestVal;

//计算四元数的值
switch(biggestIndex){
	case 0:
		w=biggestVal;
		x=(m23-m32)*mult;
		y=(m31-m13)*mult;
		z=(m12-m21)*mult;
		break;
	case 1:
		x = biggestVal;
		w =(m23-m32)*mult;
		y =(m12+m21)*mult;
		z =(m31+m13)*mult;
		break;
	case 2:
		y =biggestVal;
		w =(m31-m13)*mult;
		x =(m12+m21)*mult;
		z =(m23+m32)*mult;
		break;
	case 3:
		z =biggestVal;
		w =(m12-m21)*mult;
		x =(m31+m13)*mult;
		y =(m23+m32)*mult;
		break;
} 
        
        
        
//if((target_pose.position.y-a)>0)
            
       target_pose.orientation.x = x;
       target_pose.orientation.y = y;
       target_pose.orientation.z = z;
       target_pose.orientation.w = w;
    
    target_pose.position.x = 0.3;
    target_pose.position.y = 0.3;
    target_pose.position.z = 0.3;
        waypoints.push_back(target_pose);
        ur5.setPoseTarget(target_pose);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = 0.0;
    int maxtries = 10000;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = (ur5.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        attempts++;

        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    if(fraction == 1)
    {
        ROS_INFO("Path computed successfully. Moving the arm.");

        // 执行运动
        ur5.move();
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    /* 控制机械臂先回到初始化位置*/
    // ur5.setNamedTarget("up");
    // ur5.move();
    // sleep(1);

    ros::shutdown();
    return 0;
}




