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

    // 控制机械臂先回到初始化位置
    //ur5.setNamedTarget("up");
    //ur5.move();
    //sleep(1);
    double c=0.85;//x y z center of ball corresponding to c a b
    double a=0;
    double b=-0.07;
    double d=0.3;//运行圆弧面到理论球心距离
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = 1;
    target_pose.orientation.y = 0;
    target_pose.orientation.z = 0;
    target_pose.orientation.w = -0.49;

    target_pose.position.x = c-d;
    target_pose.position.y = a;
    target_pose.position.z = b;
    

    ur5.setPoseTarget(target_pose);
    ur5.move();

    vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);

    //在xy平面内生成一个圆周
    double centerA = target_pose.position.y;
    double centerB = target_pose.position.z;
    double radius = 0.1;
        
///*************************************************************
    //for(double theta = 0.0; theta < M_PI*2; theta += 0.01)
    for(double theta = 0.0; theta < 3.1415926*2; theta += 0.01)

    {
	target_pose.position.y = centerA + radius * cos(theta);
        target_pose.position.z = centerB + radius * sin(theta);
        double m11,m12,m13;
        double m21,m22,m23;
        double m31,m32,m33;

        double w,x,y,z;

        double y_relative;
        double z_relative;

        y_relative= a- target_pose.position.y;
        z_relative= b- target_pose.position.z;    
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
        waypoints.push_back(target_pose);
    }
/////****************************************************
    // 笛卡尔空间下的路径规划
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = ur5.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;

        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
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

    /* 控制机械臂先回到初始化位置*/
   // ur5.setNamedTarget("up");
   // ur5.move();
   // sleep(1);

    ros::shutdown();
    return 0;
}
