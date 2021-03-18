# ur5e_rg2_task包的使用说明  
`Author:TK(汤凯HITSZ) Email：1292737216@qq.com`
## 1.测试文件（仅用于测试功能）
### 1.1 `rg2_ctrl_client_demo.py`：  
订阅`/rg2_control`服务的客户端demo 
 ### 1.2 `motion_test_demo.cpp`：
 测试Moveit API的运动控制demo
 ### 1.3 `transformation_demo.py`：
 测试pyhton的tf的四元数转换函数
 ## 2.实际需要使用的文件
 ### 2.1 `rg2_ctrl_server.py`与`rg2_gripper.py` :  
用于创建rg2手爪服务，服务的定义文件为`rg2_control.srv`  
```
float32 width
float32 force
---
bool success
```
### 2.2 `my_ur5e_motion_ctrl.cpp`：
实际使用的文件，内部包含的功能有：  
（1）远程控制模式下请求`"/ur_hardware_interface/dashboard/play"`服务开启`external control`。（需要提前开启dashboard client节点）  
```bash
$ rosrun ur_robot_driver dashboard_client
```
（2）远程控制模式下请求`"/rg2_control"`服务来使用rg2手爪，但每次使用socket向手爪发送命令并执行后，机器人IP会掉线，`external control`不能使用，因此在程序中每次调用完手爪服务后需要再次请求`"/ur_hardware_interface/dashboard/play"`，开启`external control`，不然机械臂动不了。  
（3）主功能：经过各个已知的位姿点夹取积木、放置积木，交叉堆叠起来。
## 3.`launch`文件使用说明
### 3.1 `startSimulation.launch`：  
纯ur5e仿真环境开启，默认限定关节范围，开启gazebo、rviz、moveit。使用纯仿真环境不连接实际机器人，需要先把`ur5_e_moveit_config`包中的`controllers.yaml`文件中`controller`的`name`设为`arm_controller`。
### 3.2 `start_ur5e_operation.launch`：  
ur5e仿真+实际操控环境开启，默认限定关节范围，开启rviz、moveit、不开起gazebo，需要设置ur机器人为远程控制模式，本文件自动开启`dashboard_client`节点，通过请求其服务来远程控制面板。`ur5_e_moveit_config`包中的`controllers.yaml`文件中`controller`的`name`设为`/scaled_pos_joint_traj_controller`。
### 3.3 `motion_test.launch`
开启手爪服务节点`rg2_ctrl_server.py`并开启主要任务节点`my_ur5e_motion_ctrl.cpp`，启动搭积木任务。
### 3.4 总体实机演示时launch文件调用方法  
关闭电源后，执行：
```bash
$ roslaunch ur5e_rg2_task start_ur5e_operation.launch
```
开启电源，启动机器人后执行：
```bash
$ roslaunch ur5e_rg2_task motion_test.launch
```