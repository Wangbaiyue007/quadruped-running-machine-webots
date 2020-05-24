/**********************************************************
**             Email:@qq.com   QQ:1069841355
**---------------------------------------------------------
**  Description: 此文件为 单腿跳跃机器人 控制器头 文件
**  Version    : 
**  Notes      : 
**  Author     : 于宪元
**********************************************************/
#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <stdbool.h>

#include "easyMat.h"
#include "q_webotsInterface.h"

//-----------------------------------------------------------macro

//-----------------------------------------------------------typedef
/*
机器人 状态机 枚举类型定义
*/
typedef enum
{
  LOADING_A = 0x00,     //落地
  COMPRESSION_A = 0x01, //压缩腿
  THRUST_A = 0x02,      //伸长腿
  UNLOADING_A = 0x03,   //离地
  FLIGHT_A = 0x04,      //飞行
  LOADING_B = 0x05,     //落地
  COMPRESSION_B = 0x06, //压缩腿
  THRUST_B = 0x07,      //伸长腿
  UNLOADING_B = 0x08,   //离地
  FLIGHT_B = 0x09       //飞行
} stateMachineTypeDef;
/*
机器人 关节空间 类型定义
*/
typedef struct
{
  double r;             //杆长
  double X_motor_angle; //x轴扭转角，角度制
  double Z_motor_angle; //z轴扭转角，角度制
} jointSpaceTypeDef;
/*
机器人定义,包括机器人状态和机器人参数
注 意：备注“ // ”中加“ * ”符号的，需要同时修改场景树中相关内容才能生效
*/
typedef struct
{
  //------------------------------------------------------------------------参数区
  double spring_normal_length; //*弹簧原长
  double d_width;              //躯宽
  double d_length;             //躯长
  double d_center;             //腿和重心距离
  double k_spring;             //弹簧刚度
  double F_thrust;             //THRUST推力
  double F_shorten;            //收缩时的力
  double k_xz_dot;             //净加速度系数
  double r_threshold;          //状态机在脱离LOADING和进入UNLOADING状态时，腿长阈值判断
  double v;                    //机器人水平运动速度
  double k_pose_p;             //姿态控制时的kp
  double k_pose_v;             //姿态控制时的kv
  double k_leg_p;              //腿部控制时的kp
  double k_leg_v;              //腿部控制时的kv
  double k_equalize;           //平衡腿长时的k
  //------------------------------------------------------------------------状态区
  eulerAngleTypeDef eulerAngle;     //欧拉角，Roll,Pitch,Yaw
  eulerAngleTypeDef eulerAngle_dot; //欧拉角的导数，Roll,Pitch,Yaw
  matTypeDef R_H_B;                 //从{B}坐标系到{H}坐标系的转换
  matTypeDef R_B_H;                 //从{H}坐标系到{B}坐标系的转换
  matTypeDef legTransl;             //四足前右脚离中心的位置

  jointSpaceTypeDef jointPoint1; //关节空间
  jointSpaceTypeDef jointPoint2;
  jointSpaceTypeDef jointPoint3;
  jointSpaceTypeDef jointPoint4;
  jointSpaceTypeDef jointPoint_dot1; //关节空间的导数
  jointSpaceTypeDef jointPoint_dot2;
  jointSpaceTypeDef jointPoint_dot3;
  jointSpaceTypeDef jointPoint_dot4;
  matTypeDef workPoint_B;         //{B}坐标系工作空间
  matTypeDef workPoint_H;         //{H}坐标系工作空间
  matTypeDef workPoint_B2;        //{B}坐标系工作空间
  matTypeDef workPoint_H2;        //{H}坐标系工作空间
  matTypeDef workPoint_B3;        //{B}坐标系工作空间
  matTypeDef workPoint_H3;        //{H}坐标系工作空间
  matTypeDef workPoint_B4;        //{B}坐标系工作空间
  matTypeDef workPoint_H4;        //{H}坐标系工作空间
  matTypeDef workPoint_B_desire;  //{B}坐标系工作空间期望值
  matTypeDef workPoint_H_desire;  //{H}坐标系工作空间期望值
  matTypeDef workPoint_B_desire2; //{B}坐标系工作空间期望值
  matTypeDef workPoint_H_desire2; //{H}坐标系工作空间期望值
  matTypeDef workPoint_B_desire3; //{B}坐标系工作空间期望值
  matTypeDef workPoint_H_desire3; //{H}坐标系工作空间期望值
  matTypeDef workPoint_B_desire4; //{B}坐标系工作空间期望值
  matTypeDef workPoint_H_desire4; //{H}坐标系工作空间期望值

  bool is_foot_touching_a; //足底是否触地
  bool is_foot_touching_b;
  double Ts;                        //上一个支撑相持续时间
  double x_dot;                     //机身x方向水平速度
  double z_dot;                     //机身z方向水平速度
  double x_dot_desire;              //机身x方向期望水平速度
  double z_dot_desire;              //机身z方向期望水平速度
  double eulerAngle_dot_desire;     //期望角速度
  int system_ms;                    //从仿真启动开始的计时器
  stateMachineTypeDef stateMachine; //状态机
} robotTypeDef;
//-----------------------------------------------------------extern
extern robotTypeDef robot;

extern void robot_init();
extern void updateRobotState();
extern void robot_control();
extern void robot_free();

#endif
