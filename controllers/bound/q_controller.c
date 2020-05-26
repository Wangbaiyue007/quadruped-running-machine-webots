/**********************************************************
**             Email:wangbyyy@qq.com
**---------------------------------------------------------
**  Description: 此文件为 单腿跳跃机器人 逻辑 文件
**  Version    : 
**  Notes      : 
**  Author     : 于宪元 && Baiyue's team
**********************************************************/
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <webots/keyboard.h>

#include "easyMat.h"
#include "q_controller.h"

robotTypeDef robot;
//----------------------------------------------------------declaration
void updateRobotStateMachine();
void forwardKinematics(matTypeDef *workPoint, jointSpaceTypeDef *jointPoint);
void update_xz_dot();
void update_last_Ts();
void update_xz_dot_desire();
void equalizeA();
void equalizeA_thrust();
void equalizeB();
void equalizeB_thrust();
void shortenA();
void shortenB();
void erectA();
void erectB();
void positionA();
void positionB();
void stopA();
void stopB();
void relaxA();
void relaxB();
/*
机器人参数初始化，在while（1）之前调用
*/
void robot_init()
{
  //-----------------------------------------------------------------------参数区
  robot.spring_normal_length = 0.7;                                            //弹簧原长
  robot.d_length = 0.45;                                                       //躯长
  robot.d_width = 0.2;                                                         //躯宽
  robot.v = 2;                                                                 //机器人水平运动速度
  robot.r_threshold = 0.97;                                                    //用于状态机切换的腿长阈值系数
  robot.k_spring = 5400;                                                       //弹簧刚度
  robot.F_thrust = 410.0;                                                      //THRUST推力，用于调节跳跃高度
  robot.k_leg_p = 8;                                                           //腿部控制时的kp
  robot.k_leg_v = 0.8;                                                         //腿部控制时的kv
  robot.k_xz_dot = 0.0;                                                        //净加速度系数
  robot.k_equalize = 45;                                                       //平衡腿长时的k
  //------------------------------------------------------------------------状态区
  //欧拉角，Roll,Pitch,Yaw
  robot.eulerAngle.roll = 0;
  robot.eulerAngle.pitch = 0;
  robot.eulerAngle.yaw = 0;
  //欧拉角的导数，Roll,Pitch,Yaw
  robot.eulerAngle_dot.roll = 0;
  robot.eulerAngle_dot.pitch = 0;
  robot.eulerAngle_dot.yaw = 0;
  //从{B}坐标系到{H}坐标系的转换
  easyMat_create(&robot.R_H_B, 3, 3);
  easyMat_eye(&robot.R_H_B);
  //从{H}坐标系到{B}坐标系的转换
  easyMat_create(&robot.R_B_H, 3, 3);
  easyMat_eye(&robot.R_B_H);
  //关节空间
  robot.jointPoint1.r = 0.5;
  robot.jointPoint1.X_motor_angle = 0;
  robot.jointPoint1.Z_motor_angle = 0;

  robot.jointPoint2.r = 0.5;
  robot.jointPoint2.X_motor_angle = 0;
  robot.jointPoint2.Z_motor_angle = 0;

  robot.jointPoint3.r = 0.5;
  robot.jointPoint3.X_motor_angle = 0;
  robot.jointPoint3.Z_motor_angle = 0;

  robot.jointPoint4.r = 0.5;
  robot.jointPoint4.X_motor_angle = 0;
  robot.jointPoint4.Z_motor_angle = 0;
  //关节空间的导数
  robot.jointPoint_dot1.r = 0;
  robot.jointPoint_dot1.X_motor_angle = 0;
  robot.jointPoint_dot1.Z_motor_angle = 0;

  robot.jointPoint_dot2.r = 0;
  robot.jointPoint_dot2.X_motor_angle = 0;
  robot.jointPoint_dot2.Z_motor_angle = 0;

  robot.jointPoint_dot3.r = 0;
  robot.jointPoint_dot3.X_motor_angle = 0;
  robot.jointPoint_dot3.Z_motor_angle = 0;

  robot.jointPoint_dot4.r = 0;
  robot.jointPoint_dot4.X_motor_angle = 0;
  robot.jointPoint_dot4.Z_motor_angle = 0;
  //{B}坐标系工作空间
  easyMat_create(&robot.workPoint_B, 3, 1);
  easyMat_clear(&robot.workPoint_B);

  easyMat_create(&robot.workPoint_B2, 3, 1);
  easyMat_clear(&robot.workPoint_B2);

  easyMat_create(&robot.workPoint_B3, 3, 1);
  easyMat_clear(&robot.workPoint_B3);

  easyMat_create(&robot.workPoint_B4, 3, 1);
  easyMat_clear(&robot.workPoint_B4);
  //{H}坐标系工作空间
  easyMat_create(&robot.workPoint_H, 3, 1);
  easyMat_clear(&robot.workPoint_H);

  easyMat_create(&robot.workPoint_H2, 3, 1);
  easyMat_clear(&robot.workPoint_H2);

  easyMat_create(&robot.workPoint_H3, 3, 1);
  easyMat_clear(&robot.workPoint_H3);

  easyMat_create(&robot.workPoint_H4, 3, 1);
  easyMat_clear(&robot.workPoint_H4);
  //{B}坐标系工作空间期望值
  easyMat_create(&robot.workPoint_B_desire, 3, 1);
  easyMat_clear(&robot.workPoint_B_desire);

  easyMat_create(&robot.workPoint_B_desire2, 3, 1);
  easyMat_clear(&robot.workPoint_B_desire2);

  easyMat_create(&robot.workPoint_B_desire3, 3, 1);
  easyMat_clear(&robot.workPoint_B_desire3);

  easyMat_create(&robot.workPoint_B_desire4, 3, 1);
  easyMat_clear(&robot.workPoint_B_desire4);
  //{H}坐标系工作空间期望值
  easyMat_create(&robot.workPoint_H_desire, 3, 1);
  easyMat_clear(&robot.workPoint_H_desire);

  easyMat_create(&robot.workPoint_H_desire2, 3, 1);
  easyMat_clear(&robot.workPoint_H_desire2);

  easyMat_create(&robot.workPoint_H_desire3, 3, 1);
  easyMat_clear(&robot.workPoint_H_desire3);

  easyMat_create(&robot.workPoint_H_desire4, 3, 1);
  easyMat_clear(&robot.workPoint_H_desire4);

  robot.is_foot_touching_a = false; //Virtual leg A touch justification
  robot.is_foot_touching_b = false; //Virtual leg B touch justification
  robot.Ts = 0.0;                   //上一个支撑相持续时间
  robot.x_dot = 3;                  //机身x方向水平速度
  robot.z_dot = 0;                  //机身z方向水平速度
  robot.x_dot_desire = 3;           //机身x方向期望水平速度
  robot.z_dot_desire = 0;           //机身z方向期望水平速度
  robot.system_ms = 0;              //从仿真启动开始的计时器
  robot.stateMachine = FLIGHT_A;    //状态机
}
/*
机器人内存空间释放
*/
void robot_free()
{
  easyMat_free(&robot.R_H_B);
  easyMat_free(&robot.R_B_H);
  easyMat_free(&robot.workPoint_B);
  easyMat_free(&robot.workPoint_H);
  easyMat_free(&robot.workPoint_B2);
  easyMat_free(&robot.workPoint_H2);
  easyMat_free(&robot.workPoint_B3);
  easyMat_free(&robot.workPoint_H3);
  easyMat_free(&robot.workPoint_B4);
  easyMat_free(&robot.workPoint_H4);
  easyMat_free(&robot.workPoint_B_desire);
  easyMat_free(&robot.workPoint_H_desire);
  easyMat_free(&robot.workPoint_B_desire2);
  easyMat_free(&robot.workPoint_H_desire2);
  easyMat_free(&robot.workPoint_B_desire3);
  easyMat_free(&robot.workPoint_H_desire3);
  easyMat_free(&robot.workPoint_B_desire4);
  easyMat_free(&robot.workPoint_H_desire4);
}
/*
机器人状态更新，包括读取传感器的数据以及一些状态估计
*/
void updateRobotState()
{
  /*clock*/
  robot.system_ms += TIME_STEP;
  /*update desired velocity*/
  update_xz_dot_desire();
  /*touch sensor on the feet*/
  robot.is_foot_touching_a = is_foot_touching1() && is_foot_touching3();
  robot.is_foot_touching_b = is_foot_touching2() && is_foot_touching4();

  /*body pose*/
  eulerAngleTypeDef now_IMU = get_IMU_Angle();
  eulerAngleTypeDef now_IMU_dot;

  now_IMU_dot.roll = (now_IMU.roll - robot.eulerAngle.roll) / (0.001 * TIME_STEP);
  now_IMU_dot.pitch = (now_IMU.pitch - robot.eulerAngle.pitch) / (0.001 * TIME_STEP);
  now_IMU_dot.yaw = (now_IMU.yaw - robot.eulerAngle.yaw) / (0.001 * TIME_STEP);
  robot.eulerAngle = now_IMU;

  robot.eulerAngle_dot.roll = robot.eulerAngle_dot.roll * 0.5 + now_IMU_dot.roll * 0.5;
  robot.eulerAngle_dot.pitch = robot.eulerAngle_dot.pitch * 0.5 + now_IMU_dot.pitch * 0.5;
  robot.eulerAngle_dot.yaw = robot.eulerAngle_dot.yaw * 0.5 + now_IMU_dot.yaw * 0.5;

  easyMat_RPY(&robot.R_H_B, robot.eulerAngle.roll, robot.eulerAngle.pitch, robot.eulerAngle.yaw);
  easyMat_trans(&robot.R_B_H, &robot.R_H_B);
  /*Spring length*/
  double now_r1 = get_spring_length1();
  double now_r2 = get_spring_length2();
  double now_r3 = get_spring_length3();
  double now_r4 = get_spring_length4();
  double now_r1_dot = (now_r1 - robot.jointPoint1.r) / (0.001 * TIME_STEP);
  double now_r2_dot = (now_r2 - robot.jointPoint2.r) / (0.001 * TIME_STEP);
  double now_r3_dot = (now_r3 - robot.jointPoint3.r) / (0.001 * TIME_STEP);
  double now_r4_dot = (now_r4 - robot.jointPoint4.r) / (0.001 * TIME_STEP);
  robot.jointPoint1.r = now_r1;
  robot.jointPoint2.r = now_r2;
  robot.jointPoint3.r = now_r3;
  robot.jointPoint4.r = now_r4;
  robot.jointPoint_dot1.r = robot.jointPoint_dot1.r * 0.5 + now_r1_dot * 0.5; //一阶低通滤波器
  robot.jointPoint_dot2.r = robot.jointPoint_dot2.r * 0.5 + now_r2_dot * 0.5;
  robot.jointPoint_dot3.r = robot.jointPoint_dot3.r * 0.5 + now_r3_dot * 0.5;
  robot.jointPoint_dot4.r = robot.jointPoint_dot4.r * 0.5 + now_r4_dot * 0.5;
  /*X, Z motor angle*/
  double now_X_motor_angle1 = get_X_motor_angle1();
  double now_X_motor_angle2 = get_X_motor_angle2();
  double now_X_motor_angle3 = get_X_motor_angle3();
  double now_X_motor_angle4 = get_X_motor_angle4();
  double now_X_motor_angle_dot1 = (now_X_motor_angle1 - robot.jointPoint1.X_motor_angle) / (0.001 * TIME_STEP);
  double now_X_motor_angle_dot2 = (now_X_motor_angle2 - robot.jointPoint2.X_motor_angle) / (0.001 * TIME_STEP);
  double now_X_motor_angle_dot3 = (now_X_motor_angle3 - robot.jointPoint3.X_motor_angle) / (0.001 * TIME_STEP);
  double now_X_motor_angle_dot4 = (now_X_motor_angle4 - robot.jointPoint4.X_motor_angle) / (0.001 * TIME_STEP);
  robot.jointPoint1.X_motor_angle = now_X_motor_angle1;
  robot.jointPoint2.X_motor_angle = now_X_motor_angle2;
  robot.jointPoint3.X_motor_angle = now_X_motor_angle3;
  robot.jointPoint4.X_motor_angle = now_X_motor_angle4;
  robot.jointPoint_dot1.X_motor_angle = robot.jointPoint_dot1.X_motor_angle * 0.5 + now_X_motor_angle_dot1 * 0.5; //一阶低通滤波器
  robot.jointPoint_dot2.X_motor_angle = robot.jointPoint_dot2.X_motor_angle * 0.5 + now_X_motor_angle_dot2 * 0.5;
  robot.jointPoint_dot3.X_motor_angle = robot.jointPoint_dot3.X_motor_angle * 0.5 + now_X_motor_angle_dot3 * 0.5;
  robot.jointPoint_dot4.X_motor_angle = robot.jointPoint_dot4.X_motor_angle * 0.5 + now_X_motor_angle_dot4 * 0.5;

  double now_Z_motor_angle1 = get_Z_motor_angle1();
  double now_Z_motor_angle2 = get_Z_motor_angle2();
  double now_Z_motor_angle3 = get_Z_motor_angle3();
  double now_Z_motor_angle4 = get_Z_motor_angle4();
  double now_Z_motor_angle_dot1 = (now_Z_motor_angle1 - robot.jointPoint1.Z_motor_angle) / (0.001 * TIME_STEP);
  double now_Z_motor_angle_dot2 = (now_Z_motor_angle2 - robot.jointPoint2.Z_motor_angle) / (0.001 * TIME_STEP);
  double now_Z_motor_angle_dot3 = (now_Z_motor_angle3 - robot.jointPoint3.Z_motor_angle) / (0.001 * TIME_STEP);
  double now_Z_motor_angle_dot4 = (now_Z_motor_angle4 - robot.jointPoint4.Z_motor_angle) / (0.001 * TIME_STEP);
  robot.jointPoint1.Z_motor_angle = now_Z_motor_angle1;
  robot.jointPoint2.Z_motor_angle = now_Z_motor_angle2;
  robot.jointPoint3.Z_motor_angle = now_Z_motor_angle3;
  robot.jointPoint4.Z_motor_angle = now_Z_motor_angle4;
  robot.jointPoint_dot1.Z_motor_angle = robot.jointPoint_dot1.Z_motor_angle * 0.5 + now_Z_motor_angle_dot1 * 0.5; //一阶低通滤波器
  robot.jointPoint_dot2.Z_motor_angle = robot.jointPoint_dot2.Z_motor_angle * 0.5 + now_Z_motor_angle_dot2 * 0.5;
  robot.jointPoint_dot3.Z_motor_angle = robot.jointPoint_dot3.Z_motor_angle * 0.5 + now_Z_motor_angle_dot3 * 0.5;
  robot.jointPoint_dot4.Z_motor_angle = robot.jointPoint_dot4.Z_motor_angle * 0.5 + now_Z_motor_angle_dot4 * 0.5;

  /*Estimate current velocity*/
  update_xz_dot();
  /*Update last supporting time*/
  update_last_Ts();
  /*Update state machine*/
  updateRobotStateMachine();
}
void robot_control()
{
  /* A touching ground*/
  if (robot.stateMachine == LOADING_A)
  {
    equalizeA();
    relaxA();
    shortenB();
    stopB();
  }
  if (robot.stateMachine == COMPRESSION_A)
  {
    equalizeA();
    relaxA();
    shortenB();
    positionB();
  }
  if (robot.stateMachine == THRUST_A)
  {
    equalizeA_thrust(robot.F_thrust);
    relaxA();
    shortenB();
    positionB();
  }
  if (robot.stateMachine == UNLOADING_A)
  {
    shortenA();
    relaxA();
    shortenB();
    positionB();
  }
  if (robot.stateMachine == FLIGHT_A)
  {
    shortenA();
    stopA();
    equalizeB();
    positionB();
  }

  /* B touching ground*/
  if (robot.stateMachine == LOADING_B)
  {
    equalizeB();
    relaxB();
    shortenA();
    stopA();
  }
  if (robot.stateMachine == COMPRESSION_B)
  {
    equalizeB();
    relaxB();
    shortenA();
    positionA();
  }
  if (robot.stateMachine == THRUST_B)
  {
    equalizeB_thrust(robot.F_thrust);
    relaxB();
    shortenA();
    positionA();
  }
  if (robot.stateMachine == UNLOADING_B)
  {
    shortenB();
    relaxB();
    ;
    shortenA();
    positionA();
  }
  if (robot.stateMachine == FLIGHT_B)
  {
    shortenB();
    stopB();
    equalizeA();
    positionA();
  }
}

/*
Equalize axial forces
*/
void equalizeA()
{
  double dx1 = robot.spring_normal_length - robot.jointPoint1.r; //求压缩量
  double dx3 = robot.spring_normal_length - robot.jointPoint3.r;
  double F_spring1 = dx1 * robot.k_spring - robot.k_equalize * (robot.jointPoint1.r - robot.jointPoint3.r) / 2;
  double F_spring3 = dx3 * robot.k_spring - robot.k_equalize * (robot.jointPoint3.r - robot.jointPoint1.r) / 2;
  set_spring_force1(F_spring1);
  set_spring_force3(F_spring3);
}
void equalizeA_thrust(double dF)
{
  double dx1 = robot.spring_normal_length - robot.jointPoint1.r; //求压缩量
  double dx2 = robot.spring_normal_length - robot.jointPoint3.r;
  double F_spring1 = dx1 * robot.k_spring - robot.k_equalize * (robot.jointPoint1.r - robot.jointPoint3.r) / 2;
  double F_spring2 = dx2 * robot.k_spring - robot.k_equalize * (robot.jointPoint3.r - robot.jointPoint1.r) / 2;
  set_spring_force1(F_spring1 + dF);
  set_spring_force3(F_spring2 + dF);
}
void equalizeB()
{
  double dx4 = robot.spring_normal_length - robot.jointPoint4.r; //求压缩量
  double dx2 = robot.spring_normal_length - robot.jointPoint2.r;
  double F_spring4 = dx4 * robot.k_spring - robot.k_equalize * (robot.jointPoint4.r - robot.jointPoint2.r) / 2;
  double F_spring2 = dx2 * robot.k_spring - robot.k_equalize * (robot.jointPoint2.r - robot.jointPoint4.r) / 2;
  set_spring_force4(F_spring4);
  set_spring_force2(F_spring2);
}
void equalizeB_thrust(double dF)
{
  double dx4 = robot.spring_normal_length - robot.jointPoint4.r; //求压缩量
  double dx2 = robot.spring_normal_length - robot.jointPoint2.r;
  double F_spring4 = dx4 * robot.k_spring - robot.k_equalize * (robot.jointPoint4.r - robot.jointPoint2.r) / 2;
  double F_spring2 = dx2 * robot.k_spring - robot.k_equalize * (robot.jointPoint2.r - robot.jointPoint4.r) / 2;
  set_spring_force4(F_spring4 + dF);
  set_spring_force2(F_spring2 + dF);
}

/*
Shorten legs
*/
void shortenA()
{
  set_spring_position1(0.0);
  set_spring_position3(0.0);
}
void shortenB()
{
  set_spring_position2(0.0);
  set_spring_position4(0.0);
}

void relaxA()
{
  set_Z_torque1(0.0);
  set_Z_torque3(0.0);
  set_X_torque1(0.0);
  set_X_torque3(0.0);
}
void stopA()
{
  set_Z_motor_angle1(0.0);
  set_Z_motor_angle3(0.0);
}
void relaxB()
{
  set_Z_torque2(0.0);
  set_Z_torque4(0.0);
  set_X_torque2(0.0);
  set_X_torque4(0.0);
}
void stopB()
{
  set_Z_motor_angle2(0.0);
  set_Z_motor_angle4(0.0);
}

/*
Position feet
*/
void positionA()
{

  double Tx1, Tz1, Tx3, Tz3;
  double r = robot.jointPoint1.r;
  double x_f1 = robot.x_dot * robot.Ts / 2.0 + robot.k_xz_dot * (robot.x_dot - robot.x_dot_desire);
  double z_f1 = robot.z_dot * robot.Ts / 2.0 + robot.k_xz_dot * (robot.z_dot - robot.z_dot_desire);
  double y_f1 = -sqrt(r * r - x_f1 * x_f1 - z_f1 * z_f1);
  double x_f3 = robot.x_dot * robot.Ts / 2.0 + robot.k_xz_dot * (robot.x_dot - robot.x_dot_desire);
  double z_f3 = robot.z_dot * robot.Ts / 2.0 + robot.k_xz_dot * (robot.z_dot - robot.z_dot_desire);
  double y_f3 = -sqrt(r * r - x_f3 * x_f3 - z_f3 * z_f3);
  robot.workPoint_H_desire.data[0][0] = x_f1;
  robot.workPoint_H_desire.data[1][0] = y_f1;
  robot.workPoint_H_desire.data[2][0] = z_f1;
  robot.workPoint_H_desire3.data[0][0] = x_f3;
  robot.workPoint_H_desire3.data[1][0] = y_f3;
  robot.workPoint_H_desire3.data[2][0] = z_f3;
  // Calculate desired angle for the two legs
  easyMat_mult(&robot.workPoint_B_desire, &robot.R_B_H, &robot.workPoint_H_desire);
  double x_f_B = robot.workPoint_H_desire.data[0][0];
  double y_f_B = robot.workPoint_H_desire.data[1][0];
  double z_f_B = robot.workPoint_H_desire.data[2][0];
  double x_angle_desire = atan(z_f_B / y_f_B) * 180.0 / PI;
  double z_angle_desire = asin(x_f_B / r) * 180.0 / PI;
  easyMat_mult(&robot.workPoint_B_desire3, &robot.R_B_H, &robot.workPoint_H_desire3);
  double x_f_B3 = robot.workPoint_H_desire3.data[0][0];
  double y_f_B3 = robot.workPoint_H_desire3.data[1][0];
  double z_f_B3 = robot.workPoint_H_desire3.data[2][0];
  double x_angle_desire3 = atan(z_f_B3 / y_f_B3) * 180.0 / PI;
  double z_angle_desire3 = asin(x_f_B3 / r) * 180.0 / PI;
  // Get the current angle and apply torque
  double x_angle1 = robot.jointPoint1.X_motor_angle;
  double z_angle1 = robot.jointPoint1.Z_motor_angle;
  double x_angle_dot1 = robot.jointPoint_dot1.X_motor_angle;
  double z_angle_dot1 = robot.jointPoint_dot1.Z_motor_angle;
  double x_angle3 = robot.jointPoint3.X_motor_angle;
  double z_angle3 = robot.jointPoint3.Z_motor_angle;
  double x_angle_dot3 = robot.jointPoint_dot3.X_motor_angle;
  double z_angle_dot3 = robot.jointPoint_dot3.Z_motor_angle;
  Tx1 = -robot.k_leg_p * (x_angle1 - x_angle_desire) - robot.k_leg_v * x_angle_dot1;
  Tz1 = -robot.k_leg_p * (z_angle1 - z_angle_desire) - robot.k_leg_v * z_angle_dot1;
  Tx3 = -robot.k_leg_p * (x_angle3 - x_angle_desire3) - robot.k_leg_v * x_angle_dot3;
  Tz3 = -robot.k_leg_p * (z_angle3 - z_angle_desire3) - robot.k_leg_v * z_angle_dot3;
  set_X_torque1(Tx1);
  set_Z_torque1(Tz1);
  set_X_torque3(Tx3);
  set_Z_torque3(Tz3);
}
void positionB()
{
  double Tx2, Tz2, Tx4, Tz4;
  double r = robot.jointPoint4.r;
  double x_f2 = robot.x_dot * robot.Ts / 2.0 + robot.k_xz_dot * (robot.x_dot - robot.x_dot_desire) + 0.0;
  double z_f2 = robot.z_dot * robot.Ts / 2.0 + robot.k_xz_dot * (robot.z_dot - robot.z_dot_desire) - 0.0; //bend inside towards COM
  double y_f2 = -sqrt(r * r - x_f2 * x_f2 - z_f2 * z_f2);
  double x_f4 = robot.x_dot * robot.Ts / 2.0 + robot.k_xz_dot * (robot.x_dot - robot.x_dot_desire) + 0.0;
  double z_f4 = robot.z_dot * robot.Ts / 2.0 + robot.k_xz_dot * (robot.z_dot - robot.z_dot_desire) + 0.0;
  double y_f4 = -sqrt(r * r - x_f4 * x_f4 - z_f4 * z_f4);
  robot.workPoint_H_desire2.data[0][0] = x_f2;
  robot.workPoint_H_desire2.data[1][0] = y_f2;
  robot.workPoint_H_desire2.data[2][0] = z_f2;
  robot.workPoint_H_desire4.data[0][0] = x_f4;
  robot.workPoint_H_desire4.data[1][0] = y_f4;
  robot.workPoint_H_desire4.data[2][0] = z_f4;
  // Calculate desired angle for the two legs
  easyMat_mult(&robot.workPoint_B_desire2, &robot.R_B_H, &robot.workPoint_H_desire2);
  double x_f_B2 = robot.workPoint_H_desire2.data[0][0];
  double y_f_B2 = robot.workPoint_H_desire2.data[1][0];
  double z_f_B2 = robot.workPoint_H_desire2.data[2][0];
  double x_angle_desire2 = atan(z_f_B2 / y_f_B2) * 180.0 / PI;
  double z_angle_desire2 = asin(x_f_B2 / r) * 180.0 / PI;
  easyMat_mult(&robot.workPoint_B_desire4, &robot.R_B_H, &robot.workPoint_H_desire4);
  double x_f_B4 = robot.workPoint_H_desire4.data[0][0];
  double y_f_B4 = robot.workPoint_H_desire4.data[1][0];
  double z_f_B4 = robot.workPoint_H_desire4.data[2][0];
  double x_angle_desire4 = atan(z_f_B4 / y_f_B4) * 180.0 / PI;
  double z_angle_desire4 = asin(x_f_B4 / r) * 180.0 / PI;
  // Get the current angle and apply torque
  double x_angle4 = robot.jointPoint4.X_motor_angle;
  double z_angle4 = robot.jointPoint4.Z_motor_angle;
  double x_angle_dot4 = robot.jointPoint_dot4.X_motor_angle;
  double z_angle_dot4 = robot.jointPoint_dot4.Z_motor_angle;
  double x_angle2 = robot.jointPoint2.X_motor_angle;
  double z_angle2 = robot.jointPoint2.Z_motor_angle;
  double x_angle_dot2 = robot.jointPoint_dot2.X_motor_angle;
  double z_angle_dot2 = robot.jointPoint_dot2.Z_motor_angle;
  Tx4 = -robot.k_leg_p * (x_angle4 - x_angle_desire4) - robot.k_leg_v * x_angle_dot4;
  Tz4 = -robot.k_leg_p * (z_angle4 - z_angle_desire4) - robot.k_leg_v * z_angle_dot4;
  Tx2 = -robot.k_leg_p * (x_angle2 - x_angle_desire2) - robot.k_leg_v * x_angle_dot2;
  Tz2 = -robot.k_leg_p * (z_angle2 - z_angle_desire2) - robot.k_leg_v * z_angle_dot2;
  set_X_torque4(Tx4);
  set_Z_torque4(Tz4);
  set_X_torque2(Tx2);
  set_Z_torque2(Tz2);
}

/*
Erect body through hip torque
*/
void erectA()
{
  double Tx, Tz;
  Tx = -(-robot.k_pose_p * robot.eulerAngle.roll - robot.k_pose_v * robot.eulerAngle_dot.roll);
  Tz = -(-robot.k_pose_p * robot.eulerAngle.pitch - robot.k_pose_v * robot.eulerAngle_dot.pitch);
  set_X_torque1(Tx);
  set_Z_torque1(Tz);
  set_X_torque3(Tx);
  set_Z_torque3(Tz);
}
void erectB()
{
  double Tx, Tz;
  Tx = -(-robot.k_pose_p * robot.eulerAngle.roll - robot.k_pose_v * robot.eulerAngle_dot.roll);
  Tz = -(-robot.k_pose_p * robot.eulerAngle.pitch - robot.k_pose_v * robot.eulerAngle_dot.pitch);
  set_X_torque4(Tx);
  set_Z_torque4(Tz);
  set_X_torque2(Tx);
  set_Z_torque2(Tz);
}
/*
Update desired velocity by reading the keyboard
*/
void update_xz_dot_desire()
{
    robot.z_dot_desire = 0;
    robot.x_dot_desire = 2.5;
}
/*
Update last contacting time
*/
void update_last_Ts()
{
  static bool pre_is_foot_touching = false;
  static int stance_start_ms = 0;
  if ((pre_is_foot_touching == false) && (robot.is_foot_touching_a == true || robot.is_foot_touching_b == true)) //此时进入支撑相
  {
    stance_start_ms = robot.system_ms;
  }
  if ((pre_is_foot_touching == true) && (robot.is_foot_touching_a == false && robot.is_foot_touching_b == false)) //此时进入摆动相
  {
    int stance_end_ms = robot.system_ms;
    robot.Ts = 0.001 * (double)(stance_end_ms - stance_start_ms);
  }
  if (robot.is_foot_touching_a || robot.is_foot_touching_b)
    pre_is_foot_touching = true;
  else
    pre_is_foot_touching = false;
}
/*
Estimate the current velocity through forward kinematics
*/
void update_xz_dot()
{
  double pre_x, pre_z, now_x, now_z, now_x_dot, now_z_dot;
  double pre_x2, pre_z2, now_x2, now_z2, now_x_dot2, now_z_dot2;
  static double pre_x_dot = 0;
  static double pre_z_dot = 0;
  static double pre_x_dot2 = 0;
  static double pre_z_dot2 = 0;
  if (robot.stateMachine == COMPRESSION_A || robot.stateMachine == THRUST_A || robot.stateMachine == UNLOADING_A || robot.stateMachine == LOADING_A)
  {
    //正运动学: leg1
    forwardKinematics(&robot.workPoint_B, &robot.jointPoint1);
    //转换到{H}坐标系下
    pre_x = robot.workPoint_H.data[0][0];
    pre_z = robot.workPoint_H.data[2][0];
    easyMat_mult(&robot.workPoint_H, &robot.R_H_B, &robot.workPoint_B);
    now_x = robot.workPoint_H.data[0][0];
    now_z = robot.workPoint_H.data[2][0];
    //求导
    now_x_dot = -(now_x - pre_x) / (0.001 * TIME_STEP);
    now_z_dot = -(now_z - pre_z) / (0.001 * TIME_STEP);
    //滤波
    now_x_dot = pre_x_dot * 0.5 + now_x_dot * 0.5;
    now_z_dot = pre_z_dot * 0.5 + now_z_dot * 0.5;
    pre_x_dot = now_x_dot;
    pre_z_dot = now_z_dot;

    //正运动学: leg3
    forwardKinematics(&robot.workPoint_B3, &robot.jointPoint3);
    //转换到{H}坐标系下
    pre_x2 = robot.workPoint_H3.data[0][0];
    pre_z2 = robot.workPoint_H3.data[2][0];
    easyMat_mult(&robot.workPoint_H3, &robot.R_H_B, &robot.workPoint_B3);
    now_x2 = robot.workPoint_H3.data[0][0];
    now_z2 = robot.workPoint_H3.data[2][0];
    //求导
    now_x_dot2 = -(now_x2 - pre_x2) / (0.001 * TIME_STEP);
    now_z_dot2 = -(now_z2 - pre_z2) / (0.001 * TIME_STEP);
    //滤波
    now_x_dot2 = pre_x_dot2 * 0.5 + now_x_dot2 * 0.5;
    now_z_dot2 = pre_z_dot2 * 0.5 + now_z_dot2 * 0.5;
    pre_x_dot2 = now_x_dot2;
    pre_z_dot2 = now_z_dot2;
  }
  else
  {
    //正运动学: leg4
    forwardKinematics(&robot.workPoint_B4, &robot.jointPoint4);
    //转换到{H}坐标系下
    pre_x = robot.workPoint_H4.data[0][0];
    pre_z = robot.workPoint_H4.data[2][0];
    easyMat_mult(&robot.workPoint_H4, &robot.R_H_B, &robot.workPoint_B4);
    now_x = robot.workPoint_H4.data[0][0];
    now_z = robot.workPoint_H4.data[2][0];
    //求导
    now_x_dot = -(now_x - pre_x) / (0.001 * TIME_STEP);
    now_z_dot = -(now_z - pre_z) / (0.001 * TIME_STEP);
    //滤波
    now_x_dot = pre_x_dot * 0.5 + now_x_dot * 0.5;
    now_z_dot = pre_z_dot * 0.5 + now_z_dot * 0.5;
    pre_x_dot = now_x_dot;
    pre_z_dot = now_z_dot;

    //正运动学: leg2
    forwardKinematics(&robot.workPoint_B2, &robot.jointPoint2);
    //转换到{H}坐标系下
    pre_x2 = robot.workPoint_H2.data[0][0];
    pre_z2 = robot.workPoint_H2.data[2][0];
    easyMat_mult(&robot.workPoint_H2, &robot.R_H_B, &robot.workPoint_B2);
    now_x2 = robot.workPoint_H2.data[0][0];
    now_z2 = robot.workPoint_H2.data[2][0];
    //求导
    now_x_dot2 = -(now_x2 - pre_x2) / (0.001 * TIME_STEP);
    now_z_dot2 = -(now_z2 - pre_z2) / (0.001 * TIME_STEP);
    //滤波
    now_x_dot2 = pre_x_dot2 * 0.5 + now_x_dot2 * 0.5;
    now_z_dot2 = pre_z_dot2 * 0.5 + now_z_dot2 * 0.5;
    pre_x_dot2 = now_x_dot2;
    pre_z_dot2 = now_z_dot2;
  }
  if (robot.stateMachine == COMPRESSION_A || robot.stateMachine == COMPRESSION_B || robot.stateMachine == THRUST_A || robot.stateMachine == THRUST_B)
  {
    robot.x_dot = (now_x_dot + now_x_dot2) / 2;
    robot.z_dot = (now_z_dot + now_z_dot2) / 2;
  }
}

/*
机器人状态机切换
---------------------------------------------------
|  名称     |     代码       |       触发条件      |
---------------------------------------------------
|  落地     |   LOADING      |   足底传感器触地    |
|  压缩腿   |   COMPRESSION  |   腿长小于阈值      |
|  伸长腿   |   THRUST       |   腿长导数为正      |
|  离地     |   UNLOADING    |   腿长大于阈值      |
| 飞行      |   FLIGHT       |   足底传感器离地    |
---------------------------------------------------
*/
void updateRobotStateMachine()
{
  switch (robot.stateMachine)
  {
  case LOADING_A:
  {
    if (robot.jointPoint_dot1.r < 0 && robot.jointPoint_dot3.r < 0 && robot.is_foot_touching_a)
      robot.stateMachine = COMPRESSION_A;
    break;
  }
  case COMPRESSION_A:
  {
    if (robot.jointPoint_dot1.r > 0 && robot.jointPoint_dot3.r > 0 && robot.is_foot_touching_a)
      robot.stateMachine = THRUST_A;
    break;
  }
  case THRUST_A:
  {
    if (robot.jointPoint1.r > robot.spring_normal_length * robot.r_threshold && robot.jointPoint3.r > robot.spring_normal_length * robot.r_threshold && robot.is_foot_touching_a)
      robot.stateMachine = UNLOADING_A;
    break;
  }
  case UNLOADING_A:
  {
    if (is_foot_touching1() == false && is_foot_touching3() == false)
      robot.stateMachine = FLIGHT_A;
    break;
  }
  case FLIGHT_A:
  {
    if (robot.is_foot_touching_b)
      robot.stateMachine = LOADING_B;
    break;
  }
  case LOADING_B:
  {
    if (robot.jointPoint_dot4.r < 0 && robot.jointPoint_dot2.r < 0 && robot.is_foot_touching_b)
      robot.stateMachine = COMPRESSION_B;
    break;
  }
  case COMPRESSION_B:
  {
    if (robot.jointPoint_dot4.r > 0 && robot.jointPoint_dot2.r > 0 && robot.is_foot_touching_b)
      robot.stateMachine = THRUST_B;
    break;
  }
  case THRUST_B:
  {
    if (robot.jointPoint4.r > robot.spring_normal_length * robot.r_threshold && robot.jointPoint2.r > robot.spring_normal_length * robot.r_threshold && robot.is_foot_touching_b)
      robot.stateMachine = UNLOADING_B;
    break;
  }
  case UNLOADING_B:
  {
    if (is_foot_touching4() == false && is_foot_touching2() == false)
      robot.stateMachine = FLIGHT_B;
    break;
  }
  case FLIGHT_B:
  {
    if (robot.is_foot_touching_a)
      robot.stateMachine = LOADING_A;
    break;
  }
  default:
    break;
  }
}
/*
Forward Kinematics, from the hip motor angles to the coordinate of the end point (foot).
{B}坐标系下足底坐标 = forwardKinematics(关节角度)
*/
void forwardKinematics(matTypeDef *workPoint, jointSpaceTypeDef *jointPoint)
{
  //转换成弧度制
  double Tx = jointPoint->X_motor_angle * PI / 180.0;
  double Tz = jointPoint->Z_motor_angle * PI / 180.0;
  double r = jointPoint->r;

  workPoint->data[0][0] = r * sin(Tz);
  workPoint->data[1][0] = -r * cos(Tz) * cos(Tx);
  workPoint->data[2][0] = -r * cos(Tz) * sin(Tx);
}
