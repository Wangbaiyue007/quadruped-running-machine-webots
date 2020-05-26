/**********************************************************
**             Email:wangbyyy@qq.com
**---------------------------------------------------------
**  Description: 此文件为 单腿跳跃机器人 仿真环境或硬件接口 头文件
**  Version    : 
**  Notes      : 
**  Author     : 于宪元 && Baiyue's team
**********************************************************/
#ifndef _WEBOTSINTERFACE_H_
#define _WEBOTSINTERFACE_H_

//-----------------------------------------------------------macro
#define PI (3.141892654)
#define TIME_STEP (2)
//-----------------------------------------------------------typedef
/*
1，陀螺仪数据定义,为了方便调试采用角度制，注意，采用角度制
2，webots IMU模块采用RPY角度制，定系旋转，矩阵左乘，即：
       Rot=RotY(yaw)*RotZ(pitch)*RotX(roll);
*/
typedef struct
{
  double roll;  //横滚，x轴
  double pitch; //俯仰，z轴
  double yaw;   //偏航，y轴
} eulerAngleTypeDef;

//-----------------------------------------------------------extern
extern void webots_device_init();
extern void set_spring_force1(double force);
extern void set_spring_force2(double force);
extern void set_spring_force3(double force);
extern void set_spring_force4(double force);
extern void set_spring_position1(double v);
extern void set_spring_position2(double v);
extern void set_spring_position3(double v);
extern void set_spring_position4(double v);
extern void set_X_torque1(double torque);
extern void set_X_torque2(double torque);
extern void set_X_torque3(double torque);
extern void set_X_torque4(double torque);
extern void set_Z_torque1(double torque);
extern void set_Z_torque2(double torque);
extern void set_Z_torque3(double torque);
extern void set_Z_torque4(double torque);
extern void set_Z_motor_angle1(double r);
extern void set_Z_motor_angle2(double r);
extern void set_Z_motor_angle3(double r);
extern void set_Z_motor_angle4(double r);
extern double get_spring_length1();
extern double get_spring_length2();
extern double get_spring_length3();
extern double get_spring_length4();
extern double get_X_motor_angle1();
extern double get_X_motor_angle2();
extern double get_X_motor_angle3();
extern double get_X_motor_angle4();
extern double get_Z_motor_angle1();
extern double get_Z_motor_angle2();
extern double get_Z_motor_angle3();
extern double get_Z_motor_angle4();
extern bool is_foot_touching1();
extern bool is_foot_touching2();
extern bool is_foot_touching3();
extern bool is_foot_touching4();
extern int get_keyboard();
extern eulerAngleTypeDef get_IMU_Angle();
#endif
