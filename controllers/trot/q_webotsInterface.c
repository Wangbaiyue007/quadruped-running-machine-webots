/**********************************************************
**             Email:@qq.com   QQ:1069841355
**---------------------------------------------------------
**  Description: 此文件为 单腿跳跃机器人 仿真环境 接口文件
**  Version    : 
**  Notes      : 
**  Author     : 于宪元
**********************************************************/
#include <stdio.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/touch_sensor.h>
#include <webots/keyboard.h>
#include <webots/inertial_unit.h>

#include "q_webotsInterface.h"

//-----------------------------------------------------------device
WbDeviceTag spring_motor1;
WbDeviceTag spring_motor2;
WbDeviceTag spring_motor3;
WbDeviceTag spring_motor4;
WbDeviceTag spring_pos_sensor1;
WbDeviceTag spring_pos_sensor2;
WbDeviceTag spring_pos_sensor3;
WbDeviceTag spring_pos_sensor4;
WbDeviceTag touch_sensor1;
WbDeviceTag touch_sensor2;
WbDeviceTag touch_sensor3;
WbDeviceTag touch_sensor4;
WbDeviceTag IMU;
WbDeviceTag X_motor1;
WbDeviceTag X_motor2;
WbDeviceTag X_motor3;
WbDeviceTag X_motor4;
WbDeviceTag Z_motor1;
WbDeviceTag Z_motor2;
WbDeviceTag Z_motor3;
WbDeviceTag Z_motor4;
WbDeviceTag X_motor_pos_sensor1;
WbDeviceTag X_motor_pos_sensor2;
WbDeviceTag X_motor_pos_sensor3;
WbDeviceTag X_motor_pos_sensor4;
WbDeviceTag Z_motor_pos_sensor1;
WbDeviceTag Z_motor_pos_sensor2;
WbDeviceTag Z_motor_pos_sensor3;
WbDeviceTag Z_motor_pos_sensor4;
/*
函数功能：初始化devices
*/
void webots_device_init()
{
  //get device
  spring_motor1       = wb_robot_get_device("leg_1_linear_motor");
  spring_motor2       = wb_robot_get_device("leg_2_linear_motor");
  spring_motor3       = wb_robot_get_device("leg_3_linear_motor");
  spring_motor4       = wb_robot_get_device("leg_4_linear_motor");
  spring_pos_sensor1  = wb_robot_get_device("leg_1_linear_motor_p_sensor");
  spring_pos_sensor2  = wb_robot_get_device("leg_2_linear_motor_p_sensor");
  spring_pos_sensor3  = wb_robot_get_device("leg_3_linear_motor_p_sensor");
  spring_pos_sensor4  = wb_robot_get_device("leg_4_linear_motor_p_sensor");
  touch_sensor1       = wb_robot_get_device("touch sensor 1");
  touch_sensor2       = wb_robot_get_device("touch sensor 2");
  touch_sensor3       = wb_robot_get_device("touch sensor 3");
  touch_sensor4       = wb_robot_get_device("touch sensor 4");
  IMU                 = wb_robot_get_device("inertial unit");
  X_motor1            = wb_robot_get_device("leg_1_rmotor_x");
  X_motor2            = wb_robot_get_device("leg_2_rmotor_x");
  X_motor3            = wb_robot_get_device("leg_3_rmotor_x");
  X_motor4            = wb_robot_get_device("leg_4_rmotor_x");
  Z_motor1            = wb_robot_get_device("leg_1_rmotor_z");
  Z_motor2            = wb_robot_get_device("leg_2_rmotor_z");
  Z_motor3            = wb_robot_get_device("leg_3_rmotor_z");
  Z_motor4            = wb_robot_get_device("leg_4_rmotor_z");
  X_motor_pos_sensor1 = wb_robot_get_device("leg_1_rmotor_x_sensor");
  X_motor_pos_sensor2 = wb_robot_get_device("leg_2_rmotor_x_sensor");
  X_motor_pos_sensor3 = wb_robot_get_device("leg_3_rmotor_x_sensor");
  X_motor_pos_sensor4 = wb_robot_get_device("leg_4_rmotor_x_sensor");
  Z_motor_pos_sensor1 = wb_robot_get_device("leg_1_rmotor_z_sensor");
  Z_motor_pos_sensor2 = wb_robot_get_device("leg_2_rmotor_z_sensor");
  Z_motor_pos_sensor3 = wb_robot_get_device("leg_3_rmotor_z_sensor");
  Z_motor_pos_sensor4 = wb_robot_get_device("leg_4_rmotor_z_sensor");
  //enable
  wb_position_sensor_enable(spring_pos_sensor1, TIME_STEP);
  wb_position_sensor_enable(spring_pos_sensor2, TIME_STEP);
  wb_position_sensor_enable(spring_pos_sensor3, TIME_STEP);
  wb_position_sensor_enable(spring_pos_sensor4, TIME_STEP);
  wb_position_sensor_enable(X_motor_pos_sensor1, TIME_STEP);
  wb_position_sensor_enable(X_motor_pos_sensor2, TIME_STEP);
  wb_position_sensor_enable(X_motor_pos_sensor3, TIME_STEP);
  wb_position_sensor_enable(X_motor_pos_sensor4, TIME_STEP);
  wb_position_sensor_enable(Z_motor_pos_sensor1, TIME_STEP);
  wb_position_sensor_enable(Z_motor_pos_sensor2, TIME_STEP);
  wb_position_sensor_enable(Z_motor_pos_sensor3, TIME_STEP);
  wb_position_sensor_enable(Z_motor_pos_sensor4, TIME_STEP);
  wb_touch_sensor_enable(touch_sensor1,TIME_STEP);
  wb_touch_sensor_enable(touch_sensor2,TIME_STEP);
  wb_touch_sensor_enable(touch_sensor3,TIME_STEP);
  wb_touch_sensor_enable(touch_sensor4,TIME_STEP);
  wb_keyboard_enable(TIME_STEP);
  wb_inertial_unit_enable(IMU,TIME_STEP);
}

//-----------------------------------------------------------motor
/*
函数功能：设置虚拟弹簧的力
注    意：弹簧力正方向定义为：腿不动，将机身向y轴方向推动的力
*/
void set_spring_force1(double force)
{
  wb_motor_set_force(spring_motor1, -force);
}

void set_spring_force2(double force)
{
  wb_motor_set_force(spring_motor2, -force);
}

void set_spring_force3(double force)
{
  wb_motor_set_force(spring_motor3, -force);
}

void set_spring_force4(double force)
{
  wb_motor_set_force(spring_motor4, -force);
}

/*
函数功能：设置臀部 X 轴电机的扭矩
注    意：扭矩正方向定义为：机身不动，使腿绕 X 轴旋转的力
*/
void set_X_torque1(double torque)
{
  wb_motor_set_torque(X_motor1, torque);
}

void set_X_torque2(double torque)
{
  wb_motor_set_torque(X_motor2, torque);
}

void set_X_torque3(double torque)
{
  wb_motor_set_torque(X_motor3, torque);
}

void set_X_torque4(double torque)
{
  wb_motor_set_torque(X_motor4, torque);
}

/*
函数功能：设置臀部 Z 轴电机的扭矩
注    意：扭矩正方向定义为：机身不动，使腿绕 Z 轴旋转的力
*/
void set_Z_torque1(double torque)
{
  wb_motor_set_torque(Z_motor1, torque);
}

void set_Z_torque2(double torque)
{
  wb_motor_set_torque(Z_motor2, torque);
}

void set_Z_torque3(double torque)
{
  wb_motor_set_torque(Z_motor3, torque);
}

void set_Z_torque4(double torque)
{
  wb_motor_set_torque(Z_motor4, torque);
}

/*
Set velocity of the motor at hip
*/
void set_X_velocity1(double v)
{
  wb_motor_set_acceleration(X_motor1, 1.0);
  wb_motor_set_velocity(X_motor1, v);
}
void set_X_velocity2(double v)
{
  wb_motor_set_acceleration(X_motor2, 1.0);
  wb_motor_set_velocity(X_motor2, v);
}
void set_X_velocity3(double v)
{
  wb_motor_set_acceleration(X_motor3, 1.0);
  wb_motor_set_velocity(X_motor3, v);
}
void set_X_velocity4(double v)
{
  wb_motor_set_acceleration(X_motor4, 1.0);
  wb_motor_set_velocity(X_motor4, v);
}
void set_Z_velocity1(double v)
{
  wb_motor_set_acceleration(Z_motor1, 1.0);
  wb_motor_set_velocity(Z_motor1, v);
}
void set_Z_velocity2(double v)
{
  wb_motor_set_acceleration(Z_motor2, 1.0);
  wb_motor_set_velocity(Z_motor2, v);
}
void set_Z_velocity3(double v)
{
  wb_motor_set_acceleration(Z_motor3, 1.0);
  wb_motor_set_velocity(Z_motor3, v);
}
void set_Z_velocity4(double v)
{
  wb_motor_set_acceleration(Z_motor4, 1.0);
  wb_motor_set_velocity(Z_motor4, v);
}

//-----------------------------------------------------------sensor
/*
函数功能：获取弹簧长度
*/
double get_spring_length1()
{
  double length = wb_position_sensor_get_value(spring_pos_sensor1);
  return -length + 0.6;
}

double get_spring_length2()
{
  double length = wb_position_sensor_get_value(spring_pos_sensor2);
  return -length + 0.6;
}

double get_spring_length3()
{
  double length = wb_position_sensor_get_value(spring_pos_sensor3);
  return -length + 0.6;
}

double get_spring_length4()
{
  double length = wb_position_sensor_get_value(spring_pos_sensor4);
  return -length + 0.6;
}

/*
函数功能：获取 X 轴电机角度,角度制
*/
double get_X_motor_angle1()
{
  double angle = wb_position_sensor_get_value(X_motor_pos_sensor1);
  return angle*180.0f/PI;
}

double get_X_motor_angle2()
{
  double angle = wb_position_sensor_get_value(X_motor_pos_sensor2);
  return angle*180.0f/PI;
}

double get_X_motor_angle3()
{
  double angle = wb_position_sensor_get_value(X_motor_pos_sensor3);
  return angle*180.0f/PI;
}

double get_X_motor_angle4()
{
  double angle = wb_position_sensor_get_value(X_motor_pos_sensor4);
  return angle*180.0f/PI;
}
/*
函数功能：获取 Z 轴电机角度,角度制
*/
double get_Z_motor_angle1()
{
  double angle = wb_position_sensor_get_value(Z_motor_pos_sensor1);
  return angle*180.0f/PI;
}

double get_Z_motor_angle2()
{
  double angle = wb_position_sensor_get_value(Z_motor_pos_sensor2);
  return angle*180.0f/PI;
}

double get_Z_motor_angle3()
{
  double angle = wb_position_sensor_get_value(Z_motor_pos_sensor3);
  return angle*180.0f/PI;
}

double get_Z_motor_angle4()
{
  double angle = wb_position_sensor_get_value(Z_motor_pos_sensor4);
  return angle*180.0f/PI;
}

/*
函数功能：检测足底是否接触地面
*/
bool is_foot_touching1()
{
  return wb_touch_sensor_get_value(touch_sensor1);
}

bool is_foot_touching2()
{
  return wb_touch_sensor_get_value(touch_sensor2);
}

bool is_foot_touching3()
{
  return wb_touch_sensor_get_value(touch_sensor3);
}

bool is_foot_touching4()
{
  return wb_touch_sensor_get_value(touch_sensor4);
}

/*
函数功能：读取IMU数据
*/
eulerAngleTypeDef get_IMU_Angle()
{
  const double* data = wb_inertial_unit_get_roll_pitch_yaw(IMU);
  
  eulerAngleTypeDef eulerAngle;
  eulerAngle.roll  = data[0]*180.0f/PI;
  eulerAngle.pitch = data[1]*180.0f/PI;
  eulerAngle.yaw   = data[2]*180.0f/PI;
  
  return eulerAngle;
}

//-----------------------------------------------------------keyboard
/*
函数功能：读取键盘键值
*/
int get_keyboard()
{
  return wb_keyboard_get_key();
}

