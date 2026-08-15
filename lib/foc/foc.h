//灯哥开源，遵循GNU协议，转载请著名版权！
//FOC 控制核心库 —— 独立组件，不依赖应用层（串口等）
//仅在DengFOC官方硬件上测试过，欢迎硬件购买/支持作者，淘宝搜索店铺：灯哥开源
#ifndef FOC_H
#define FOC_H

#include <Arduino.h>

//电源电压设置 + PWM 初始化
void DFOC_Vbus(float power_supply);
//传感器电角度校准
void DFOC_alignSensor(int _PP,int _DIR);
//电角度
float _electricalAngle();
//传感器读取
float DFOC_M0_Velocity();
float DFOC_M0_Angle();
//PID 参数设置与调用
void DFOC_M0_SET_ANGLE_PID(float P,float I,float D,float ramp);
void DFOC_M0_SET_VEL_PID(float P,float I,float D,float ramp);
float DFOC_M0_VEL_PID(float error);
float DFOC_M0_ANGLE_PID(float error);
//接口函数
void DFOC_M0_set_Velocity_Angle(float Target);
void DFOC_M0_setVelocity(float Target);
void DFOC_M0_set_Force_Angle(float Target);
void DFOC_M0_setTorque(float Target);

#endif // FOC_H
