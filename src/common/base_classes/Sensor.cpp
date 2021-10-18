#include "Sensor.h"
#include "../foc_utils.h"
#include "../time_utils.h"
#include "fix.h"

// TODO 添加一个初始化方法，通过将内部变量初始化为当前值而不是0，使启动更加顺畅。

void Sensor::update() {
    float val = getSensorAngle();
    angle_prev_ts = _micros();
    float d_angle = val - angle_prev;
    // if overflow happened track it as full rotation
    if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
}


 /** 获取当前角速度 (rad/s) */
float Sensor::getVelocity() {
    // 计算采样时间
    float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6;
    // 对奇怪情况的快速修复（微溢出）。
    if(Ts <= 0) Ts = 1e-3f;
    // 速度计算
    float vel = ( (float)(full_rotations - vel_full_rotations)*_2PI + (angle_prev - vel_angle_prev) ) / Ts;    
    // 保存变量以备将来使用
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return vel;
}

void Sensor::init() {
    // 初始化传感器的所有内部变量，以确保 "顺利 "启动（没有从零开始的 "跳跃"）。
    getSensorAngle(); // 调用一次
    delayMicroseconds(1);//微秒
    vel_angle_prev = getSensorAngle(); // 再次调用
    vel_angle_prev_ts = _micros();
    delay(1);
    getSensorAngle(); // 调用一次
    delayMicroseconds(1);
    angle_prev = getSensorAngle(); // 再次调用
    angle_prev_ts = _micros();
}


float Sensor::getMechanicalAngle() {
    return angle_prev;
}



float Sensor::getAngle(){
    return (float)full_rotations * _2PI + angle_prev;
}



double Sensor::getPreciseAngle() {
    return (double)full_rotations * (double)_2PI + (double)angle_prev;
}



int32_t Sensor::getFullRotations() {
    return full_rotations;
}



int Sensor::needsSearch() {
    return 0; // 默认为 false
}
