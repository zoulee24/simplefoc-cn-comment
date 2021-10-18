#include "CurrentSense.h"


// 获取电流大小 
//   - 绝对值 - 如果没有提供电角度 
//   - signed    - if angle provided
float CurrentSense::getDCCurrent(float motor_electrical_angle){
    // read current phase currents
    PhaseCurrent_s current = getPhaseCurrents();
    // currnet sign - 如果没有提供电机角度，其幅度总是正的。
    float sign = 1;

    // 计算 clarke 变换
    float i_alpha, i_beta;
    if(!current.c){
        // 如果只有两个电流检测芯片
        i_alpha = current.a;  
        i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
    }else{
        //信号滤波，使用特征a+b+c=0。假设测量误差为正态分布。
        float mid = (1.f/3) * (current.a + current.b + current.c);
        float a = current.a - mid;
        float b = current.b - mid;
        i_alpha = a;
        i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
    }

    // 如果提供电机角度，则函数返回有符号的电流值
    // 确定当前的符号
    // sign(atan2(current.q, current.d)) is the same as c.q > 0 ? 1 : -1  
    if(motor_electrical_angle) 
        sign = (i_beta * _cos(motor_electrical_angle) - i_alpha*_sin(motor_electrical_angle)) > 0 ? 1 : -1;  
    // return current magnitude
    return sign*_sqrt(i_alpha*i_alpha + i_beta*i_beta);
}

// 与foc algorihtm一起使用的函数 
//  从相电流计算DQ电流
//   - 计算相电流的Park和Clarke变换的函数 
//   - using getPhaseCurrents internally
DQCurrent_s CurrentSense::getFOCCurrents(float angle_el){
    // read current phase currents
    PhaseCurrent_s current = getPhaseCurrents();

    // 计算 clarke 变换
    float i_alpha, i_beta;
    if(!current.c){
        // if only two measured currents
        i_alpha = current.a;  
        i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
    } else {
        // signal filtering using identity a + b + c = 0. Assumes measurement error is normally distributed.
        float mid = (1.f/3) * (current.a + current.b + current.c);
        float a = current.a - mid;
        float b = current.b - mid;
        i_alpha = a;
        i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
    }

    // 计算park变换
    float ct = _cos(angle_el);
    float st = _sin(angle_el);
    DQCurrent_s return_current;
    return_current.d = i_alpha * ct + i_beta * st;
    return_current.q = i_beta * ct - i_alpha * st;
    return return_current;
}