#include "pid.h"

PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)    // 输出导数限制 [ volts / second ]
    , limit(limit)         // 输出电源限制     [volts]
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
{
    timestamp_prev = _micros();
}

// PID controller function
float PIDController::operator() (float error){
    // 计算从最后一次调用的时间
    unsigned long timestamp_now = _micros();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f;
    // quick fix for strange cases (micros overflow)
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // u(s) = (P + I/s + Ds)e(s)
    // 离散的实现方式
    // 成比例的部分
    // u_p  = P *e(k)
    float proportional = P * error;
    // 积分部分的塔斯廷变换
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    float integral = integral_prev + I*Ts*0.5f*(error + error_prev);
    // 抗倒伏 - 限制输出
    integral = _constrain(integral, -limit, limit);
    // 离散推导
    // u_dk = D(ek - ek_1)/Ts
    float derivative = D*(error - error_prev)/Ts;

    // 所有成分的总和
    float output = proportional + integral + derivative;
    // antiwindup - 限制输出变量
    output = _constrain(output, -limit, limit);

    // 如果定义了输出斜率
    if(output_ramp > 0){
        // 通过斜坡输出来限制加速度
        float output_rate = (output - output_prev)/Ts;
        if (output_rate > output_ramp)
            output = output_prev + output_ramp*Ts;
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp*Ts;
    }
    // 为下一次的PID做准备
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    return output;
}
