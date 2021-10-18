#include "InlineCurrentSense.h"
#include "fix.h"
// InlineCurrentSensor构造函数
// - 分流电阻 - 分流电阻阻值
// - 增益 - 电流感应运算放大器增益
// - phA - A相adc引脚
// - phB - B相adc引脚
// - phC - C相adc引脚（可选）。
InlineCurrentSense::InlineCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC){
    pinA = _pinA;
    pinB = _pinB;
    pinC = _pinC;

    shunt_resistor = _shunt_resistor;
    amp_gain  = _gain;
    volts_to_amps_ratio = 1.0f /_shunt_resistor / _gain; // 电压转安培
    // 电机三相的增益
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
}

// Inline sensor init function
void InlineCurrentSense::init(){
    // configure ADC variables
    _configureADCInline(pinA,pinB,pinC);
    // calibrate zero offsets
    calibrateOffsets();
}
// 寻找ADC零点偏移的功能
void InlineCurrentSense::calibrateOffsets(){
    const int calibration_rounds = 1000;//读取adc电压次数次(越多越好)

    //找到adc偏移=零电流电压
    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;
    // 读取adc电压1000次（任意数字）
    for (int i = 0; i < calibration_rounds; i++) {
        offset_ia += _readADCVoltageInline(pinA);
        offset_ib += _readADCVoltageInline(pinB);
        if(_isset(pinC)) offset_ic += _readADCVoltageInline(pinC);
        _delay(1);
    }
    //计算平均偏移量
    offset_ia = offset_ia / calibration_rounds;
    offset_ib = offset_ib / calibration_rounds;
    if(_isset(pinC)) offset_ic = offset_ic / calibration_rounds;
}

// 读取所有三相电流（如果可能是2或3）。
PhaseCurrent_s InlineCurrentSense::getPhaseCurrents(){
    PhaseCurrent_s current;
    current.a = (_readADCVoltageInline(pinA) - offset_ia)*gain_a;// 安培
    current.b = (_readADCVoltageInline(pinB) - offset_ib)*gain_b;// 安培
    current.c = (!_isset(pinC)) ? 0 : (_readADCVoltageInline(pinC) - offset_ic)*gain_c; // 安培
    return current;
}
// 使电流感应与电机驱动器同步的功能
// for in-line sensig no such thing is necessary
int InlineCurrentSense::driverSync(BLDCDriver *driver){
    return 1;
}

// 将电流感应与电机驱动器对齐的功能
// 如果所有的引脚都连接得很好，那么这些都是不必要的。- 可以避免
// return flag
// 0 - 失败
// 1 - 成功，没有变化
// 2 - 成功，但引脚被重新配置
// 3 - 成功，但收益倒置
// 4 - 成功，但引脚被重新配置，收益倒置
int InlineCurrentSense::driverAlign(BLDCDriver *driver, float voltage){
    int exit_flag = 1;
    if(skip_align) return exit_flag;

    //设置A相为激活状态，B相和C相为关闭状态
    driver->setPwm(voltage, 0, 0);
    _delay(200);
    PhaseCurrent_s c = getPhaseCurrents();
    // 读取当前的100次（任意的数字）。
    for (int i = 0; i < 100; i++) {
        PhaseCurrent_s c1 = getPhaseCurrents();
        c.a = c.a*0.6f + 0.4f*c1.a;
        c.b = c.b*0.6f + 0.4f*c1.b;
        c.c = c.c*0.6f + 0.4f*c1.c;
        _delay(3);
    }
    driver->setPwm(0, 0, 0);
    // align phase A
    float ab_ratio = fabs(c.a / c.b);
    float ac_ratio = c.c ? fabs(c.a / c.c) : 0;
    if( ab_ratio > 1.5f ){ // 应该接近2.0
        gain_a *= _sign(c.a);
    }else if( ab_ratio < 0.7f ){ //应该接近0.5
        //切换A相和B相
        int tmp_pinA = pinA;
        pinA = pinB;
        pinB = tmp_pinA;
        gain_a *= _sign(c.b);
        exit_flag = 2; //发出信号表明引脚已被切换
    }else if(_isset(pinC) &&  ac_ratio < 0.7f ){ // //应该接近0.5
        //切换A相和C相
        int tmp_pinA = pinA;
        pinA = pinC;
        pinC= tmp_pinA;
        gain_a *= _sign(c.c);
        exit_flag = 2;//发出信号表明引脚已被切换
    }else{
        // 电流感应错误--相位未测量或连接不良
        return 0;
    }

    // set phase B active and phases A and C down
    driver->setPwm(0, voltage, 0);
    _delay(200);
    c = getPhaseCurrents();
    // read the current 50 times
    for (int i = 0; i < 100; i++) {
        PhaseCurrent_s c1 = getPhaseCurrents();
        c.a = c.a*0.6f + 0.4f*c1.a;
        c.b = c.b*0.6f + 0.4f*c1.b;
        c.c = c.c*0.6f + 0.4f*c1.c;
        _delay(3);
    }
    driver->setPwm(0, 0, 0);
    float ba_ratio = fabs(c.b/c.a);
    float bc_ratio = c.c ? fabs(c.b / c.c) : 0;
     if( ba_ratio > 1.5f ){ // 应该接近2.0
        gain_b *= _sign(c.b);
    }else if( ba_ratio < 0.7f ){ // 应该接近0.5
        // switch phase A and B
        int tmp_pinB = pinB;
        pinB = pinA;
        pinA = tmp_pinB;
        gain_b *= _sign(c.a);
        exit_flag = 2; // 引脚已被切换的信号
    }else if(_isset(pinC) && bc_ratio < 0.7f ){ // 应该接近0.5
        // switch phase A and C
        int tmp_pinB = pinB;
        pinB = pinC;
        pinC = tmp_pinB;
        gain_b *= _sign(c.c);
        exit_flag = 2; // 引脚已被切换的信号
    }else{
        // error in current sense - phase either not measured or bad connection
        return 0;
    }

    // if phase C measured
    if(_isset(pinC)){
        // set phase B active and phases A and C down
        driver->setPwm(0, 0, voltage);
        _delay(200);
        c = getPhaseCurrents();
        // read the adc voltage 500 times ( arbitrary number )
        for (int i = 0; i < 50; i++) {
            PhaseCurrent_s c1 = getPhaseCurrents();
            c.c = (c.c+c1.c)/50.0f;
        }
        driver->setPwm(0, 0, 0);
        gain_c *= _sign(c.c);
    }

    if(gain_a < 0 || gain_b < 0 || gain_c < 0) exit_flag +=2;
    // exit flag is either
    // 0 - fail
    // 1 - success and nothing changed
    // 2 - success but pins reconfigured
    // 3 - success but gains inverted
    // 4 - success but pins reconfigured and gains inverted
    return exit_flag;
}
