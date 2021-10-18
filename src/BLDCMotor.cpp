#include "BLDCMotor.h"
#include "math.h"
#include "Print.h"
#include "fix.h"

// BLDCMotor( int pp , float R)
// - pp            - 机极点对数
// - R             - 电机相位电阻
BLDCMotor::BLDCMotor(int pp, float _R)
: FOCMotor()
{
  //保存电机极点对数
  pole_pairs = pp;
  // 保存电机相位电阻
  phase_resistance = _R;
  // 默认情况下，扭矩控制类型为电压
  torque_controller = TorqueControlType::voltage;
}


/**
	连接控制电机的驱动器
*/
void BLDCMotor::linkDriver(BLDCDriver* _driver) {
  driver = _driver;
}

// init hardware pins
void BLDCMotor::init() {
  if(monitor_port) monitor_port->println("MOT: Init");

  // 如果没有电流感应，并且用户已经设置了电机的相位电阻，则使用电流限制来计算电压限制。
  if( !current_sense && _isset(phase_resistance)) {
    float new_voltage_limit = current_limit * (phase_resistance); // v_lim = current_lim / (3/2 phase resistance) - worst case
    // 如果它小于用户设置的电压限制，则使用它。
    voltage_limit = new_voltage_limit < voltage_limit ? new_voltage_limit : voltage_limit;
  }
  // 电压限制配置的正确性检查
  if(voltage_limit > driver->voltage_limit) voltage_limit =  driver->voltage_limit;
  // 用于传感器校准的约束电压
  if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;

  // 更新控制器限制
  if(current_sense){
    // 电流控制回路控制电压
    PID_current_q.limit = voltage_limit;
    PID_current_d.limit = voltage_limit;
    // 速度控制回路控制电流
    PID_velocity.limit = current_limit;
  }else if(!current_sense && _isset(phase_resistance)){
    PID_velocity.limit = current_limit;
  }else{
    PID_velocity.limit = voltage_limit;
  }
  P_angle.limit = velocity_limit;

  _delay(500);
  // 使能电机
  if(monitor_port) monitor_port->println("MOT: Enable driver.");
  enable();
  _delay(500);
}


// 禁用电机驱动器
void BLDCMotor::disable()
{
  // 设置PWM占空比为0
  driver->setPwm(0, 0, 0);
  // 禁用电机
  driver->disable();
  // 电机状态更新
  enabled = 0;
}
// 使能电机驱动器
void BLDCMotor::enable()
{
  // 使能电机
  driver->enable();
  // 设置PWM占空比为0
  driver->setPwm(0, 0, 0);
  // 电机状态更新
  enabled = 1;
}

/**
  FOC functions
*/
// 功能：FOC初始化
int  BLDCMotor::initFOC( float zero_electric_offset, Direction _sensor_direction) {
  int exit_flag = 1;
  // 如果有必要，对准电机
  //编码器需要对准！对准是必要的。
  if(_isset(zero_electric_offset)){
    // 提供了绝对零度的偏移量--不需要对齐
    zero_electric_angle = zero_electric_offset;
    //设置传感器方向--默认为CW
    sensor_direction = _sensor_direction;
  }

  // 传感器和电机对准 - 可以跳过
  // 通过设置电机和传感器方向和电机.零电角度来实现。
  _delay(500);
  if(sensor){
    exit_flag *= alignSensor();
    // 添加了轴心角的更新
    sensor->update();
    shaft_angle = shaftAngle();
  }else if(monitor_port) monitor_port->println("MOT: No sensor.");

  // 对准电流传感器 - 可以跳过
  // 检查驱动相位是否与电流感应相位相同
  // 并检查测量的方向。
  _delay(500);
  if(exit_flag){
    if(current_sense) exit_flag *= alignCurrentSense();
    else if(monitor_port) monitor_port->println("MOT: No current sense.");
  }

  if(exit_flag){
    if(monitor_port) monitor_port->println("MOT: Ready.");
  }else{
    if(monitor_port) monitor_port->println("MOT: Init FOC failed.");
    disable();
  }

  return exit_flag;
}

// 校准电机和电流感应相位
int BLDCMotor::alignCurrentSense() {
  int exit_flag = 1; // success

  if(monitor_port) monitor_port->println("MOT: Align current sense.");

  //对准电流感应和驱动器
  exit_flag = current_sense->driverAlign(driver, voltage_sensor_align);
  if(!exit_flag){
    // 电流感应错误--相位未测量或连接不良
    if(monitor_port) monitor_port->println("MOT: Align error!");
    exit_flag = 0;
  }else{
    // 输出对齐状态标志
    if(monitor_port) monitor_port->print("MOT: Success: ");
    if(monitor_port) monitor_port->println(exit_flag);
  }

  return exit_flag > 0;
}

// 编码器对准电气0角度
int BLDCMotor::alignSensor() {
  int exit_flag = 1; //成功
  if(monitor_port) monitor_port->println("MOT: Align sensor.");

  // 如果未知的自然方向
  if(!_isset(sensor_direction)){
    // 检查传感器是否需要搜零
    if(sensor->needsSearch()) exit_flag = absoluteZeroSearch();
    // 如果没有找到索引就停止启动
    if(!exit_flag) return exit_flag;

    // 找到自然的方向
    // move one electrical revolution forward
    for (int i = 0; i <=500; i++ ) {
      float angle = _3PI_2 + _2PI * i / 500.0f;
      setPhaseVoltage(voltage_sensor_align, 0,  angle);
      _delay(2);
    }
    // 取中间角
    sensor->update();
    float mid_angle = sensor->getAngle();
    // move one electrical revolution backwards
    for (int i = 500; i >=0; i-- ) {
      float angle = _3PI_2 + _2PI * i / 500.0f ;
      setPhaseVoltage(voltage_sensor_align, 0,  angle);
      _delay(2);
    }
    sensor->update();
    float end_angle = sensor->getAngle();
    setPhaseVoltage(0, 0, 0);
    _delay(200);
    // 确定传感器的移动方向
    if (mid_angle == end_angle) {
      if(monitor_port) monitor_port->println("MOT: Failed to notice movement");
      return 0; // 校准失败
    } else if (mid_angle < end_angle) {
      if(monitor_port) monitor_port->println("MOT: sensor_direction==CCW");
      sensor_direction = Direction::CCW;
    } else{
      if(monitor_port) monitor_port->println("MOT: sensor_direction==CW");
      sensor_direction = Direction::CW;
    }
    // 检查电机极对数量
    if(monitor_port) monitor_port->print("MOT: 检查电机极对数: ");
    float moved =  fabs(mid_angle - end_angle);
    if( fabs(moved*pole_pairs - _2PI) > 0.5f ) { // 0.5f是任意的数字，它可以更低或更高!
      if(monitor_port) monitor_port->print("fail - estimated pp:");
      if(monitor_port) monitor_port->println(_2PI/moved,4);
    }else if(monitor_port) monitor_port->println("OK!");

  }else if(monitor_port) monitor_port->println("MOT: Skip dir calib.");

  // 零电角度不详
  if(!_isset(zero_electric_angle)){
    // 对齐电机和传感器的电相
    // 设置角度-90(270=3*PI/2)度
    setPhaseVoltage(voltage_sensor_align, 0,  _3PI_2);
    _delay(700);
    // read the sensor
    sensor->update();
    // get the current zero electric angle
    zero_electric_angle = 0;
    zero_electric_angle = electricalAngle();
    //zero_electric_angle =  _normalizeAngle(_electricalAngle(sensor_direction*sensor->getAngle(), pole_pairs));
    _delay(20);
    if(monitor_port){
      monitor_port->print("MOT: Zero elec. angle: ");
      monitor_port->println(zero_electric_angle);
    }
    // stop everything
    setPhaseVoltage(0, 0, 0);
    _delay(200);
  }else if(monitor_port) monitor_port->println("MOT: Skip offset calib.");
  return exit_flag;
}

// 编码器对准绝对零度角
// - to the index
int BLDCMotor::absoluteZeroSearch() {
  // 传感器精度：这都是可以的，因为搜索发生在0角附近，在那里，浮点数的精度是足够的。
  if(monitor_port) monitor_port->println("MOT: Index search...");
  // 以小速度搜索绝对零度
  float limit_vel = velocity_limit;
  float limit_volt = voltage_limit;
  velocity_limit = velocity_index_search;
  voltage_limit = voltage_sensor_align;
  shaft_angle = 0;
  while(sensor->needsSearch() && shaft_angle < _2PI){
    angleOpenloop(1.5f*_2PI);
    // 调用一些重要的传感器，以避免松动计数
    // 在搜索中不需要
    sensor->update();
  }
  // 禁用电机
  setPhaseVoltage(0, 0, 0);
  // 重新设定速度和电压限制
  velocity_limit = limit_vel;
  voltage_limit = limit_volt;
  // 检查是否找到了零点
  if(monitor_port){
    if(sensor->needsSearch()) monitor_port->println("MOT: Error: Not found!");
    else monitor_port->println("MOT: Success!");
  }
  return !sensor->needsSearch();
}

// 循环FOC算法的迭代函数，在电机上设置Uq
// 运行得越快越好
void BLDCMotor::loopFOC() {
  //更新传感器--即使在开环模式下也要这样做，因为用户可能会在不同的模式之间切换，否则我们可能会失去对全部旋转的跟踪。
  if (sensor) sensor->update();

  // 如果开环，不运行
  if( controller==MotionControlType::angle_openloop || controller==MotionControlType::velocity_openloop ) return;
  
  // 如果禁用，不运行
  if(!enabled) return;

  // 需要先调用 update()
  // 这个函数不会有精度问题，因为它使用了传感器::getMechanicalAngle()，其范围是0-2PI。
  electrical_angle = electricalAngle();
  switch (torque_controller) {
    case TorqueControlType::voltage:
      // 无
      break;
    case TorqueControlType::dc_current:
      if(!current_sense) return;
      // 读取整体电流大小
      current.q = current_sense->getDCCurrent(electrical_angle);
      // 滤波电流
      current.q = LPF_current_q(current.q);
      // 计算相位电压
      voltage.q = PID_current_q(current_sp - current.q);
      voltage.d = 0;
      break;
    case TorqueControlType::foc_current:
      if(!current_sense) return;
      // 读取Id和Iq
      current = current_sense->getFOCCurrents(electrical_angle);
      // 滤波Iq和Id
      current.q = LPF_current_q(current.q);
      current.d = LPF_current_d(current.d);
      // 计算相电压
      voltage.q = PID_current_q(current_sp - current.q);
      voltage.d = PID_current_d(-current.d);
      break;
    default:
      // 没有选择扭矩控制
      if(monitor_port) monitor_port->println("MOT: 没有选择扭矩控制!");
      break;
  }

  //! 设置相位电压--FOC最重要功能 
  setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}

// 运行FOC算法外循环的迭代函数
// 这个函数的行为是由电机控制器变量决定的。
// 它可以运行角度、速度或扭矩循环
// - 需要迭代调用，它是一个异步函数
// 如果目标没有设置，则使用电机的目标值。
void BLDCMotor::move(float new_target) {

  // downsampling (optional)
  if(motion_cnt++ < motion_downsample) return;
  motion_cnt = 0;

  // shaft angle/velocity need the update() to be called first
  // get shaft angle
  // TODO sensor precision: the shaft_angle actually stores the complete position, including full rotations, as a float
  //                        For this reason it is NOT precise when the angles become large.
  //                        Additionally, the way LPF works on angle is a precision issue, and the angle-LPF is a problem
  //                        when switching to a 2-component representation.
  if( controller!=MotionControlType::angle_openloop && controller!=MotionControlType::velocity_openloop ) 
    shaft_angle = shaftAngle(); // read value even if motor is disabled to keep the monitoring updated but not in openloop mode
  // get angular velocity 
  shaft_velocity = shaftVelocity(); // read value even if motor is disabled to keep the monitoring updated

  // if disabled do nothing
  if(!enabled) return;
  // set internal target variable
  if(_isset(new_target)) target = new_target;

  switch (controller) {
    case MotionControlType::torque:
      if(torque_controller == TorqueControlType::voltage){ // if voltage torque control
        if(!_isset(phase_resistance))  voltage.q = target;
        else voltage.q =  target*phase_resistance;
        voltage.d = 0;
      }else{
        current_sp = target; // if current/foc_current torque control
      }
      break;
    case MotionControlType::angle:
      // TODO sensor precision: this calculation is not numerically precise. The target value cannot express precise positions when
      //                        the angles are large. This results in not being able to command small changes at high position values.
      //                        to solve this, the delta-angle has to be calculated in a numerically precise way.
      // angle set point
      shaft_angle_sp = target;
      // calculate velocity set point
      shaft_velocity_sp = P_angle( shaft_angle_sp - shaft_angle );
      // calculate the torque command - sensor precision: this calculation is ok, but based on bad value from previous calculation
      current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // if voltage torque control
      // if torque controlled through voltage
      if(torque_controller == TorqueControlType::voltage){
        // use voltage if phase-resistance not provided
        if(!_isset(phase_resistance))  voltage.q = current_sp;
        else  voltage.q = current_sp*phase_resistance;
        voltage.d = 0;
      }
      break;
    case MotionControlType::velocity:
      // velocity set point - sensor precision: this calculation is numerically precise.
      shaft_velocity_sp = target;
      // calculate the torque command
      current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // if current/foc_current torque control
      // if torque controlled through voltage control
      if(torque_controller == TorqueControlType::voltage){
        // use voltage if phase-resistance not provided
        if(!_isset(phase_resistance))  voltage.q = current_sp;
        else  voltage.q = current_sp*phase_resistance;
        voltage.d = 0;
      }
      break;
    case MotionControlType::velocity_openloop:
      // velocity control in open loop - sensor precision: this calculation is numerically precise.
      shaft_velocity_sp = target;
      voltage.q = velocityOpenloop(shaft_velocity_sp); // returns the voltage that is set to the motor
      voltage.d = 0;
      break;
    case MotionControlType::angle_openloop:
      // angle control in open loop - 
      // TODO sensor precision: this calculation NOT numerically precise, and subject
      //                        to the same problems in small set-point changes at high angles 
      //                        as the closed loop version.
      shaft_angle_sp = target;
      voltage.q = angleOpenloop(shaft_angle_sp); // returns the voltage that is set to the motor
      voltage.d = 0;
      break;
  }
}


// 使用FOC将Uq和Ud设置为电机的最佳角度的方法
// 实现空间矢量PWM和正弦PWM算法的函数
//
// 使用正弦近似法的函数
// 常规 sin + cos ~300us (无内存使用)
// 近似_sin + _cos ~110us (400Byte ~ 20%的内存)
void BLDCMotor::setPhaseVoltage(float Uq, float Ud, float angle_el) {

  float center;
  int sector;
  float _ca,_sa;

  switch (foc_modulation)
  {
    //SVPWM模式
    case FOCModulationType::SpaceVectorPWM :
      // Nice video explaining the SpaceVectorModulation (SVPWM) algorithm
      // https://www.youtube.com/watch?v=QMSWUMEAejg

      // the algorithm goes
      // 1) Ualpha, Ubeta
      // 2) Uout = sqrt(Ualpha^2 + Ubeta^2)
      // 3) angle_el = atan2(Ubeta, Ualpha)
      //
      // equivalent to 2)  because the magnitude does not change is:
      // Uout = sqrt(Ud^2 + Uq^2)
      // equivalent to 3) is
      // angle_el = angle_el + atan2(Uq,Ud)

      float Uout;
      // a bit of optitmisation
      if(Ud){ // only if Ud and Uq set
        // _sqrt is an approx of sqrt (3-4% error)
        Uout = _sqrt(Ud*Ud + Uq*Uq) / driver->voltage_limit;
        // angle normalisation in between 0 and 2pi
        // only necessary if using _sin and _cos - approximation functions
        angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
      }else{// only Uq available - no need for atan2 and sqrt
        Uout = Uq / driver->voltage_limit;
        // angle normalisation in between 0 and 2pi
        // only necessary if using _sin and _cos - approximation functions
        angle_el = _normalizeAngle(angle_el + _PI_2);
      }
      // find the sector we are in currently
      sector = floor(angle_el / _PI_3) + 1;
      // calculate the duty cycles
      float T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uout;
      float T2 = _SQRT3*_sin(angle_el - (sector-1.0f)*_PI_3) * Uout;
      // two versions possible
      float T0 = 0; // pulled to 0 - better for low power supply voltage
      if (modulation_centered) {
        T0 = 1 - T1 - T2; // modulation_centered around driver->voltage_limit/2
      }

      // calculate the duty cycles(times)
      float Ta,Tb,Tc;
      switch(sector){
        case 1:
          Ta = T1 + T2 + T0/2;
          Tb = T2 + T0/2;
          Tc = T0/2;
          break;
        case 2:
          Ta = T1 +  T0/2;
          Tb = T1 + T2 + T0/2;
          Tc = T0/2;
          break;
        case 3:
          Ta = T0/2;
          Tb = T1 + T2 + T0/2;
          Tc = T2 + T0/2;
          break;
        case 4:
          Ta = T0/2;
          Tb = T1+ T0/2;
          Tc = T1 + T2 + T0/2;
          break;
        case 5:
          Ta = T2 + T0/2;
          Tb = T0/2;
          Tc = T1 + T2 + T0/2;
          break;
        case 6:
          Ta = T1 + T2 + T0/2;
          Tb = T0/2;
          Tc = T1 + T0/2;
          break;
        default:
         // possible error state
          Ta = 0;
          Tb = 0;
          Tc = 0;
      }
      default:
        Print a;
        a.println("错误");
        break; // unknown

      // calculate the phase voltages and center
      Ua = Ta*driver->voltage_limit;
      Ub = Tb*driver->voltage_limit;
      Uc = Tc*driver->voltage_limit;
      break;
  }

  // 设置驱动器中的电压
  driver->setPwm(Ua, Ub, Uc);
}



// 产生目标速度的开环运动的函数（迭代式）。
// - 目标速度 - rad/s
// 它使用电压限制变量
float BLDCMotor::velocityOpenloop(float target_velocity){
  // 获得当前时间戳
  unsigned long now_us = _micros();
  //计算从上次调用开始的采样时间
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  //快速修复奇怪的情况（micros溢出+未定义时间戳）。
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // 计算必要的角度以达到目标速度
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
  // 用于显示目的
  shaft_velocity = target_velocity;

  //使用电压限制或电流限制
  float Uq = voltage_limit;
  if(_isset(phase_resistance)) Uq =  current_limit*phase_resistance;

  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;

  return Uq;
}

// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and velocity_limit variables
float BLDCMotor::angleOpenloop(float target_angle){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // calculate the necessary angle to move from current position towards target angle
  // with maximal velocity (velocity_limit)
  // TODO sensor precision: this calculation is not numerically precise. The angle can grow to the point
  //                        where small position changes are no longer captured by the precision of floats
  //                        when the total position is large.
  if(abs( target_angle - shaft_angle ) > abs(velocity_limit*Ts)){
    shaft_angle += _sign(target_angle - shaft_angle) * abs( velocity_limit )*Ts;
    shaft_velocity = velocity_limit;
  }else{
    shaft_angle = target_angle;
    shaft_velocity = 0;
  }


  // use voltage limit or current limit
  float Uq = voltage_limit;
  if(_isset(phase_resistance)) Uq =  current_limit*phase_resistance;
  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  // sensor precision: this calculation is OK due to the normalisation
  setPhaseVoltage(Uq,  0, _electricalAngle(_normalizeAngle(shaft_angle), pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;

  return Uq;
}
