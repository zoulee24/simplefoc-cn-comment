#ifndef INLINE_CS_LIB_H
#define INLINE_CS_LIB_H

// #include "Arduino.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/defaults.h"
#include "../common/base_classes/CurrentSense.h"
#include "../common/lowpass_filter.h"
#include "hardware_api.h"


class InlineCurrentSense: public CurrentSense{
  public:
    /**
      InlineCurrentSense类构造函数
      @param shunt_resistor 分流电阻阻值
      @param gain 电流感测运算放大器增益
      @param phA A phase adc pin
      @param phB B phase adc pin
      @param phC C phase adc pin (可选)
    */
    InlineCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = NOT_SET);

    // CurrentSense interface implementing functions 
    void init() override;
    PhaseCurrent_s getPhaseCurrents() override;
    int driverSync(BLDCDriver *driver) override;
    int driverAlign(BLDCDriver *driver, float voltage) override;

    // 每个相位的ADC测量网增益
    // 支持对更常见的倒相电流的不同相位的不同增益。
    // 这在以后应该是自动化的
    float gain_a; //!< A相增益
    float gain_b; //!< B相增益
    float gain_c; //!< C相增益

    // // per phase low pass fileters
    // LowPassFilter lpf_a{DEF_LPF_PER_PHASE_CURRENT_SENSE_Tf}; //!<  current A low pass filter
    // LowPassFilter lpf_b{DEF_LPF_PER_PHASE_CURRENT_SENSE_Tf}; //!<  current B low pass filter
    // LowPassFilter lpf_c{DEF_LPF_PER_PHASE_CURRENT_SENSE_Tf}; //!<  current C low pass filter

  private:
  
    // hardware variables
  	int pinA; //!< pin A analog pin for current measurement
  	int pinB; //!< pin B analog pin for current measurement
  	int pinC; //!< pin C analog pin for current measurement

    // gain variables
    float shunt_resistor; //!< Shunt resistor value
    float amp_gain; //!< amp gain value
    float volts_to_amps_ratio; //!< Volts to amps ratio
    
    /**
     *  Function finding zero offsets of the ADC
     */
    void calibrateOffsets();
    float offset_ia; //!< zero current A voltage value (center of the adc reading)
    float offset_ib; //!< zero current B voltage value (center of the adc reading)
    float offset_ic; //!< zero current C voltage value (center of the adc reading)

};

#endif