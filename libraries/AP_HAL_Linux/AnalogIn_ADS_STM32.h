#pragma once

#include "AP_HAL_Linux.h"
#include <AP_ADC/AP_ADC_STM32_ADC.h>

#define ADS_STM32_ADC_MAX_CHANNELS 4

class AnalogSource_ADS_STM32: public AP_HAL::AnalogSource {
public:
    friend class AnalogIn_ADS_STM32;
    AnalogSource_ADS_STM32(int16_t pin);
    float read_average() override;
    float read_latest() override;
    bool set_pin(uint8_t p) override;
    float voltage_average() override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override;
private:
    int16_t _pin;
    float _value;
};

class AnalogIn_ADS_STM32: public AP_HAL::AnalogIn {
public:
    AnalogIn_ADS_STM32();

    void init() override;
    AP_HAL::AnalogSource *channel(int16_t n) override;

    /* Board voltage is not available */
    float board_voltage() override { return 5.0f; }

private:
    uint8_t _channels_number;
    void _update();

    AP_ADC_STM32_ADC *_adc;
    AnalogSource_ADS_STM32 *_channels[ADS_STM32_ADC_MAX_CHANNELS];
    uint32_t _last_update_timestamp;
    HAL_Semaphore _semaphore;
};
