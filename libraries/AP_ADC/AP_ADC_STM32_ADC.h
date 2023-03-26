#pragma once

#include <inttypes.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

struct adc_report_s2
{
    uint8_t id;
    float data;
};

class AP_ADC_STM32_ADC
{
public:
    AP_ADC_STM32_ADC(const uint8_t bus);
    ~AP_ADC_STM32_ADC();

    bool init();
    size_t read(adc_report_s2 *report, size_t length) const;

    uint8_t get_channels_number() const
    {
        return _channels_number;
    }

private:
    static const uint8_t _channels_number;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    int                 _channel_to_read;
    adc_report_s2       *_samples;
    uint8_t _bus;
    
    void _update();

};
