#include "AnalogIn_ADS_STM32.h"
#include <stdio.h>

AnalogSource_ADS_STM32::AnalogSource_ADS_STM32(int16_t pin):
    _pin(pin),
    _value(0.0f)
{
}

bool AnalogSource_ADS_STM32::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return true;
    }
    _pin = pin;
    return true;
}

float AnalogSource_ADS_STM32::read_average()
{
    return read_latest();
}

float AnalogSource_ADS_STM32::read_latest()
{
    return _value;
}

float AnalogSource_ADS_STM32::voltage_average()
{
    return _value;
}

float AnalogSource_ADS_STM32::voltage_latest()
{
    return _value;
}

float AnalogSource_ADS_STM32::voltage_average_ratiometric()
{
    return _value;
}

extern const AP_HAL::HAL &hal;

AnalogIn_ADS_STM32::AnalogIn_ADS_STM32()
{
    _adc = new AP_ADC_STM32_ADC(1);
    _channels_number = _adc->get_channels_number();
}

AP_HAL::AnalogSource* AnalogIn_ADS_STM32::channel(int16_t pin)
{
    WITH_SEMAPHORE(_semaphore);
    for (uint8_t j = 0; j < _channels_number; j++) {
        if (_channels[j] == nullptr) {
            _channels[j] = new AnalogSource_ADS_STM32(pin);
            return _channels[j];
        }
    }

    hal.console->printf("Out of analog channels\n");
    return nullptr;
}

void AnalogIn_ADS_STM32::init()
{
    _adc->init();
    printf("AnalogIn_ADS_STM32 init\n");
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AnalogIn_ADS_STM32::_update, void));
}

void AnalogIn_ADS_STM32::_update()
{
    if (AP_HAL::micros() - _last_update_timestamp < 100000) {
        return;
    }

    adc_report_s2 reports[ADS_STM32_ADC_MAX_CHANNELS];

    size_t rc = _adc->read(reports, ADS_STM32_ADC_MAX_CHANNELS);

    for (size_t i = 0; i < rc; i++) {
        for (uint8_t j=0; j < rc; j++) {
            AnalogSource_ADS_STM32 *source = _channels[j];

            if (source != nullptr && reports[i].id == source->_pin) {
                source->_value = reports[i].data / 1000;
            }
        }
    }

    _last_update_timestamp = AP_HAL::micros();
}
