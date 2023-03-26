#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>
#include "AP_ADC_STM32_ADC.h"

#define STM32_ADC_ADDRESS           0x48 // address pin low (GND)
#define STM32_ADC_REGID             0x0f
#define STM32_ADC_REGID_VALUE       0xfe    

#define STM32_ADC_DEBUG 1
#if STM32_ADC_DEBUG
#include <cstdio>
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#define error(fmt, args ...)
#endif

extern const AP_HAL::HAL &hal;

#define STM32_ADC_CHANNELS_COUNT           4

const uint8_t AP_ADC_STM32_ADC::_channels_number  = STM32_ADC_CHANNELS_COUNT;

AP_ADC_STM32_ADC::AP_ADC_STM32_ADC(const uint8_t bus)
    : _dev{}
    , _bus(bus)
    , _channel_to_read(0)
{
    _samples = new adc_report_s2[_channels_number];
}

AP_ADC_STM32_ADC::~AP_ADC_STM32_ADC()
{
    delete[] _samples;
}

bool AP_ADC_STM32_ADC::init()
{
    _dev = hal.i2c_mgr->get_device(_bus, STM32_ADC_ADDRESS);
    if (!_dev) {
        debug("AP_ADC_STM32_ADC: init BAD\n");
        return false;
    }

    _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_ADC_STM32_ADC::_update, void));

    uint8_t id;

    if (!_dev->read_registers(STM32_ADC_REGID, &id, 1)) {
        debug("AP_ADC_STM32_ADC: init failed - could not read id\n");
        return false;
    }
    if (id!=STM32_ADC_REGID_VALUE) {
        debug("AP_ADC_STM32_ADC: init failed - bad id\n");
        return false;    // not STM32_RCOUT
    }

    debug("\nAP_ADC_STM32_ADC: init SUCCEESS - addr: 0x%x id: 0x%x\n", STM32_ADC_ADDRESS, id);
   
    return true;
}


size_t AP_ADC_STM32_ADC::read(adc_report_s2 *report, size_t length) const
{
    for (size_t i = 0; i < length; i++) {
        report[i].data = _samples[i].data;
        report[i].id = _samples[i].id;
    }

    return length;
}


void AP_ADC_STM32_ADC::_update()
{
    be16_t val;

    if (!_dev || !_dev->get_semaphore()->take_nonblocking()) {
        return;
    }
    
    if (!_dev->read_registers(_channel_to_read, (uint8_t *)&val,  sizeof(val))) {
        return;
    }
    hal.scheduler->delay(2);
    
    _dev->get_semaphore()->give();

    float sample = (le16toh(val));
    
    _samples[_channel_to_read].data = sample;
    _samples[_channel_to_read].id = _channel_to_read;

    /* select next channel */
    _channel_to_read = (_channel_to_read + 1) % _channels_number;
}
