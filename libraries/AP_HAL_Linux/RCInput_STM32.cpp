#include "RCInput_STM32.h"

#include <cmath>
#include <dirent.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <utility>

#define STM32_RCINPUT_CHANNEL_COUNT         0xfe
#define STM32_RCINPUT_REGID                 0xff

#define STM32_RCINPUT_REGID_VALUE           0x92


using namespace Linux;

#define PWM_CHAN_COUNT 8

extern const AP_HAL::HAL& hal;

RCInput_STM32::RCInput_STM32(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev) :
    _dev(std::move(dev))
{
    memset (_pwm_values, 0, sizeof(uint16_t)*LINUX_RC_INPUT_NUM_CHANNELS);
}

RCInput_STM32::~RCInput_STM32()
{
    
}

void RCInput_STM32::init()
{
    uint8_t id;

    if (!_dev->read_registers(STM32_RCINPUT_REGID, &id, 1)) {
        return ;
    }
    
    if (id!=STM32_RCINPUT_REGID_VALUE) {
        printf("Invalid STM32 RCINPUT ID %x\n", id);
        return ;    // not STM32_RCOUT
    }

    uint8_t channel_number;
    if (!_dev->read_registers(STM32_RCINPUT_CHANNEL_COUNT, &channel_number, 1)) {
        channel_number = PWM_CHAN_COUNT;
    }

    printf("STM32_RCINPUT found ... channels count %d\n", channel_number);
    
    set_num_channels(channel_number);
    //_channel_values = new uint16_t[_num_channels];
    
    _initialized = true;
}


void RCInput_STM32::_timer_tick()
{

    if (!_initialized) {
        return;
    }

    if (!_dev || !_dev->get_semaphore()->take(10)) {
        return;
    }

    for (int i=0; i<_num_channels;++i)
    {

        uint16_t channel_value;
        uint8_t * value = (uint8_t*)&channel_value;
        if (!_dev->read_registers(i, value, 2)) {
            continue;
        }
        //_process_pwm_pulse(i, 2200 - channel_value, channel_value);
        //_pwm_values[i] = channel_value;
        _pwm_values[i] = channel_value; // range: 700usec ~ 2300usec
        rc_input_count++;
    }
    
    _dev->get_semaphore()->give();
    
}