#include "RCOutput_STM32.h"

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

#include <AP_HAL/AP_HAL.h>

#include "GPIO.h"
#define STM32_RCOUT_REGID                   0xff
#define STM32_RCOUT_GET_MOTOR_COUNT         0xfc
#define STM32_RCOUT_ENABLE_OUTPUT           0xfa
#define STM32_RCOUT_CHANGE_PWM_PROTOCOL     0xfe
#define STM32_RCOUT_REGID_VALUE             0x91

using namespace Linux;

#define PWM_CHAN_COUNT 8

extern const AP_HAL::HAL& hal;

RCOutput_STM32::RCOutput_STM32(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev) :
    _dev(std::move(dev)),
    _enable_pin(nullptr),
    _frequency(50),
    _channel_number (PWM_CHAN_COUNT)
{

}

RCOutput_STM32::~RCOutput_STM32()
{
    delete [] _pulses_buffer;
}

void RCOutput_STM32::init()
{
    uint8_t id;

    if (!_dev->read_registers(STM32_RCOUT_REGID, &id, 1)) {
        return ;
    }
    
    if (id!=STM32_RCOUT_REGID_VALUE) {
        printf("Invalid STM32 RCOUTPUT ID %x\n", id);
        return ;    // not STM32_RCOUT
    }

    if (!_dev->read_registers(STM32_RCOUT_GET_MOTOR_COUNT, &_channel_number, 1)) {
        _channel_number = PWM_CHAN_COUNT;
    }

    _pulses_buffer = new uint16_t[_channel_number];
    printf("STM32_RCOUT found ... channels count %d\n", _channel_number);
}

void RCOutput_STM32::reset_all_channels()
{
    if (!_dev || !_dev->get_semaphore()->take(10)) {
        return;
    }

    _dev->write_register(STM32_RCOUT_ENABLE_OUTPUT, 0);

    /* Wait for the last pulse to end */
    hal.scheduler->delay(2);

    _dev->get_semaphore()->give();
}

void RCOutput_STM32::set_output_mode(uint32_t mask, enum output_mode mode)
{
    if (!_dev || !_dev->get_semaphore()->take(10)) {
        return;
    }

    _dev->write_register(STM32_RCOUT_CHANGE_PWM_PROTOCOL, mode);

    _dev->get_semaphore()->give();
}

void RCOutput_STM32::set_freq(uint32_t chmask, uint16_t freq_hz)
{

    _frequency = freq_hz;

}

uint16_t RCOutput_STM32::get_freq(uint8_t ch)
{
    return _frequency;
}

void RCOutput_STM32::enable_ch(uint8_t ch)
{

}

void RCOutput_STM32::disable_ch(uint8_t ch)
{
    write(ch, 0);
}

bool RCOutput_STM32::force_safety_on() {
    if (!_dev || !_dev->get_semaphore()->take(10)) {
        return false;
    }
    /* Shutdown before sleeping. */
    _dev->write_register(STM32_RCOUT_ENABLE_OUTPUT, 0);

    _dev->get_semaphore()->give();
    return true;
}

void RCOutput_STM32::force_safety_off() {
    if (!_dev || !_dev->get_semaphore()->take(10)) {
        return;
    }

    /* Restart the device and enable auto-incremented write */
    _dev->write_register(STM32_RCOUT_ENABLE_OUTPUT,1);
    _dev->get_semaphore()->give();

}

void RCOutput_STM32::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= (_channel_number)) {
        return;
    }
    if (_is_gpio_mask & (1U << ch)) {
        return;
    }
    // channel is larger than available channels in STM32
    if (_channel_number<ch) return ;

    write_raw(ch, period_us);
}

void RCOutput_STM32::write_gpio(uint8_t chan, bool active)
{
    if (chan >= (_channel_number)) {
        return;
    }
    _is_gpio_mask |= (1U << chan);
    
    // channel is larger than available channels in STM32
    if (_channel_number<chan) return ;
   
    write_raw(chan, active);
}

void RCOutput_STM32::write_raw(uint8_t ch, uint16_t period_us) {
    /* Common code used by both write() and write_gpio() */
    _pulses_buffer[ch] = period_us;
        
    if (!_corking) {
        _corking = true;
        push();
    }
}

void RCOutput_STM32::cork()
{
    _corking = true;
}

void RCOutput_STM32::push()
{
    if (!_corking) {
        return;
    }
    _corking = false;

    // Calculate the number of channels for this transfer.
    uint8_t max_ch = _channel_number; 
    uint8_t min_ch = 0; //__builtin_ctz(_pending_write_mask);

    if (!_dev || !_dev->get_semaphore()->take_nonblocking()) {
        return;
    }

    uint8_t pwm_values[3];
    for (unsigned ch = min_ch; ch < max_ch; ch++) {
        //if (_channel_number <=ch) continue;
        uint16_t period_us = _pulses_buffer[ch];
        pwm_values[0]= ch*2+1;
        pwm_values[1] = period_us & 0xff;
        pwm_values[2] = period_us >> 8;
        _dev->transfer((uint8_t *)&pwm_values, 3, nullptr, 0);
    }
    _dev->get_semaphore()->give();
}

uint16_t RCOutput_STM32::read(uint8_t ch)
{
    return _pulses_buffer[ch];
}

void RCOutput_STM32::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(0 + i);
    }
}
