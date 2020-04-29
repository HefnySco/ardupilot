#include "RCOutput_MW_I2C.h"

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

#define MW_I2C_CH0                0x00
#define MW_I2C_CH1                0x01
#define MW_I2C_CH2                0x02
#define MW_I2C_CH3                0x03
#define MW_I2C_CH4                0x04
#define MW_I2C_CH5                0x05
#define MW_I2C_CH6                0x06
#define MW_I2C_CH7                0x07
#define MW_I2C_CH8                0x08
#define MW_I2C_CH9                0x09
#define MW_I2C_CH10               0x0a
#define MW_I2C_CH11               0x0b
#define MW_I2C_CH12               0x0c
#define MW_I2C_CH13               0x0d
#define MW_I2C_CH14               0x0e
#define MW_I2C_CH15               0x0f
#define MW_I2C_CH16               0x10
#define MW_I2C_CH17               0x11
#define MW_I2C_RA_ALL_OUTPUT_SAME   0xFA


/*
 * Drift for internal oscillator
 * see: https://github.com/ArduPilot/ardupilot/commit/50459bdca0b5a1adf95
 * and https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/issues/11
 */
#define PCA9685_INTERNAL_CLOCK (1.04f * 25000000.f)
#define PCA9685_EXTERNAL_CLOCK 24576000.f

using namespace Linux;

#define PWM_CHAN_COUNT 8

extern const AP_HAL::HAL& hal;

RCOutput_MW_I2C::RCOutput_MW_I2C(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                   uint8_t channel_offset) :
    _dev(std::move(dev)),
    _enable_pin(nullptr),
    _frequency(50),
    _pulses_buffer(new uint16_t[PWM_CHAN_COUNT - channel_offset]),
    _channel_offset(channel_offset),
    _last_ch(channel_offset)
{
    if (_external_clock)
        _osc_clock = PCA9685_EXTERNAL_CLOCK;
    else
        _osc_clock = PCA9685_INTERNAL_CLOCK;
}

RCOutput_MW_I2C::~RCOutput_MW_I2C()
{
    delete [] _pulses_buffer;
}

void RCOutput_MW_I2C::init()
{
    reset_all_channels();

    /* Set the initial frequency */
    //set_freq(0, 50);

    /* Enable PCA9685 PWM */
    // if (_oe_pin_number != -1) { // MHEFNY
    //     _enable_pin = hal.gpio->channel(_oe_pin_number);
    //     _enable_pin->mode(HAL_GPIO_OUTPUT);
    //     _enable_pin->write(0);
    // }

    
}

void RCOutput_MW_I2C::reset_all_channels()
{
    // if (!_dev || !_dev->get_semaphore()->take(10)) {
    //     return;
    // }

    _dev->get_semaphore()->take_blocking();

    

    uint8_t data[] = {MW_I2C_RA_ALL_OUTPUT_SAME, 0, 0};
    _dev->transfer(data, sizeof(data), nullptr, 0);

    /* Wait for the last pulse to end */
    hal.scheduler->delay(2);

    _dev->get_semaphore()->give();
}

void RCOutput_MW_I2C::set_freq(uint32_t chmask, uint16_t freq_hz)
{

    
}

uint16_t RCOutput_MW_I2C::get_freq(uint8_t ch)
{
    return _frequency;
}

void RCOutput_MW_I2C::enable_ch(uint8_t ch)
{

}

void RCOutput_MW_I2C::disable_ch(uint8_t ch)
{
    write(ch, 0);
}

void RCOutput_MW_I2C::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= (PWM_CHAN_COUNT - _channel_offset)) {
        return;
    }

    _pulses_buffer[ch] = period_us;

    // if (!(ch & 0b01))
    // {
    //     return ; // skip odd ports as we sends in pairs
    // }
    
    if (!_corking) {
        _corking = true;
        push();
    }
    
}

void RCOutput_MW_I2C::cork()
{
    _corking = true;
}

void RCOutput_MW_I2C::push()
{
    if (!_corking) {
        return;
    }
    _corking = false;

    for (_last_ch=0; _last_ch< PWM_CHAN_COUNT; _last_ch+=2)
    {
        // Calculate the number of channels for this transfer.
        if (_last_ch >= PWM_CHAN_COUNT -1) _last_ch = _channel_offset;
        
        uint8_t min_ch =  _last_ch;
        //++_last_ch;
        uint8_t max_ch = _last_ch+1;
        //++_last_ch;
        
        /*
        * scratch buffer size is always for all the channels, but we write only
        * from min_ch to max_ch
        */
        struct PACKED pwm_values {
            uint8_t reg;
            uint8_t data[PWM_CHAN_COUNT * 2];
        } pwm_values;

        size_t payload_size = 1;

        for (unsigned ch = min_ch; ch <= max_ch; ch++) {
            const uint16_t period_us = _pulses_buffer[ch];

            uint8_t *d = &pwm_values.data[(ch - min_ch) * 2];
            *d++ = 0xFF & period_us;
            *d++ = 0xFF & (period_us >> 8);
            payload_size +=2;
        }


        pwm_values.reg = min_ch;

        // if (_dev->get_semaphore()->take_nonblocking())
        // {
        //     return ;
        // }

        _dev->transfer((uint8_t *)&pwm_values, payload_size, nullptr, 0);
    
    }

    //_dev->get_semaphore()->give();

    // _pending_write_mask = 0;
}

uint16_t RCOutput_MW_I2C::read(uint8_t ch)
{
    return _pulses_buffer[ch];
}

void RCOutput_MW_I2C::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(0 + i);
    }
}
