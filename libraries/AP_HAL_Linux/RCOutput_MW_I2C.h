#pragma once

#include <AP_HAL/I2CDevice.h>

#include "AP_HAL_Linux.h"

#define MW_I2C_PRIMARY_ADDRESS              0x40 // All address pins low, PCA9685 default

namespace Linux {

class RCOutput_MW_I2C : public AP_HAL::RCOutput {
public:
    RCOutput_MW_I2C(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                     bool external_clock,
                     uint8_t channel_offset,
                     int16_t oe_pin_number);

    ~RCOutput_MW_I2C();
    void     init() override;
    void     reset_all_channels();
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;
    void     cork() override;
    void     push() override;
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;

private:
    void reset();

    AP_HAL::DigitalSource *_enable_pin;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint16_t _frequency;
    float _osc_clock;

    uint16_t *_pulses_buffer;

    bool _external_clock;
    bool _corking = false;
    uint8_t _channel_offset;
    int16_t _oe_pin_number;
    uint16_t _pending_write_mask;
    uint16_t _last_ch;
};

}
