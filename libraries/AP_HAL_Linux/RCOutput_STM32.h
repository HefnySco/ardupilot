#pragma once

#include <AP_HAL/I2CDevice.h>

#include "AP_HAL_Linux.h"

#define PCA9685_STM32_PRIMARY_ADDRESS             0x66 

namespace Linux {

class RCOutput_STM32 : public AP_HAL::RCOutput {
public:
    RCOutput_STM32(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                     uint32_t external_clock,
                     uint8_t channel_offset,
                     int16_t oe_pin_number);

    ~RCOutput_STM32();
    void     init() override;
    void     reset_all_channels();
    void     set_output_mode(uint32_t mask, enum output_mode mode) override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    bool     force_safety_on() override;
    void     force_safety_off() override;
    void     write(uint8_t ch, uint16_t period_us) override;
    void     cork() override;
    void     push() override;
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;
    bool     supports_gpio() override { return true; };
    void     write_gpio(uint8_t chan, bool active) override;

private:
    void reset();
    void write_raw(uint8_t ch, uint16_t period_us);

    AP_HAL::DigitalSource *_enable_pin;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint16_t _frequency;

    uint16_t *_pulses_buffer;

    uint32_t _external_clock;
    bool _corking = false;
    uint8_t _channel_offset;
    int16_t _oe_pin_number;
    uint32_t _pending_write_mask;
    uint32_t _is_gpio_mask;
};

}
