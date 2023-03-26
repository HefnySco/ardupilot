#pragma once

#include <AP_HAL/I2CDevice.h>
#include "RCInput.h"
#include "AP_HAL_Linux.h"

#define STM32_RCINPUT_ADDRESS             0x27


namespace Linux {

class RCInput_STM32 : public RCInput {

public:
    RCInput_STM32(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    void     init() override;
    //uint16_t read(uint8_t ch) override;
    void _timer_tick(void) override;
    
private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    //uint8_t _channel_number;
    bool _initialized = false;
    //uint16_t *_channel_values;
};

}
