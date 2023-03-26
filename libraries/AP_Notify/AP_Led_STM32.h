#pragma once
#include "AP_Notify_config.h"


#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/I2CDevice.h>

#include "NotifyDevice.h"

#define STM32_LED_ADDRESS             0x33


class AP_LED_STM32 : public NotifyDevice 
{
public:
    AP_LED_STM32(uint8_t bus);

    ~AP_LED_STM32();
    bool  init(void) override;
    void  update() override;

private:
    void  checkGPS();
    void  checkStatus();

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint8_t _bus;
    uint8_t _state = 0;
    uint8_t _gps_state = 0;
};

