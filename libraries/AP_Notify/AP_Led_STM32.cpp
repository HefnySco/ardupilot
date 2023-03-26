#include "AP_Led_STM32.h"

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

#include "AP_Notify.h"

#include <AP_HAL/AP_HAL.h>

#define STM32_LEDX_DEBUG 1
#if STM32_LEDX_DEBUG
#include <cstdio>
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#define error(fmt, args ...)
#endif

#define STM32_LEDX_LED_1            0x11
#define STM32_LEDX_LED_2            0x12
#define STM32_LEDX_LED_3            0x13
#define STM32_LEDX_LED_4            0x14
#define STM32_LEDX_LED_5            0x15
#define STM32_LEDX_LED_6            0x16
#define STM32_LEDX_LED_7            0x17

#if defined(HAL_GPIO_LED_ON) 
#if HAL_GPIO_LED_ON==1
#define STM32_LEDX_ON               0xffff
#define STM32_LEDX_OFF              0x0
#else
#define STM32_LEDX_ON               0x0
#define STM32_LEDX_OFF              0xffff
#endif
#endif

#define STM32_LEDX_REGID            0x1F
#define STM32_LEDX_REGID_VALUE      0xFF


#define STM32_LED_DEBUG 1 
#if STM32_LED_DEBUG
#include <cstdio>
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#define error(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

AP_LED_STM32::AP_LED_STM32(uint8_t bus) :_bus(bus)
{
}

AP_LED_STM32::~AP_LED_STM32()
{
}

bool AP_LED_STM32::init(void)
{
    _dev = std::move(hal.i2c_mgr->get_device(_bus, STM32_LED_ADDRESS));

    if (!_dev) {
        debug("AP_LED_STM32: init BAD\n");
        return false;
    }

    uint8_t id;

    if (!_dev->read_registers(STM32_LEDX_REGID, &id, 1)) {
        debug("AP_LED_STM32: init failed - could not read id\n");
        return false;
    }
    
    if (id!=STM32_LEDX_REGID_VALUE) {
        debug("AP_LED_STM32: init failed - bad id\n");
        return false;    // not STM32_LED
    }
    

    debug("\nAP_LED_STM32: init successfully - addr: 0x%x id: 0x%x\n", STM32_LED_ADDRESS, id);
        
    return true;
}


void AP_LED_STM32::checkGPS()
{
    switch (((_gps_state & 0b10000000)==0) && (AP_Notify::flags.gps_status)) {
        case 0:
        {
            if ((_gps_state==0b00000001) || !_dev || !_dev->get_semaphore()->take(10)) {
                return ;
            }
            _gps_state = 0b00000001;
            
            printf("AP_Notify::flags.gps_status0 \n");
        
            _dev->write_register(STM32_LEDX_LED_2,STM32_LEDX_OFF);
            _dev->get_semaphore()->give();
        }
            break;
        case 1:
        {
            // no GPS attached or no lock - be dark
            if ((_gps_state==0b00000010) || !_dev || !_dev->get_semaphore()->take(10)) {
                return ;
            }
             
            _gps_state = 0b00000010;
            
            printf("AP_Notify::flags.gps_status1 \n");
        
            _dev->write_register(STM32_LEDX_LED_2,0x0a);
            _dev->get_semaphore()->give();
        }
            break;

        case 2: // 2d lock
        {
            if ((_gps_state==0b00000100) || !_dev || !_dev->get_semaphore()->take(10)) {
                return ;
            }
            
            _gps_state = 0b00000100;
            
            printf("AP_Notify::flags.gps_status2 \n");
            
            _dev->write_register(STM32_LEDX_LED_2,0x08);
            _dev->get_semaphore()->give();
        }
        
            break;
            
        case 3://  3d lock
        default: // show number of sats 
        {
            printf(" show number of sats \n");
            if (!_dev || !_dev->get_semaphore()->take(10)) {
                return ;
             }
             _dev->write_register(STM32_LEDX_LED_2,STM32_LEDX_ON);
             _dev->get_semaphore()->give();
        }
            break;        
    }
}

void AP_LED_STM32::checkStatus()
{

}

void AP_LED_STM32::update()
{
   // printf("_state %d initialising %d \r\n",_state, AP_Notify::flags.initialising);
    

    if (((_state & 0b00100000)==0) && (AP_Notify::flags.armed)) {
        printf("AP_Notify::flags.armed \n");
        
        if (!_dev || !_dev->get_semaphore()->take(10)) {
            return ;
        }
        
        _state = 0b00100000;
        
        _dev->write_register(STM32_LEDX_LED_1, STM32_LEDX_ON);
        
        _dev->get_semaphore()->give();

    }
    else {
        // initialising
        if (((_state & 0b00000001)==0) && (AP_Notify::flags.initialising)) {
            // blink LEDs A at 8Hz (full cycle) during initialisation
            printf("AP_Notify::flags.initialising \n");
            
            if (!_dev || !_dev->get_semaphore()->take(10)) {
                return ;
            }
            _state = 0b00000001;
            _dev->write_register(STM32_LEDX_LED_1,0x8);
            _dev->write_register(STM32_LEDX_LED_2,0x9);
            
            _dev->get_semaphore()->give();
            return;
        }
        
        else if (((_state & 0b00000010)==0) && (AP_Notify::flags.save_trim || AP_Notify::flags.esc_calibration)) {
        // save trim and ESC calibration
            printf("AP_Notify::flags.esc_calibration \n");
            
            if (!_dev || !_dev->get_semaphore()->take(10)) {
                return ;
            }
            
            _state = 0b00000010;
            _dev->write_register(STM32_LEDX_LED_1,0x17);
            
            _dev->get_semaphore()->give();
            return ;
        }

        else if (((_state & 0b00000100)==0) && (AP_Notify::flags.compass_cal_running ||
        AP_Notify::flags.temp_cal_running)){
        // compass calibration or IMU temperature calibration
            if (!_dev || !_dev->get_semaphore()->take(10)) {
                return ;
            }
            _state = _state | 0b00000100;
            
            printf("AP_Notify::flags.compass_cal_running \n");
        }

        else if (((_state & 0b00001000)==0) && (AP_Notify::events.autotune_complete)){
            printf("AP_Notify::flags.autotune_complete \n");
            if (!_dev || !_dev->get_semaphore()->take(10)) {
                return ;
            }
            _state = _state | 0b00001000;

            _dev->write_register(STM32_LEDX_LED_1,0x27);
            
            _dev->get_semaphore()->give();
        }

        else if (((_state & 0b00010000)!=0) && (AP_Notify::events.autotune_failed)){
            printf("AP_Notify::flags.esc_calibration \n");
            if (!_dev || !_dev->get_semaphore()->take(10)) {
                return ;
            }
            _state = _state | 0b00010000;

            _dev->write_register(STM32_LEDX_LED_1,0x47);
            
            _dev->get_semaphore()->give();
        }
        
        
        
        else if (((_state & 0b01000000)==0) && (AP_Notify::flags.pre_arm_check)) {
            if (!_dev || !_dev->get_semaphore()->take(10)) {
                return ;
            }
            printf("AP_Notify::flags.esc_calibration \n");
            
            _state = 0b01000000;
            
            _dev->write_register(STM32_LEDX_LED_1,0x17);
        
            _dev->get_semaphore()->give();

            return ;
        }

        else if ((_state!=0) && (!AP_Notify::flags.initialising))
        {
            if (!_dev || !_dev->get_semaphore()->take(10)) {
                return ;
            }
            printf("AP_Notify::OFF \n");
            _state = 0;
            
            _dev->write_register(STM32_LEDX_LED_1,0x27);
            _dev->write_register(STM32_LEDX_LED_2,STM32_LEDX_OFF);

            _dev->get_semaphore()->give();
            return;
        }
    }

    checkGPS();

}
