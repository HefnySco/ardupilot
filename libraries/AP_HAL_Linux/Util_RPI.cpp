#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_EDGE || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DARK || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_OBAL_V1 

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "Util.h"
#include "Util_RPI.h"

extern const AP_HAL::HAL &hal;

using namespace Linux;

UtilRPI::UtilRPI()
{
    _check_rpi_version_by_rev();
    if (_not_found) _check_rpi_version();
}

int UtilRPI::_check_rpi_version_by_rev()
{
    const unsigned int MAX_SIZE_LINE = 50;
    typedef struct  {
        const char* revision;
        int cpu_code;
    } rpi_revision_table;

    // @see https://elinux.org/RPi_HardwareHistory
    // @see https://github.com/raspberrypi/documentation/blob/develop/documentation/asciidoc/computers/raspberry-pi/revision-codes.adoc
    rpi_revision_table const rpi_revision[]= {
        { revision:"2", cpu_code:1},   // 1 Model B
        { revision:"3", cpu_code:1},   // 1 Model B
        { revision:"4", cpu_code:1},   // 1 Model B
        { revision:"5", cpu_code:1},   // 1 Model B
        { revision:"6", cpu_code:1},   // 1 Model B
        { revision:"7", cpu_code:1},   // 1 Model A
        { revision:"8", cpu_code:1},   // 1 Model A
        { revision:"9", cpu_code:1},   // 1 Model A
        { revision:"10", cpu_code:1},   // 1 Model B+
        { revision:"11", cpu_code:1},   // 1 Model Compute Module 1	
        { revision:"12", cpu_code:1},   // 1 Model A+
        { revision:"13", cpu_code:1},   // 1 Model B+
        { revision:"14", cpu_code:1},   // 1 Model Compute Module 1
        { revision:"15", cpu_code:1},   // 1 Model A+
        
        { revision:"a01040", cpu_code:2},   // 2 Model B	1.0
        { revision:"a01041", cpu_code:2},   // 2 Model B	1.1
        { revision:"a21041", cpu_code:2},   // 2 Model B	1.1
        { revision:"a22042", cpu_code:2},   // 2 Model B (with BCM2837)  1.2	 
        
        { revision:"900092", cpu_code:0},   // Zero 1.2	
        { revision:"900093", cpu_code:0},   // Zero 1.3	
        { revision:"920093", cpu_code:0},   // Zero 1.3	
        { revision:"9000c1", cpu_code:0},   // Zero W 1.1
        
        { revision:"a02082", cpu_code:3},   // 3 Model B
        { revision:"a020a0", cpu_code:3},   // Compute Module 3 (and CM3 Lite)
        { revision:"a22082", cpu_code:3},   // 3 Model B
        { revision:"a32082", cpu_code:3},   // 3 Model B
        { revision:"a020d3", cpu_code:3},   // 3 Model B+
        { revision:"9020e0", cpu_code:3},   // 3 Model A+
        { revision:"a02100", cpu_code:3},   // Compute Module 3+

        { revision:"a03111", cpu_code:4},   // 4 Model B
        { revision:"b03111", cpu_code:4},   // 4 Model B
        { revision:"b03112", cpu_code:4},   // 4 Model B
        { revision:"b03114", cpu_code:4},   // 4 Model B
        { revision:"c03111", cpu_code:4},   // 4 Model B
        { revision:"c03112", cpu_code:4},   // 4 Model B
        { revision:"c03114", cpu_code:4},   // 4 Model B
        { revision:"d03114", cpu_code:4},   // 4 Model B
        
        { revision:"a03140", cpu_code:4},   // Computer Module 4 1GB
        { revision:"b03140", cpu_code:4},   // Computer Module 4 2GB
        { revision:"c03140", cpu_code:4},   // Computer Module 4 4GB
        { revision:"d03140", cpu_code:4},   // Computer Module 4 8GB

        { revision:"902120", cpu_code:2}    // Zero 2 W
    };

    // assume 0 if unknown
    _rpi_version = 0;

    char buffer[MAX_SIZE_LINE] = { 0 };
    
    FILE *f;
    const char *revision_file = "/proc/cpuinfo";
    
    if ((f = fopen(revision_file, "r")) == NULL) {
        printf("Can't open '%s'\n", revision_file);
    }
    else {

        // loop till Revision line        
        while (fgets(buffer, MAX_SIZE_LINE, f) != nullptr) {
            if (strstr(buffer, "Revision") == nullptr) {
                continue;
            }
            fclose(f);
        }   
        
        // extract number in "Revision	: 9000c1"
        char * pch;
        pch = strtok (buffer," \r");
        pch = strtok (NULL, "\r\n");

        // search for a revision number
        const unsigned int count = sizeof(rpi_revision)/ sizeof(rpi_revision_table);
        for (uint32_t i=0;i< count;++i) {
            if (strcmp(rpi_revision[i].revision, pch) == 0) {
                printf ("Revision %s (intern: %d)\n", rpi_revision[i].revision, rpi_revision[i].cpu_code);
                _rpi_version = rpi_revision[i].cpu_code;
                _not_found = false;
                break;
            }
        }
        

    }
    return _rpi_version;
}

int UtilRPI::_check_rpi_version()
{
    const unsigned int MAX_SIZE_LINE = 50;
    char buffer[MAX_SIZE_LINE];
    int hw;

    memset(buffer, 0, MAX_SIZE_LINE);
    FILE *f = fopen("/sys/firmware/devicetree/base/model", "r");
    if (f != nullptr && fgets(buffer, MAX_SIZE_LINE, f) != nullptr) {
        fclose(f);
        
        int ret = strncmp(buffer, "Raspberry Pi Compute Module 4", 28);
        if (ret == 0) {
             _rpi_version = 4; // compute module 4 e.g. Raspberry Pi Compute Module 4 Rev 1.0.
             printf("%s. (intern: %d)\n", buffer, _rpi_version);
             return _rpi_version;
        }
        
        ret = strncmp(buffer, "Raspberry Pi Zero 2", 19);
        if (ret == 0) {
             _rpi_version = 2; // Raspberry PI Zero 2 W e.g. Raspberry Pi Zero 2 Rev 1.0.
             printf("%s. (intern: %d)\n", buffer, _rpi_version);
             return _rpi_version;
        }
        
        ret = sscanf(buffer + 12, "%d", &_rpi_version);
        if (ret != EOF) {
            if (_rpi_version > 3)  {
                _rpi_version = 4;
            } else if (_rpi_version > 2) {
                // Preserving old behavior.
                _rpi_version = 2;
            } else if (_rpi_version == 0) {
                // RPi 1 doesn't have a number there, so sscanf() won't have read anything.
                _rpi_version = 1;
            }

            printf("%s. (intern: %d)\n", buffer, _rpi_version);

            return _rpi_version;
        }
    }

    // Attempting old method if the version couldn't be read with the new one.
    hw = Util::from(hal.util)->get_hw_arm32();

    if (hw == UTIL_HARDWARE_RPI4) {
        printf("Raspberry Pi 4 with BCM2711!\n");
        _rpi_version = 4;
    } else if (hw == UTIL_HARDWARE_RPI2) {
        printf("Raspberry Pi 2/3 with BCM2709!\n");
        _rpi_version = 2;
    } else if (hw == UTIL_HARDWARE_RPI1) {
        printf("Raspberry Pi 1 with BCM2708!\n");
        _rpi_version = 1;
    } else {
        /* defaults to RPi version 2/3 */
        fprintf(stderr, "Could not detect RPi version, defaulting to 2/3\n");
        _rpi_version = 2;
    }
    return _rpi_version;
}

int UtilRPI::get_rpi_version() const
{
    return _rpi_version;
}

#endif
