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
}

int UtilRPI::_check_rpi_version_by_rev()
{
    const unsigned int MAX_SIZE_LINE = 50;
    typedef struct  {
        const char* revision;
        int soc_code;
    } rpi_revision_table;

    // @see https://elinux.org/RPi_HardwareHistory
    // @see https://github.com/raspberrypi/documentation/blob/develop/documentation/asciidoc/computers/raspberry-pi/revision-codes.adoc
    rpi_revision_table const rpi_revision[]= {
        { revision:"2", soc_code:1},   // 1 Model B
        { revision:"3", soc_code:1},   // 1 Model B
        { revision:"4", soc_code:1},   // 1 Model B
        { revision:"5", soc_code:1},   // 1 Model B
        { revision:"6", soc_code:1},   // 1 Model B
        { revision:"7", soc_code:1},   // 1 Model A
        { revision:"8", soc_code:1},   // 1 Model A
        { revision:"9", soc_code:1},   // 1 Model A
        { revision:"10", soc_code:1},   // 1 Model B+
        { revision:"11", soc_code:1},   // 1 Model Compute Module 1	
        { revision:"12", soc_code:1},   // 1 Model A+
        { revision:"13", soc_code:1},   // 1 Model B+
        { revision:"14", soc_code:1},   // 1 Model Compute Module 1
        { revision:"15", soc_code:1},   // 1 Model A+
        
        { revision:"a01040", soc_code:2},   // 2 Model B	1.0
        { revision:"a01041", soc_code:2},   // 2 Model B	1.1
        { revision:"a21041", soc_code:2},   // 2 Model B	1.1
        { revision:"a22042", soc_code:2},   // 2 Model B (with BCM2837)  1.2	 
        
        { revision:"900092", soc_code:1},   // Zero 1.2	
        { revision:"900093", soc_code:1},   // Zero 1.3	
        { revision:"920093", soc_code:1},   // Zero 1.3	
        { revision:"9000c1", soc_code:1},   // Zero W 1.1
        
        { revision:"a02082", soc_code:3},   // 3 Model B
        { revision:"a020a0", soc_code:3},   // Compute Module 3 (and CM3 Lite)
        { revision:"a22082", soc_code:3},   // 3 Model B
        { revision:"a32082", soc_code:3},   // 3 Model B
        { revision:"a020d3", soc_code:3},   // 3 Model B+
        { revision:"9020e0", soc_code:3},   // 3 Model A+
        { revision:"a02100", soc_code:3},   // Compute Module 3+

        { revision:"a03111", soc_code:4},   // 4 Model B
        { revision:"b03111", soc_code:4},   // 4 Model B
        { revision:"b03112", soc_code:4},   // 4 Model B
        { revision:"b03114", soc_code:4},   // 4 Model B
        { revision:"c03111", soc_code:4},   // 4 Model B
        { revision:"c03112", soc_code:4},   // 4 Model B
        { revision:"c03114", soc_code:4},   // 4 Model B
        { revision:"d03114", soc_code:4},   // 4 Model B
        
        { revision:"a03140", soc_code:4},   // Computer Module 4 1GB
        { revision:"b03140", soc_code:4},   // Computer Module 4 2GB
        { revision:"c03140", soc_code:4},   // Computer Module 4 4GB
        { revision:"d03140", soc_code:4},   // Computer Module 4 8GB

        { revision:"902120", soc_code:2}    // Zero 2 W
    };

    // assume 2 if unknown
    _rpi_version = 2;

    char buffer[MAX_SIZE_LINE] = { 0 };
    
    FILE *f;
    const char *revision_file = "/proc/cpuinfo";
    
    if ((f = fopen(revision_file, "r")) == NULL) {
        printf("Can't open '%s'\n", revision_file);
    }
    else {

        bool _revision_found = false;
        // loop till Revision line        
        while (fgets(buffer, MAX_SIZE_LINE, f) != nullptr) {
            if (strstr(buffer, "Revision") != nullptr) {
                _revision_found = true;
                break;
            }
        }   

        fclose(f);
        if (!_revision_found) return _rpi_version;
        
        // extract number in "Revision	: 9000c1"
        char * pch;
        pch = strtok (buffer," \r");
        pch = strtok (NULL, "\r\n");

        // search for a revision number
        const unsigned int count = sizeof(rpi_revision)/ sizeof(rpi_revision_table);
        for (uint32_t i=0;i< count;++i) {
            if (strcmp(rpi_revision[i].revision, pch) == 0) {
                printf ("Revision %s (intern: %d)\n", rpi_revision[i].revision, rpi_revision[i].soc_code);
                _rpi_version = rpi_revision[i].soc_code;
                break;
            }
        }
        

    }
    return _rpi_version;
}


int UtilRPI::get_rpi_version() const
{
    return _rpi_version;
}

#endif
