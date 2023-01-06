#include <AP_HAL/AP_HAL.h>
#ifdef BANANA_PI

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DARK || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_OBAL_V1

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "GPIO.h"
#include "Util_RPI.h"

#define GPIO_PA00	0
#define GPIO_PA01	1
#define GPIO_PA02	2
#define GPIO_PA03	3
#define GPIO_PA04	4
#define GPIO_PA05	5
#define GPIO_PA06	6
#define GPIO_PA07	7
#define GPIO_PA08	8
#define GPIO_PA09	9
#define GPIO_PA10	10
#define GPIO_PA11	11
#define GPIO_PA12	12
#define GPIO_PA13	13
#define GPIO_PA14	14
#define GPIO_PA15	15
#define GPIO_PA16	16
#define GPIO_PA17	17
#define GPIO_PA18	18
#define GPIO_PA19	19
#define GPIO_PA20	20
#define GPIO_PA21	21
#define GPIO_PA22	22
#define GPIO_PA23	23
#define GPIO_PA24	24
#define GPIO_PA25	25
#define GPIO_PA26	26
#define GPIO_PA27	27
#define GPIO_PA28	28
#define GPIO_PA29	29
#define GPIO_PA30	30
#define GPIO_PA31	31

#define GPIO_PB00	32
#define GPIO_PB01	1 + GPIO_PB00
#define GPIO_PB02	2 + GPIO_PB00
#define GPIO_PB03	3 + GPIO_PB00
#define GPIO_PB04	4 + GPIO_PB00
#define GPIO_PB05	5 + GPIO_PB00
#define GPIO_PB06	6 + GPIO_PB00
#define GPIO_PB07	7 + GPIO_PB00
#define GPIO_PB08	8 + GPIO_PB00
#define GPIO_PB09	9 + GPIO_PB00
#define GPIO_PB10	10 + GPIO_PB00
#define GPIO_PB11	11 + GPIO_PB00
#define GPIO_PB12	12 + GPIO_PB00
#define GPIO_PB13	13 + GPIO_PB00
#define GPIO_PB14	14 + GPIO_PB00
#define GPIO_PB15	15 + GPIO_PB00
#define GPIO_PB16	16 + GPIO_PB00
#define GPIO_PB17	17 + GPIO_PB00
#define GPIO_PB18	18 + GPIO_PB00
#define GPIO_PB19	19 + GPIO_PB00
#define GPIO_PB20	20 + GPIO_PB00
#define GPIO_PB21	21 + GPIO_PB00
#define GPIO_PB22	22 + GPIO_PB00
#define GPIO_PB23	23 + GPIO_PB00
#define GPIO_PB24	24 + GPIO_PB00
#define GPIO_PB25	25 + GPIO_PB00
#define GPIO_PB26	26 + GPIO_PB00
#define GPIO_PB27	27 + GPIO_PB00
#define GPIO_PB28	28 + GPIO_PB00
#define GPIO_PB29	29 + GPIO_PB00
#define GPIO_PB30	30 + GPIO_PB00
#define GPIO_PB31	31 + GPIO_PB00

#define GPIO_PC00	64
#define GPIO_PC01	1 + GPIO_PC00
#define GPIO_PC02	2 + GPIO_PC00
#define GPIO_PC03	3 + GPIO_PC00
#define GPIO_PC04	4 + GPIO_PC00
#define GPIO_PC05	5 + GPIO_PC00
#define GPIO_PC06	6 + GPIO_PC00
#define GPIO_PC07	7 + GPIO_PC00
#define GPIO_PC08	8 + GPIO_PC00
#define GPIO_PC09	9 + GPIO_PC00
#define GPIO_PC10	10 + GPIO_PC00
#define GPIO_PC11	11 + GPIO_PC00
#define GPIO_PC12	12 + GPIO_PC00
#define GPIO_PC13	13 + GPIO_PC00
#define GPIO_PC14	14 + GPIO_PC00
#define GPIO_PC15	15 + GPIO_PC00
#define GPIO_PC16	16 + GPIO_PC00
#define GPIO_PC17	17 + GPIO_PC00
#define GPIO_PC18	18 + GPIO_PC00
#define GPIO_PC19	19 + GPIO_PC00
#define GPIO_PC20	20 + GPIO_PC00
#define GPIO_PC21	21 + GPIO_PC00
#define GPIO_PC22	22 + GPIO_PC00
#define GPIO_PC23	23 + GPIO_PC00
#define GPIO_PC24	24 + GPIO_PC00
#define GPIO_PC25	25 + GPIO_PC00
#define GPIO_PC26	26 + GPIO_PC00
#define GPIO_PC27	27 + GPIO_PC00
#define GPIO_PC28	28 + GPIO_PC00
#define GPIO_PC29	29 + GPIO_PC00
#define GPIO_PC30	30 + GPIO_PC00
#define GPIO_PC31	31 + GPIO_PC00

#define GPIO_PD00	96
#define GPIO_PD01	1 + GPIO_PD00
#define GPIO_PD02	2 + GPIO_PD00
#define GPIO_PD03	3 + GPIO_PD00
#define GPIO_PD04	4 + GPIO_PD00
#define GPIO_PD05	5 + GPIO_PD00
#define GPIO_PD06	6 + GPIO_PD00
#define GPIO_PD07	7 + GPIO_PD00
#define GPIO_PD08	8 + GPIO_PD00
#define GPIO_PD09	9 + GPIO_PD00
#define GPIO_PD10	10 + GPIO_PD00
#define GPIO_PD11	11 + GPIO_PD00
#define GPIO_PD12	12 + GPIO_PD00
#define GPIO_PD13	13 + GPIO_PD00
#define GPIO_PD14	14 + GPIO_PD00
#define GPIO_PD15	15 + GPIO_PD00
#define GPIO_PD16	16 + GPIO_PD00
#define GPIO_PD17	17 + GPIO_PD00
#define GPIO_PD18	18 + GPIO_PD00
#define GPIO_PD19	19 + GPIO_PD00
#define GPIO_PD20	20 + GPIO_PD00
#define GPIO_PD21	21 + GPIO_PD00
#define GPIO_PD22	22 + GPIO_PD00
#define GPIO_PD23	23 + GPIO_PD00
#define GPIO_PD24	24 + GPIO_PD00
#define GPIO_PD25	25 + GPIO_PD00
#define GPIO_PD26	26 + GPIO_PD00
#define GPIO_PD27	27 + GPIO_PD00
#define GPIO_PD28	28 + GPIO_PD00
#define GPIO_PD29	29 + GPIO_PD00
#define GPIO_PD30	30 + GPIO_PD00
#define GPIO_PD31	31 + GPIO_PD00

#define GPIO_PE00	128
#define GPIO_PE01	1 + GPIO_PE00
#define GPIO_PE02	2 + GPIO_PE00
#define GPIO_PE03	3 + GPIO_PE00
#define GPIO_PE04	4 + GPIO_PE00
#define GPIO_PE05	5 + GPIO_PE00
#define GPIO_PE06	6 + GPIO_PE00
#define GPIO_PE07	7 + GPIO_PE00
#define GPIO_PE08	8 + GPIO_PE00
#define GPIO_PE09	9 + GPIO_PE00
#define GPIO_PE10	10 + GPIO_PE00
#define GPIO_PE11	11 + GPIO_PE00
#define GPIO_PE12	12 + GPIO_PE00
#define GPIO_PE13	13 + GPIO_PE00
#define GPIO_PE14	14 + GPIO_PE00
#define GPIO_PE15	15 + GPIO_PE00
#define GPIO_PE16	16 + GPIO_PE00
#define GPIO_PE17	17 + GPIO_PE00
#define GPIO_PE18	18 + GPIO_PE00
#define GPIO_PE19	19 + GPIO_PE00
#define GPIO_PE20	20 + GPIO_PE00
#define GPIO_PE21	21 + GPIO_PE00
#define GPIO_PE22	22 + GPIO_PE00
#define GPIO_PE23	23 + GPIO_PE00
#define GPIO_PE24	24 + GPIO_PE00
#define GPIO_PE25	25 + GPIO_PE00
#define GPIO_PE26	26 + GPIO_PE00
#define GPIO_PE27	27 + GPIO_PE00
#define GPIO_PE28	28 + GPIO_PE00
#define GPIO_PE29	29 + GPIO_PE00
#define GPIO_PE30	30 + GPIO_PE00
#define GPIO_PE31	31 + GPIO_PE00

#define GPIO_PG00	192
#define GPIO_PG01	1 + GPIO_PG00
#define GPIO_PG02	2 + GPIO_PG00
#define GPIO_PG03	3 + GPIO_PG00
#define GPIO_PG04	4 + GPIO_PG00
#define GPIO_PG05	5 + GPIO_PG00
#define GPIO_PG06	6 + GPIO_PG00
#define GPIO_PG07	7 + GPIO_PG00
#define GPIO_PG08	8 + GPIO_PG00
#define GPIO_PG09	9 + GPIO_PG00
#define GPIO_PG10	10 + GPIO_PG00
#define GPIO_PG11	11 + GPIO_PG00
#define GPIO_PG12	12 + GPIO_PG00
#define GPIO_PG13	13 + GPIO_PG00
#define GPIO_PG14	14 + GPIO_PG00
#define GPIO_PG15	15 + GPIO_PG00
#define GPIO_PG16	16 + GPIO_PG00
#define GPIO_PG17	17 + GPIO_PG00
#define GPIO_PG18	18 + GPIO_PG00
#define GPIO_PG19	19 + GPIO_PG00
#define GPIO_PG20	20 + GPIO_PG00
#define GPIO_PG21	21 + GPIO_PG00
#define GPIO_PG22	22 + GPIO_PG00
#define GPIO_PG23	23 + GPIO_PG00
#define GPIO_PG24	24 + GPIO_PG00
#define GPIO_PG25	25 + GPIO_PG00
#define GPIO_PG26	26 + GPIO_PG00
#define GPIO_PG27	27 + GPIO_PG00
#define GPIO_PG28	28 + GPIO_PG00
#define GPIO_PG29	29 + GPIO_PG00
#define GPIO_PG30	30 + GPIO_PG00
#define GPIO_PG31	31 + GPIO_PG00

#define GPIO_PH00	224
#define GPIO_PH01	1 + GPIO_PH00
#define GPIO_PH02	2 + GPIO_PH00
#define GPIO_PH03	3 + GPIO_PH00
#define GPIO_PH04	4 + GPIO_PH00
#define GPIO_PH05	5 + GPIO_PH00
#define GPIO_PH06	6 + GPIO_PH00
#define GPIO_PH07	7 + GPIO_PH00
#define GPIO_PH08	8 + GPIO_PH00
#define GPIO_PH09	9 + GPIO_PH00
#define GPIO_PH10	10 + GPIO_PH00
#define GPIO_PH11	11 + GPIO_PH00
#define GPIO_PH12	12 + GPIO_PH00
#define GPIO_PH13	13 + GPIO_PH00
#define GPIO_PH14	14 + GPIO_PH00
#define GPIO_PH15	15 + GPIO_PH00
#define GPIO_PH16	16 + GPIO_PH00
#define GPIO_PH17	17 + GPIO_PH00
#define GPIO_PH18	18 + GPIO_PH00
#define GPIO_PH19	19 + GPIO_PH00
#define GPIO_PH20	20 + GPIO_PH00
#define GPIO_PH21	21 + GPIO_PH00
#define GPIO_PH22	22 + GPIO_PH00
#define GPIO_PH23	23 + GPIO_PH00
#define GPIO_PH24	24 + GPIO_PH00
#define GPIO_PH25	25 + GPIO_PH00
#define GPIO_PH26	26 + GPIO_PH00
#define GPIO_PH27	27 + GPIO_PH00
#define GPIO_PH28	28 + GPIO_PH00
#define GPIO_PH29	29 + GPIO_PH00
#define GPIO_PH30	30 + GPIO_PH00
#define GPIO_PH31	31 + GPIO_PH00

#define GPIO_PI00	256
#define GPIO_PI01	1 + GPIO_PI00
#define GPIO_PI02	2 + GPIO_PI00
#define GPIO_PI03	3 + GPIO_PI00
#define GPIO_PI04	4 + GPIO_PI00
#define GPIO_PI05	5 + GPIO_PI00
#define GPIO_PI06	6 + GPIO_PI00
#define GPIO_PI07	7 + GPIO_PI00
#define GPIO_PI08	8 + GPIO_PI00
#define GPIO_PI09	9 + GPIO_PI00
#define GPIO_PI10	10 + GPIO_PI00
#define GPIO_PI11	11 + GPIO_PI00
#define GPIO_PI12	12 + GPIO_PI00
#define GPIO_PI13	13 + GPIO_PI00
#define GPIO_PI14	14 + GPIO_PI00
#define GPIO_PI15	15 + GPIO_PI00
#define GPIO_PI16	16 + GPIO_PI00
#define GPIO_PI17	17 + GPIO_PI00
#define GPIO_PI18	18 + GPIO_PI00
#define GPIO_PI19	19 + GPIO_PI00
#define GPIO_PI20	20 + GPIO_PI00
#define GPIO_PI21	21 + GPIO_PI00
#define GPIO_PI22	22 + GPIO_PI00
#define GPIO_PI23	23 + GPIO_PI00
#define GPIO_PI24	24 + GPIO_PI00
#define GPIO_PI25	25 + GPIO_PI00
#define GPIO_PI26	26 + GPIO_PI00
#define GPIO_PI27	27 + GPIO_PI00
#define GPIO_PI28	28 + GPIO_PI00
#define GPIO_PI29	29 + GPIO_PI00
#define GPIO_PI30	30 + GPIO_PI00
#define GPIO_PI31	31 + GPIO_PI00

#define GPIO_PL00	352
#define GPIO_PL01	1 + GPIO_PL00
#define GPIO_PL02	2 + GPIO_PL00
#define GPIO_PL03	3 + GPIO_PL00
#define GPIO_PL04	4 + GPIO_PL00
#define GPIO_PL05	5 + GPIO_PL00
#define GPIO_PL06	6 + GPIO_PL00
#define GPIO_PL07	7 + GPIO_PL00
#define GPIO_PL08	8 + GPIO_PL00
#define GPIO_PL09	9 + GPIO_PL00
#define GPIO_PL10	10 + GPIO_PL00
#define GPIO_PL11	11 + GPIO_PL00
#define GPIO_PL12	12 + GPIO_PL00
#define GPIO_PL13	13 + GPIO_PL00
#define GPIO_PL14	14 + GPIO_PL00
#define GPIO_PL15	15 + GPIO_PL00
#define GPIO_PL16	16 + GPIO_PL00
#define GPIO_PL17	17 + GPIO_PL00
#define GPIO_PL18	18 + GPIO_PL00
#define GPIO_PL19	19 + GPIO_PL00
#define GPIO_PL20	20 + GPIO_PL00
#define GPIO_PL21	21 + GPIO_PL00
#define GPIO_PL22	22 + GPIO_PL00
#define GPIO_PL23	23 + GPIO_PL00
#define GPIO_PL24	24 + GPIO_PL00
#define GPIO_PL25	25 + GPIO_PL00
#define GPIO_PL26	26 + GPIO_PL00
#define GPIO_PL27	27 + GPIO_PL00
#define GPIO_PL28	28 + GPIO_PL00
#define GPIO_PL29	29 + GPIO_PL00
#define GPIO_PL30	30 + GPIO_PL00
#define GPIO_PL31	31 + GPIO_PL00

#define GPIO_PM00	384
#define GPIO_PM01	1 + GPIO_PM00
#define GPIO_PM02	2 + GPIO_PM00
#define GPIO_PM03	3 + GPIO_PM00
#define GPIO_PM04	4 + GPIO_PM00
#define GPIO_PM05	5 + GPIO_PM00
#define GPIO_PM06	6 + GPIO_PM00
#define GPIO_PM07	7 + GPIO_PM00
#define GPIO_PM08	8 + GPIO_PM00
#define GPIO_PM09	9 + GPIO_PM00
#define GPIO_PM10	10 + GPIO_PM00
#define GPIO_PM11	11 + GPIO_PM00
#define GPIO_PM12	12 + GPIO_PM00
#define GPIO_PM13	13 + GPIO_PM00
#define GPIO_PM14	14 + GPIO_PM00
#define GPIO_PM15	15 + GPIO_PM00
#define GPIO_PM16	16 + GPIO_PM00
#define GPIO_PM17	17 + GPIO_PM00
#define GPIO_PM18	18 + GPIO_PM00
#define GPIO_PM19	19 + GPIO_PM00
#define GPIO_PM20	20 + GPIO_PM00
#define GPIO_PM21	21 + GPIO_PM00
#define GPIO_PM22	22 + GPIO_PM00
#define GPIO_PM23	23 + GPIO_PM00
#define GPIO_PM24	24 + GPIO_PM00
#define GPIO_PM25	25 + GPIO_PM00
#define GPIO_PM26	26 + GPIO_PM00
#define GPIO_PM27	27 + GPIO_PM00
#define GPIO_PM28	28 + GPIO_PM00
#define GPIO_PM29	29 + GPIO_PM00
#define GPIO_PM30	30 + GPIO_PM00
#define GPIO_PM31	31 + GPIO_PM00

#define	BPIPIN_01	-1
#define	BPIPIN_03	GPIO_PA12
#define	BPIPIN_05	GPIO_PA11
#define	BPIPIN_07	GPIO_PA06
#define	BPIPIN_09	-1
#define	BPIPIN_11	GPIO_PA01
#define	BPIPIN_13	GPIO_PA00
#define	BPIPIN_15	GPIO_PA03
#define	BPIPIN_17	-1
#define	BPIPIN_19	GPIO_PC00
#define	BPIPIN_21	GPIO_PC01
#define	BPIPIN_23	GPIO_PC02
#define	BPIPIN_25	-1
#define	BPIPIN_27	GPIO_PA19
#define	BPIPIN_29	GPIO_PA07
#define	BPIPIN_31	GPIO_PA08
#define	BPIPIN_33	GPIO_PA09
#define	BPIPIN_35	GPIO_PA10
#define	BPIPIN_37	GPIO_PA17
#define	BPIPIN_39	-1

#define	BPIPIN_02	-1
#define	BPIPIN_04	-1
#define	BPIPIN_06	-1
#define	BPIPIN_08	GPIO_PA13
#define	BPIPIN_10	GPIO_PA14
#define	BPIPIN_12	GPIO_PA16
#define	BPIPIN_14	-1
#define	BPIPIN_16	GPIO_PA15
#define	BPIPIN_18	GPIO_PC04
#define	BPIPIN_20	-1
#define	BPIPIN_22	GPIO_PA02
#define	BPIPIN_24	GPIO_PC03
#define	BPIPIN_26	GPIO_PC07
#define	BPIPIN_28	GPIO_PA18
#define	BPIPIN_30	-1
#define	BPIPIN_32	GPIO_PL02
#define	BPIPIN_34	-1
#define	BPIPIN_36	GPIO_PL04
#define	BPIPIN_38	GPIO_PA21
#define	BPIPIN_40	GPIO_PA20

#define SUNXI_GPIO_BASE       (0x01c20800)
#define SUNXI_GPIO_LM_BASE    (0x01f02c00)
#define GPIO_BASE_LM_BP		(0x01f02000)   
#define GPIO_BASE_BP        (0x01C20000)

// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices
#define	PI_GPIO_MASK	(0xFFFFFFC0)


#define MTK_GPIO_DIR 0x00
#define MTK_GPIO_PULLE 0x150
#define MTK_GPIO_DOUT 0x500
#define MTK_GPIO_DIN 0x630
#define MTK_GPIO_MODE 0x760
#define BLOCK_SIZE 4096
#define MAP_MASK (BLOCK_SIZE - 1)


// Pin modes

#define	BPI_INPUT			    0
#define	BPI_OUTPUT			    1
#define	BPI_PWM_OUTPUT          2
#define	BPI_GPIO_CLOCK		    3
#define	BPI_SOFT_PWM_OUTPUT		4
#define	BPI_SOFT_TONE_OUTPUT	5
#define	BPI_PWM_TONE_OUTPUT		6

#define	LOW			        0
#define	HIGH			    1


//map bcm gpio_num(index) to bp gpio_num(element)
const int pinTobcm_BP [64] =
{
  BPIPIN_27, BPIPIN_28,  //0, 1
  BPIPIN_03, BPIPIN_05,  //2, 3
  BPIPIN_07, BPIPIN_29,  //4, 5
  BPIPIN_31, BPIPIN_26,  //6, 7
  BPIPIN_24, BPIPIN_21,  //8, 9
  BPIPIN_19, BPIPIN_23,  //10, 11
  BPIPIN_32, BPIPIN_33,  //12, 13
  BPIPIN_08, BPIPIN_10,  //14, 15
  BPIPIN_36, BPIPIN_11,  //16, 17
  BPIPIN_12, BPIPIN_35,	 //18, 19
  BPIPIN_38, BPIPIN_40,  //20, 21
  BPIPIN_15, BPIPIN_16,  //22, 23
  BPIPIN_18, BPIPIN_22,  //24, 25
  BPIPIN_37, BPIPIN_13,  //26, 27
  -1, -1,
  -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 63
} ;



#define GPIO_BananaPI_MAX_NUMBER_PINS 32

using namespace Linux;

extern const AP_HAL::HAL& hal;

// Range based in the first memory address of the first register and the last memory addres
// for the GPIO section (0x7E20'00B4 - 0x7E20'0000).
const uint8_t GPIO_BananaPI::_gpio_registers_memory_range = 0xB4;
const char* GPIO_BananaPI::_system_memory_device_path = "/dev/mem";

GPIO_BananaPI::GPIO_BananaPI()
{
}

void GPIO_BananaPI::set_gpio_mode_alt(int pin, int alternative)
{
    // Each register can contain 10 pins
    const uint8_t pins_per_register = 10;
    // Calculates the position of the 3 bit mask in the 32 bits register
    const uint8_t tree_bits_position_in_register = (pin%pins_per_register)*3;
    /** Creates a mask to enable the alternative function based in the following logic:
     *
     * | Alternative Function | 3 bits value |
     * |:--------------------:|:------------:|
     * |      Function 0      |     0b100    |
     * |      Function 1      |     0b101    |
     * |      Function 2      |     0b110    |
     * |      Function 3      |     0b111    |
     * |      Function 4      |     0b011    |
     * |      Function 5      |     0b010    |
     */
    const uint8_t alternative_value =
        (alternative < 4 ? (alternative + 4) : (alternative == 4 ? 3 : 2));
    // 0b00'000'000'000'000'000'000'ALT'000'000'000 enables alternative for the 4th pin
    const uint32_t mask_with_alt = static_cast<uint32_t>(alternative_value) << tree_bits_position_in_register;
    const uint32_t mask = 0b111 << tree_bits_position_in_register;
    // Clear all bits in our position and apply our mask with alt values
    uint32_t register_value = _gpio[pin / pins_per_register];
    register_value &= ~mask;
    _gpio[pin / pins_per_register] = register_value | mask_with_alt;
}

void GPIO_BananaPI::set_gpio_mode_in(int pin)
{
    // Each register can contain 10 pins
    const uint8_t pins_per_register = 10;
    // Calculates the position of the 3 bit mask in the 32 bits register
    const uint8_t tree_bits_position_in_register = (pin%pins_per_register)*3;
    // Create a mask that only removes the bits in this specific GPIO pin, E.g:
    // 0b11'111'111'111'111'111'111'000'111'111'111 for the 4th pin
    const uint32_t mask = ~(0b111<<tree_bits_position_in_register);
    // Apply mask
    _gpio[pin / pins_per_register] &= mask;
}

void GPIO_BananaPI::set_gpio_mode_out(int pin)
{
    // Each register can contain 10 pins
    const uint8_t pins_per_register = 10;
    // Calculates the position of the 3 bit mask in the 32 bits register
    const uint8_t tree_bits_position_in_register = (pin%pins_per_register)*3;
    // Create a mask to enable the bit that sets output functionality
    // 0b00'000'000'000'000'000'000'001'000'000'000 enables output for the 4th pin
    const uint32_t mask_with_bit = 0b001 << tree_bits_position_in_register;
    const uint32_t mask = 0b111 << tree_bits_position_in_register;
    // Clear all bits in our position and apply our mask with alt values
    uint32_t register_value = _gpio[pin / pins_per_register];
    register_value &= ~mask;
    _gpio[pin / pins_per_register] = register_value | mask_with_bit;
}

void GPIO_BananaPI::set_gpio_high(int pin)
{
    // Calculate index of the array for the register GPSET0 (0x7E20'001C)
    constexpr uint32_t gpset0_memory_offset_value = 0x1c;
    constexpr uint32_t gpset0_index_value = gpset0_memory_offset_value / sizeof(*_gpio);
    _gpio[gpset0_index_value] = 1 << pin;
}

void GPIO_BananaPI::set_gpio_low(int pin)
{
    // Calculate index of the array for the register GPCLR0 (0x7E20'0028)
    constexpr uint32_t gpclr0_memory_offset_value = 0x28;
    constexpr uint32_t gpclr0_index_value = gpclr0_memory_offset_value / sizeof(*_gpio);
    _gpio[gpclr0_index_value] = 1 << pin;
}

bool GPIO_BananaPI::get_gpio_logic_state(int pin)
{
    // Calculate index of the array for the register GPLEV0 (0x7E20'0034)
    constexpr uint32_t gplev0_memory_offset_value = 0x34;
    constexpr uint32_t gplev0_index_value = gplev0_memory_offset_value / sizeof(*_gpio);
    return _gpio[gplev0_index_value] & (1 << pin);
}



volatile uint32_t* GPIO_BananaPI::get_memory_pointer(uint32_t address, uint32_t range) const
{
    auto pointer = mmap(
        nullptr,                         // Any adddress in our space will do
        range,                           // Map length
        PROT_READ|PROT_WRITE|PROT_EXEC,  // Enable reading & writing to mapped memory
        MAP_SHARED|MAP_LOCKED,           // Shared with other processes
        _system_memory_device,           // File to map
        address                          // Offset to GPIO peripheral
    );

    if (pointer == MAP_FAILED) {
        return nullptr;
    }

    return static_cast<volatile uint32_t*>(pointer);
}

bool GPIO_BananaPI::openMemoryDevice()
{
    _system_memory_device = open(_system_memory_device_path, O_RDWR|O_SYNC|O_CLOEXEC);
    if (_system_memory_device < 0) {
        AP_HAL::panic("Can't open %s", GPIO_BananaPI::_system_memory_device_path);
        return false;
    }

    return true;
}

void GPIO_BananaPI::closeMemoryDevice()
{
    close(_system_memory_device);
    // Invalidate device variable
    _system_memory_device = -1;
}

void GPIO_BananaPI::init()
{
    
    if (_initialized) return ;

    _initialized = true ;


    if (!openMemoryDevice()) {
        AP_HAL::panic("Failed to initialize memory device.");
        return;
    }

    printf ("wiringPi: wiringPiSetup called\n") ;

  
    //gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE_BP);
    //_gpio_lm = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE_LM_BP);
    _gpio_lm = get_memory_pointer(GPIO_BASE_LM_BP, BLOCK_SIZE);
    _gpio = get_memory_pointer(GPIO_BASE_BP, BLOCK_SIZE);
    if (!_gpio) {
        AP_HAL::panic("Failed to get GPIO memory map.");
    }

    // No need to keep mem_fd open after mmap
    closeMemoryDevice();
}


int GPIO_BananaPI::get_pin (const int pin) const 
{
  if ((pin & PI_GPIO_MASK) == 0)    // On-board pin
  {
    int bpi_pin= pinTobcm_BP[pin];//need map A20 to bcm
    
    if (-1 == bpi_pin)  /*VCC or GND return directly*/
    {
      //printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
      return -1;
    }
    // softPwmStop (origPin) ;
    // softToneStop (origPin) ;
    return bpi_pin;
  }
  return -1;
}


void GPIO_BananaPI::sunxi_set_pin_alt(int pin, int mode)
{
  uint32_t regval = 0;
  const int bank = pin >> 5;
  const int index = pin - (bank << 5); 
  const int offset = ((index - ((index >> 3) << 3)) << 2);
  uint32_t phyaddr=0;
                         
  /* for M2 PM and PL */
  if(bank == 11)
  {
    phyaddr = SUNXI_GPIO_LM_BASE + ((bank - 11) * 36) + ((index >> 3) << 2);
  }
  else
  {
    phyaddr = SUNXI_GPIO_BASE + (bank * 36) + ((index >> 3) << 2);
  }
    //printf("func:%s pin:%d, MODE:%d bank:%d index:%d phyaddr:0x%x\n",__func__, pin , mode,bank,index,phyaddr);
  regval = sunxi_gpio_readl(phyaddr, bank);
    //printf("read reg val: 0x%x offset:%d\n",regval,offset);
  regval &= ~(7 << offset);
  regval |=  ((mode & 0x7) << offset);
    //printf("Out mode ready set val: 0x%x\n",regval);
  sunxi_gpio_writel(regval, phyaddr, bank);
  //regval = sunxi_gpio_readl(phyaddr, bank);
    //printf("Out mode set over reg val: 0x%x\n",regval);
  return;
}


void GPIO_BananaPI::sunxi_digitalWrite(int pin, int value)
{ 
  uint32_t regval = 0;
  const int bank = pin >> 5;
  const int index = pin - (bank << 5);
  uint32_t phyaddr=0;

  /* for M2 PM and PL */
  if(bank == 11)
    phyaddr = SUNXI_GPIO_LM_BASE + ((bank - 11) * 36) + 0x10;
  else
     phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x10;

    //printf("func:%s pin:%d, value:%d bank:%d index:%d phyaddr:0x%x\n",__func__, pin , value,bank,index,phyaddr);

//  if(BP_PIN_MASK[bank][index] != -1)
  if(1)
  {
    regval = sunxi_gpio_readl(phyaddr, bank);
	
      //printf("befor write reg val: 0x%x,index:%d\n",regval,index);

    if(0 == value)
    {
      regval &= ~(1 << index);
      sunxi_gpio_writel(regval, phyaddr, bank);
      //regval = sunxi_gpio_readl(phyaddr, bank);
	  
        //printf("LOW val set over reg val: 0x%x\n",regval);
    }
    else
    {
      regval |= (1 << index);
      sunxi_gpio_writel(regval, phyaddr, bank);
      //regval = sunxi_gpio_readl(phyaddr, bank);
	  
        //printf("HIGH val set over reg val: 0x%x\n",regval);
    }
  }
  else
  {
    //printf("line:__%d___ %d pin (%d:%d) number error\n",__LINE__,pin,bank,index);
  }
	 
	 return ;
}

uint32_t GPIO_BananaPI::sunxi_gpio_readl(uint32_t addr, int bank)
{
  uint32_t val = 0;
  uint32_t mmap_base = (addr & ~MAP_MASK);
  uint32_t mmap_seek = ((addr - mmap_base) >> 2);

  /* DK, for PL and PM */
  if(bank == 11) {
      val = *(_gpio_lm+ mmap_seek);
  }
  else {
      val = *(_gpio + mmap_seek);
  }

  return val;
}

void GPIO_BananaPI::sunxi_gpio_writel(uint32_t val, uint32_t addr, int bank)
{
  uint32_t mmap_base = (addr & ~MAP_MASK);
  uint32_t mmap_seek = ((addr - mmap_base) >> 2);

  if(bank == 11)
      *(_gpio_lm+ mmap_seek) = val;
  else
      *(_gpio + mmap_seek) = val;
}

void GPIO_BananaPI::pinMode(uint8_t pin, uint8_t output)
{
    const int bpi_pin = get_pin(pin);

    if (output == HAL_GPIO_INPUT) {
        sunxi_set_pin_alt(bpi_pin,BPI_INPUT);
    } else if (output == HAL_GPIO_OUTPUT) {
        sunxi_set_pin_alt(bpi_pin,BPI_OUTPUT);
    }
    return;
}

void GPIO_BananaPI::pinMode(uint8_t pin, uint8_t output, uint8_t alt)
{
    pinMode(pin, output);
}

uint8_t GPIO_BananaPI::read(uint8_t pin)
{
    if (pin >= GPIO_BananaPI_MAX_NUMBER_PINS) {
        return 0;
    }
    return static_cast<uint8_t>(get_gpio_logic_state(pin));
}

void GPIO_BananaPI::write(uint8_t pin, uint8_t value)
{
    if (pin >= GPIO_BananaPI_MAX_NUMBER_PINS) {
        return ;
    }
    
    const int bpi_pin = get_pin(pin);

    uint32_t regval = 0;
  int bank = bpi_pin >> 5;
  int index = bpi_pin - (bank << 5);
  uint32_t phyaddr=0;

  /* for M2 PM and PL */
  if(bank == 11)
    phyaddr = SUNXI_GPIO_LM_BASE + ((bank - 11) * 36) + 0x10;
  else
     phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x10;

    //printf("func:%s pin:%d, value:%d bank:%d index:%d phyaddr:0x%x\n",__func__, pin , value,bank,index,phyaddr);

//  if(BP_PIN_MASK[bank][index] != -1)
  if(1) {
    regval = sunxi_gpio_readl(phyaddr, bank);
	
      //printf("befor write reg val: 0x%x,index:%d\n",regval,index);

    if(0 == value){
      regval &= ~(1 << index);
      sunxi_gpio_writel(regval, phyaddr, bank);
      regval = sunxi_gpio_readl(phyaddr, bank);
	  
        //printf("LOW val set over reg val: 0x%x\n",regval);
    }
    else {
      regval |= (1 << index);
      sunxi_gpio_writel(regval, phyaddr, bank);
      regval = sunxi_gpio_readl(phyaddr, bank);
	  
        //printf("HIGH val set over reg val: 0x%x\n",regval);
    }
  }
  else {
    //printf("line:__%d___ %d pin (%d:%d) number error\n",__LINE__,pin,bank,index);
  }
	 
	 return ;
}

void GPIO_BananaPI::toggle(uint8_t pin)
{
    if (pin >= GPIO_BananaPI_MAX_NUMBER_PINS) {
        return ;
    }
    uint32_t flag = (1 << pin);
    _gpio_output_port_status ^= flag;
    write(pin, (_gpio_output_port_status & flag) >> pin);
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO_BananaPI::channel(uint16_t n)
{
    return new DigitalSource(n);
}

bool GPIO_BananaPI::usb_connected(void)
{
    return false;
}

#endif
#endif