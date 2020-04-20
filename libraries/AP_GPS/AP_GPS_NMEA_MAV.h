/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
//  Mavlink GPS driver which accepts gps NMEA data from an external
//  companion computer
//
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_GPS.h"
#include "GPS_Backend.h"

class AP_GPS_NMEA_MAV : public AP_GPS_Backend {
public:
    AP_GPS_NMEA_MAV(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    bool read() override;

    static bool _detect(struct MAV_detect_state &state, uint8_t data);

    void inject_data(const uint8_t *data, uint16_t len) override;

    const char *name() const override { return "NMEA_MAV"; }

private:
    bool _new_data;
    JitterCorrection jitter{2000};
     /// Coding for the GPS sentences that the parser handles
    enum _sentence_types {      //there are some more than 10 fields in some sentences , thus we have to increase these value.
        _GPS_SENTENCE_RMC = 32,
        _GPS_SENTENCE_GGA = 64,
        _GPS_SENTENCE_VTG = 96,
        _GPS_SENTENCE_HDT = 128,
        _GPS_SENTENCE_OTHER = 0
    };

    /// Update the decode state machine with a new character
    ///
    /// @param	c		The next character in the NMEA input stream
    /// @returns		True if processing the character has resulted in
    ///					an update to the GPS state
    ///
    bool                        _decode(char c);

    /// Parses the @p as a NMEA-style decimal number with
    /// up to 3 decimal digits.
    ///
    /// @returns		The value expressed by the string in @p,
    ///					multiplied by 100.
    ///
    static int32_t _parse_decimal_100(const char *p);

    /// Parses the current term as a NMEA-style degrees + minutes
    /// value with up to four decimal digits.
    ///
    /// This gives a theoretical resolution limit of around 1cm.
    ///
    /// @returns		The value expressed by the string in _term,
    ///					multiplied by 1e7.
    ///
    uint32_t    _parse_degrees();

    /// Processes the current term when it has been deemed to be
    /// complete.
    ///
    /// Each GPS message is broken up into terms separated by commas.
    /// Each term is then processed by this function as it is received.
    ///
    /// @returns		True if completing the term has resulted in
    ///					an update to the GPS state.
    bool                        _term_complete();

    /// return true if we have a new set of NMEA messages
    bool _have_new_message(void);

    uint8_t _parity;                                                    ///< NMEA message checksum accumulator
    bool _is_checksum_term;                                     ///< current term is the checksum
    char _term[15];                                                     ///< buffer for the current term within the current sentence
    uint8_t _sentence_type;                                     ///< the sentence type currently being processed
    uint8_t _term_number;                                       ///< term index within the current sentence
    uint8_t _term_offset;                                       ///< character offset with the term being received
    uint16_t _sentence_length;
    bool _gps_data_good;                                        ///< set when the sentence indicates data is good

    // The result of parsing terms within a message is stored temporarily until
    // the message is completely processed and the checksum validated.
    // This avoids the need to buffer the entire message.
    int32_t _new_time;                                                  ///< time parsed from a term
    int32_t _new_date;                                                  ///< date parsed from a term
    int32_t _new_latitude;                                      ///< latitude parsed from a term
    int32_t _new_longitude;                                     ///< longitude parsed from a term
    int32_t _new_altitude;                                      ///< altitude parsed from a term
    int32_t _new_speed;                                                 ///< speed parsed from a term
    int32_t _new_course;                                        ///< course parsed from a term
    float   _new_gps_yaw;                                        ///< yaw parsed from a term
    uint16_t _new_hdop;                                                 ///< HDOP parsed from a term
    uint8_t _new_satellite_count;                       ///< satellite count parsed from a term
    uint8_t _new_quality_indicator;                                     ///< GPS quality indicator parsed from a term

    uint32_t _last_RMC_ms = 0;
    uint32_t _last_GGA_ms = 0;
    uint32_t _last_VTG_ms = 0;
    uint32_t _last_HDT_ms = 0;

    /// @name	Init strings
    ///			In ::init, an attempt is made to configure the GPS
    ///			unit to send just the messages that we are interested
    ///			in using these strings
    //@{
    static const char _SiRF_init_string[];         ///< init string for SiRF units
    static const char _MTK_init_string[];                  ///< init string for MediaTek units
    static const char _ublox_init_string[];        ///< init string for ublox units
    //@}

    static const char _initialisation_blob[];
};

#ifndef  AP_GPS_NMEA_HEMISPHERE_INIT_STRING 
#define AP_GPS_NMEA_HEMISPHERE_INIT_STRING \
        "$JATT,NMEAHE,0\r\n" /* Prefix of GP on the HDT message */      \
        "$JASC,GPGGA,5\r\n" /* GGA at 5Hz */                            \
        "$JASC,GPRMC,5\r\n" /* RMC at 5Hz */                            \
        "$JASC,GPVTG,5\r\n" /* VTG at 5Hz */                            \
        "$JASC,GPHDT,5\r\n" /* HDT at 5Hz */                            \
        "$JMODE,SBASR,YES\r\n" /* Enable SBAS */
#endif