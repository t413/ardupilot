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
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_RADAR_ENABLED
#define AP_RADAR_ENABLED 1
#endif

#ifndef HAL_MSP_RADAR_ENABLED
#define HAL_MSP_RADAR_ENABLED (AP_RADAR_ENABLED && HAL_MSP_ENABLED)
#endif

#if AP_RADAR_ENABLED

/*
 * AP_Radar.h - Radar Base Class for ArduPilot
 */

#include <AP_MSP/msp.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/Location.h>

#define RADAR_MAX_PEERS 6
#define RADAR_PEER_FRESH_TIME_MS 3000


struct AP_Radar_Peer {
    uint8_t radar_no;
    uint8_t state;
    Location location;
    uint16_t heading;
    uint16_t speed;
    uint8_t lq;
    uint32_t last_update;
};

class AP_Radar_backend;

class AP_Radar
{
    friend class AP_Radar_backend;

public:
    AP_Radar();

    CLASS_NO_COPY(AP_Radar);

    // get singleton instance
    static AP_Radar *get_singleton() {
        return _singleton;
    }

    enum class Type {
        NONE = 0,
        MSP = 1,
    };

    // init - initialise sensor
    void init();

    // enabled - returns true if radar is enabled
    bool enabled() const { return _type != Type::NONE; }

    // healthy - return true if the sensor is healthy
    bool healthy() const { return backend != nullptr && _flags.healthy; }

    // periodic update processing
    void update(void);

    // handle radar mavlink messages
    void handle_msg(const mavlink_message_t &msg);

    #if HAL_MSP_RADAR_ENABLED
    // handle radar msp messages
    void handle_msp(const MSP::msp_radar_pos_message_t &pkt);
    #endif //HAL_MSP_RADAR_ENABLED

    struct Radar_state {
        AP_Radar_Peer peers[RADAR_MAX_PEERS];   // container for peer statuses
    };

    // return peer at index
    AP_Radar_Peer get_peer(uint8_t id);

    // return true if peer has been updated recently with valid data
    bool get_peer_healthy(uint8_t id);

    // returns the next peer with healthy data
    uint8_t get_next_healthy_peer(uint8_t current_id);

    // parameter var info table
    static const struct AP_Param::GroupInfo var_info[];

private:

    static AP_Radar *_singleton;

    AP_Radar_backend *backend;

    struct AP_Radar_Flags {
        uint8_t healthy     : 1;    // true if sensor is healthy
    } _flags;

    // parameters
    AP_Enum<Type>  _type;           // user configurable sensor type

    // method called by backend to update frontend state:
    void update_state(const Radar_state &state);

    // state filled in by backend
    struct Radar_state _state;

    uint32_t _last_update_ms;        // millis() time of last update

};

namespace AP {
    AP_Radar *radar();
}

#include "AP_Radar_Backend.h"

#endif // AP_RADAR_ENABLED
