// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

// Aircraft class implements vehicle reported by ADSB receiver.
// Each newly discovered ICAO code instantiates this object.
// Adsb_vehicle has all ADSB related telemetry fields predefined.
// Derived class only needs to populate them.

#ifndef _ADSB_AIRCRAFT_H_
#define _ADSB_AIRCRAFT_H_

#include <ugcs/vsm/adsb_vehicle.h>
#include <ugcs/vsm/mavlink.h>

class Adsb_aircraft: public Adsb_vehicle
{
    DEFINE_COMMON_CLASS(Adsb_aircraft, Adsb_vehicle)

public:
    Adsb_aircraft(uint32_t icao) : Adsb_vehicle(icao) {}

    // On_message processes ADSB_VEHICLE message and populates the telemetry fields.
    // With Ping receiver it's simple, it reports each aircraft in separate mavlink message.
    void
    On_message(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::ADSB_VEHICLE>::Ptr message);

    // Return seconds passed since last valid message about this vehicle was received.
    std::chrono::seconds
    Time_since_last_update();

private:
    // When was the last data for this vehicle was received
    std::chrono::time_point<std::chrono::steady_clock> last_update_time;
};

#endif /* _ADSB_AIRCRAFT_H_ */
