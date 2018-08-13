// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <ugcs/vsm/math.h>
#include <adsb_aircraft.h>

using namespace ugcs::vsm;

void
Adsb_aircraft::On_message(
        mavlink::Message<mavlink::MESSAGE_ID::ADSB_VEHICLE>::Ptr message)
{
    auto p = message->payload;

    auto flags = p->flags.Get();

    // no need to switch here as mavlink::ADSB_ALTITUDE_TYPE corresponds 1:1 to proto::Adsb_altitude_source
    t_altitude_type->Set_value(p->altitude_type.Get());

    // no need to switch here as mavlink::ADSB_EMITTER_TYPE corresponds 1:1 to proto::Adsb_emitter_type
    t_emitter_type->Set_value(p->emitter_type.Get());

    if (flags & mavlink::ADSB_FLAGS_VALID_ALTITUDE) {
        t_altitude_amsl->Set_value(static_cast<float>(p->altitude) / 1000);
    }
    if (flags & mavlink::ADSB_FLAGS_VALID_CALLSIGN) {
        t_callsign->Set_value(p->callsign.Get_string());
    }
    if (flags & mavlink::ADSB_FLAGS_VALID_COORDS) {
        t_latitude->Set_value(static_cast<double>(p->lat) / 1e7 / 180.0 * M_PI);
        t_longitude->Set_value(static_cast<double>(p->lon) / 1e7 / 180.0 * M_PI);
    }
    if (flags & mavlink::ADSB_FLAGS_VALID_HEADING) {
        t_heading->Set_value(static_cast<float>(p->heading.Get()) / 100 / 180.0 * M_PI);
    }
    if (flags & mavlink::ADSB_FLAGS_VALID_SQUAWK) {
        t_squawk->Set_value(p->squawk);
    }
    if (flags & mavlink::ADSB_FLAGS_VALID_VELOCITY) {
        t_ground_speed->Set_value(static_cast<float>(p->hor_velocity.Get()) / 100);
        t_vertical_speed->Set_value(static_cast<float>(p->ver_velocity.Get()) / 100);
    }
    Commit_to_ucs();

    last_update_time = std::chrono::steady_clock::now();
}

std::chrono::seconds
Adsb_aircraft::Time_since_last_update()
{
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - last_update_time);
}
