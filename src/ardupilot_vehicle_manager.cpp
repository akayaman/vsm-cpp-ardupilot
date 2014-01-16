// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <ardupilot_vehicle_manager.h>
#include <ardupilot_vehicle.h>
#include <vsm/transport_detector.h>

using namespace vsm;

Ardupilot_vehicle_manager::Ardupilot_vehicle_manager() :
Mavlink_vehicle_manager(
        "Ardupilot",
        "vehicle.apm",
        mavlink::apm::Extension::Get())
{

}

void
Ardupilot_vehicle_manager::Register_detectors()
{
    /* Bind a detector to all configured serial ports/baudrates. */
    Transport_detector::Get_instance()->Add_detector(
            config_prefix + ".serial_port",
            vsm::Transport_detector::Make_connect_handler(
                    &Ardupilot_vehicle_manager::On_new_connection,
                    Shared_from_this()),
                    Shared_from_this());

    /* Bind a detector to all configured TCP addresses/ports, this is for
     * simulator support.
     */
    Transport_detector::Get_instance()->Add_detector(
            config_prefix + ".tcp",
            vsm::Transport_detector::Make_connect_handler(
                    &Ardupilot_vehicle_manager::On_new_connection,
                    Shared_from_this()),
                    Shared_from_this());
}

Mavlink_vehicle::Ptr
Ardupilot_vehicle_manager::Create_mavlink_vehicle(
        Mavlink_demuxer::System_id system_id,
        Mavlink_demuxer::Component_id component_id,
        mavlink::MAV_TYPE type,
        Io_stream::Ref stream,
        std::string serial_number,
        std::string model_name)
{
    return Ardupilot_vehicle::Create(
            system_id,
            component_id,
            type,
            stream,
            serial_number,
            model_name);
}
