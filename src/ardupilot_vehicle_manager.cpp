// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <ardupilot_vehicle_manager.h>
#include <ardupilot_vehicle.h>
#include <ugcs/vsm/transport_detector.h>

using namespace ugcs::vsm;

Ardupilot_vehicle_manager::Ardupilot_vehicle_manager() :
Mavlink_vehicle_manager(
        "ArduPilot",
        "vehicle.ardupilot",
        mavlink::apm::Extension::Get())
{

}

void
Ardupilot_vehicle_manager::Register_detectors()
{
    /* Bind a detector to all configured serial ports/baudrates. */
    Transport_detector::Get_instance()->Add_detector(
            config_prefix + ".serial_port",
            ugcs::vsm::Transport_detector::Make_connect_handler(
                    &Ardupilot_vehicle_manager::Handle_new_connection,
                    Shared_from_this(),
                    ugcs::vsm::Optional<std::string>(),
                    ugcs::vsm::Optional<std::string>()),
                    Shared_from_this());

    /* Bind a detector to all configured TCP addresses/ports, this is for
     * simulator support.
     */
    Transport_detector::Get_instance()->Add_detector(
            config_prefix + ".tcp",
            ugcs::vsm::Transport_detector::Make_connect_handler(
                    &Ardupilot_vehicle_manager::Handle_new_connection,
                    Shared_from_this(),
                    ugcs::vsm::Optional<std::string>(),
                    ugcs::vsm::Optional<std::string>()),
                    Shared_from_this());

    /* Bind a detector to all configured UDP addresses/ports.
     */
    Transport_detector::Get_instance()->Add_detector(
            config_prefix + ".udp",
            ugcs::vsm::Transport_detector::Make_connect_handler(
                    &Ardupilot_vehicle_manager::Handle_new_connection,
                    Shared_from_this(),
                    ugcs::vsm::Optional<std::string>(),
                    ugcs::vsm::Optional<std::string>()),
                    Shared_from_this());

    /* Patterns below are known to appear during Ardupilot booting process. */
    Add_timeout_extension_pattern(regex::regex("Init Ardu"));
    Add_timeout_extension_pattern(regex::regex("Free RAM:"));
    Add_timeout_extension_pattern(regex::regex("start interactive setup"));
    Add_timeout_extension_pattern(regex::regex("Demo Servos"));
    Add_timeout_extension_pattern(regex::regex("Init Gyro"));
    Add_timeout_extension_pattern(regex::regex("Initialising APM"));
    Add_timeout_extension_pattern(regex::regex("barometer calibration"));
    Add_timeout_extension_pattern(regex::regex("Calibrating barometer"));
    Add_timeout_extension_pattern(regex::regex("load_al"));
    Add_timeout_extension_pattern(regex::regex("GROUND START"));
}

Mavlink_vehicle::Ptr
Ardupilot_vehicle_manager::Create_mavlink_vehicle(
        Mavlink_demuxer::System_id system_id,
        Mavlink_demuxer::Component_id component_id,
        mavlink::MAV_TYPE type,
        Io_stream::Ref stream,
        ugcs::vsm::Socket_address::Ptr,
        ugcs::vsm::Optional<std::string> mission_dump_path,
        std::string serial_number,
        std::string model_name,
        bool id_overridden)
{
    if (!id_overridden) {
        switch (Ardupilot_vehicle::Get_type(type)) {
        case Ardupilot_vehicle::Type::COPTER:
            model_name = "ArduCopter";
            break;
        case Ardupilot_vehicle::Type::PLANE:
            model_name = "ArduPlane";
            break;
        case Ardupilot_vehicle::Type::ROVER:
            model_name = "ArduRover";
            break;
        case Ardupilot_vehicle::Type::OTHER:
            model_name = "ArduPilot";
            break;
        }
    }

    return Ardupilot_vehicle::Create(
            system_id,
            component_id,
            type,
            stream,
            mission_dump_path,
            serial_number,
            model_name);
}
