// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <ardupilot_vehicle_manager.h>
#include <ardupilot_vehicle.h>
#include <ugcs/vsm/transport_detector.h>

using namespace ugcs::vsm;

Ardupilot_vehicle_manager::Ardupilot_vehicle_manager() :
Mavlink_vehicle_manager(
        "ArduPilot",
        "vehicle.ardupilot")
{
    copter_processor = Ardupilot_vehicle::Create(proto::VEHICLE_TYPE_MULTICOPTER);
    vtol_processor = Ardupilot_vehicle::Create(proto::VEHICLE_TYPE_VTOL);
    plane_processor = Ardupilot_vehicle::Create(proto::VEHICLE_TYPE_FIXED_WING);
}

void
Ardupilot_vehicle_manager::Register_detectors()
{
    Transport_detector::Get_instance()->Add_detector(
        Transport_detector::Make_connect_handler(
            &Ardupilot_vehicle_manager::Handle_new_connection,
            Shared_from_this(),
            mavlink::MAV_AUTOPILOT_ARDUPILOTMEGA,
            Optional<std::string>(),
            Optional<std::string>()),
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

    vtol_processor->Enable();
    copter_processor->Enable();
    plane_processor->Enable();
}

void
Ardupilot_vehicle_manager::On_manager_disable()
{
    copter_processor->Disable();
    plane_processor->Disable();
    vtol_processor->Disable();
}

Mavlink_vehicle::Ptr
Ardupilot_vehicle_manager::Create_mavlink_vehicle(
    Mavlink_demuxer::System_id system_id,
    Mavlink_demuxer::Component_id component_id,
    mavlink::MAV_TYPE type,
    Mavlink_stream::Ptr stream,
    Socket_address::Ptr,
    Optional<std::string> mission_dump_path,
    const std::string& serial_number,
    const std::string& model_name,
    Request_processor::Ptr proc,
    Request_completion_context::Ptr comp)
{
    return Ardupilot_vehicle::Create(
        system_id,
        component_id,
        type,
        stream,
        mission_dump_path,
        serial_number,
        model_name,
        proc,
        comp);
}
