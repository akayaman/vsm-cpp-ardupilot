// Copyright (c) 2017, Smart Projects Holdings Ltd
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
}

void
Ardupilot_vehicle_manager::Register_detectors()
{
    Transport_detector::Get_instance()->Add_detector(
        ugcs::vsm::Transport_detector::Make_connect_handler(
            &Ardupilot_vehicle_manager::Handle_new_connection,
            Shared_from_this(),
            ugcs::vsm::mavlink::MAV_AUTOPILOT_ARDUPILOTMEGA,
            false,
            ugcs::vsm::Optional<std::string>(),
            ugcs::vsm::Optional<std::string>()),
        Shared_from_this());

    Transport_detector::Get_instance()->Add_detector(
        ugcs::vsm::Transport_detector::Make_connect_handler(
            &Ardupilot_vehicle_manager::Handle_new_injector,
            Shared_from_this()),
        Shared_from_this(),
        "mavlink_injection");

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

void
Ardupilot_vehicle_manager::Handle_new_injector(
    std::string, int, ugcs::vsm::Socket_address::Ptr, ugcs::vsm::Io_stream::Ref stream)
{
    auto mav_stream = Mavlink_vehicle::Mavlink_stream::Create(stream);
    mav_stream->Bind_decoder_demuxer();
    mav_stream->Get_demuxer().
            Register_handler<mavlink::MESSAGE_ID::COMMAND_LONG, mavlink::Extension>(
            Mavlink_demuxer::Make_handler<mavlink::MESSAGE_ID::COMMAND_LONG, mavlink::Extension>(
                    &Ardupilot_vehicle_manager::On_command_long,
                    Shared_from_this()));
    LOG_INFO("MAVlink injection enabled on %s", stream->Get_name().c_str());
    Schedule_injection_read(mav_stream);
}

void
Ardupilot_vehicle_manager::Schedule_injection_read(Mavlink_vehicle::Mavlink_stream::Ptr mav_stream)
{
    auto stream = mav_stream->Get_stream();
    if (stream) {
        if (Get_worker()->Is_enabled()) {
            size_t to_read = mav_stream->Get_decoder().Get_next_read_size();
            size_t max_read;
            if (stream->Get_type() == Io_stream::Type::UDP) {
                max_read = ugcs::vsm::MIN_UDP_PAYLOAD_SIZE_TO_READ;
            } else {
                max_read = to_read;
            }
            injection_readers[mav_stream].Abort();
            injection_readers.emplace(
                mav_stream,
                stream->Read(
                    max_read,
                    to_read,
                    Make_read_callback(
                        [this](Io_buffer::Ptr buffer, Io_result result, Mavlink_vehicle::Mavlink_stream::Ptr mav_stream)
                        {
                            if (result == Io_result::OK) {
                                mav_stream->Get_decoder().Decode(buffer);
                                Schedule_injection_read(mav_stream);
                            } else {
                                if (mav_stream->Get_stream()) {
                                    mav_stream->Get_stream()->Close();
                                }
                                mav_stream->Disable();
                                injection_readers.erase(mav_stream);
                            }
                        },
                        mav_stream),
                    Get_worker()));
        } else {
            mav_stream->Disable();
            stream->Close();
        }
    }
}

void
Ardupilot_vehicle_manager::On_command_long(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::COMMAND_LONG>::Ptr message)
{
    auto it = vehicles.find(message->payload->target_system.Get());
    if (it != vehicles.end()) {
        it->second.vehicle->Send_message(message->payload);
    }
}

void
Ardupilot_vehicle_manager::On_manager_disable()
{
    for (auto& i : injection_readers) {
        i.second.Abort();
        if (i.first->Get_stream()) {
            i.first->Get_stream()->Close();
        }
        i.first->Disable();
    }
    injection_readers.clear();
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
            model_name,
            !id_overridden);
}
