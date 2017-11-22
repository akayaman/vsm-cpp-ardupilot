// Copyright (c) 2017, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file ardupilot_vehicle_manager.h
 */
#ifndef _ARDUPILOT_VEHICLE_MANAGER_H_
#define _ARDUPILOT_VEHICLE_MANAGER_H_

#include <mavlink_vehicle_manager.h>

class Ardupilot_vehicle_manager: public Mavlink_vehicle_manager {
    DEFINE_COMMON_CLASS(Ardupilot_vehicle_manager, Mavlink_vehicle_manager)

public:

    /** Constructor. */
    Ardupilot_vehicle_manager();

private:

    virtual Mavlink_vehicle::Ptr
    Create_mavlink_vehicle(
            ugcs::vsm::Mavlink_demuxer::System_id system_id,
            ugcs::vsm::Mavlink_demuxer::Component_id component_id,
            ugcs::vsm::mavlink::MAV_TYPE type,
            ugcs::vsm::Io_stream::Ref stream,
            ugcs::vsm::Socket_address::Ptr,
            ugcs::vsm::Optional<std::string> mission_dump_path,
            std::string serial_number,
            std::string model_name,
            bool id_overridden) override;

    virtual void
    Register_detectors() override;

    /** Disable the manager. */
    virtual void
    On_manager_disable() override;

    void
    Schedule_injection_read(Mavlink_vehicle::Mavlink_stream::Ptr);

    void
    Handle_new_injector(std::string, int, ugcs::vsm::Socket_address::Ptr, ugcs::vsm::Io_stream::Ref);

    void
    On_command_long(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::COMMAND_LONG>::Ptr message);

    std::unordered_map<Mavlink_vehicle::Mavlink_stream::Ptr, ugcs::vsm::Operation_waiter> injection_readers;

};

#endif /* _ARDUPILOT_VEHICLE_MANAGER_H_ */
