// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file ardupilot_vehicle_manager.h
 */
#ifndef _ARDUPILOT_VEHICLE_MANAGER_H_
#define _ARDUPILOT_VEHICLE_MANAGER_H_

#include <mavlink_vehicle_manager.h>
#include <ardupilot_vehicle.h>

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
            ugcs::vsm::Mavlink_stream::Ptr stream,
            ugcs::vsm::Socket_address::Ptr,
            ugcs::vsm::Optional<std::string> mission_dump_path,
            const std::string& serial_number,
            const std::string& model_name,
            ugcs::vsm::Request_processor::Ptr proc,
            ugcs::vsm::Request_completion_context::Ptr comp) override;

    virtual void
    Register_detectors() override;

    /** Disable the manager. */
    virtual void
    On_manager_disable() override;

    // Command processors.
    Ardupilot_vehicle::Ptr copter_processor;
    Ardupilot_vehicle::Ptr plane_processor;
    Ardupilot_vehicle::Ptr vtol_processor;
};

#endif /* _ARDUPILOT_VEHICLE_MANAGER_H_ */
