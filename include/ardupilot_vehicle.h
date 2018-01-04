// Copyright (c) 2017, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file ardupilot_vehicle.h
 */
#ifndef _ARDUPILOT_VEHICLE_H_
#define _ARDUPILOT_VEHICLE_H_

#include <mavlink_vehicle.h>

#define ARDUPILOT_VERSION(maj, min, patch) ((maj << 24) + (min << 16) + (patch << 8))
#define DISARM_MAGIC_VALUE 21196.0f

/** Vehicle supporting Ardupilot specific flavor of Mavlink. */
class Ardupilot_vehicle: public Mavlink_vehicle {
    DEFINE_COMMON_CLASS(Ardupilot_vehicle, Mavlink_vehicle)

public:
    template<typename... Args>
    Ardupilot_vehicle(
            ugcs::vsm::Mavlink_demuxer::System_id system_id,
            ugcs::vsm::Mavlink_demuxer::Component_id component_id,
            ugcs::vsm::mavlink::MAV_TYPE type,
            ugcs::vsm::Io_stream::Ref stream,
            ugcs::vsm::Optional<std::string> mission_dump_path,
            Args &&... args) :
            Mavlink_vehicle(
                    system_id, component_id, type,
                    ugcs::vsm::mavlink::MAV_AUTOPILOT::MAV_AUTOPILOT_ARDUPILOTMEGA,
                    Vehicle::Capabilities(),
                    stream, mission_dump_path, std::forward<Args>(args)...),
            vehicle_command(*this),
            task_upload(*this)
    {
        autopilot_type = "ardupilot";
        /* Consider this as uptime start. */
        recent_connect = std::chrono::steady_clock::now();
        Configure();
        Update_capabilities();
    }

    virtual void
    On_enable();

    virtual void
    On_disable();

    /** Distinguishable type of Ardupilot vehicle. This is mainly driven by
     * Ardupilot firmware flavors each having some minor differences. */
    enum Type {
        /** Copter (quad, octa, hexa etc). */
        COPTER,
        /** Fixed wing plane. */
        PLANE,
        /** Rovers, cars. */
        ROVER,
        /** Other unsupported/unknown vehicle type, depending on the context.*/
        OTHER
    };

    /** UCS has sent a task for a vehicle. */
    virtual void
    Handle_vehicle_request(ugcs::vsm::Vehicle_task_request::Handle request) override;

    /**
     * UCS requesting command execution on a vehicle.
     */
    virtual void
    Handle_vehicle_request(ugcs::vsm::Vehicle_command_request::Handle request) override;

    virtual void
    Handle_ucs_command(ugcs::vsm::Ucs_request::Ptr ucs_request);

    /** Get the Type of the vehicle. */
    Type
    Get_type() const;

    /** Get the Type of the vehicle based on Mavlink type. */
    static Type
    Get_type(ugcs::vsm::mavlink::MAV_TYPE);

    /** Ardupilot specific activity. */
    class Ardupilot_activity : public Activity {
    public:
        /** Constructor based on Ardupilot vehicle class. */
        Ardupilot_activity(Ardupilot_vehicle& ardu_vehicle) :
            Activity(ardu_vehicle),
            ardu_vehicle(ardu_vehicle) {}

        /** Managed Ardupilot vehicle. */
        Ardupilot_vehicle& ardu_vehicle;
    };

    /** Data related to vehicle command processing. */
    class Vehicle_command_act : public Ardupilot_activity {
    public:
        using Ardupilot_activity::Ardupilot_activity;

        /** Try execute command a vehicle. */
        bool
        Try();

        /** Try execute a verifying command for polyfence */
        bool
        Try_verify_polyfence();

        /** Command ack received. */
        void
        On_command_ack(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::COMMAND_ACK>::Ptr);

        void
        On_mission_ack(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::MISSION_ACK>::Ptr);

        void
        On_param_value(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr);

        void
        On_point_value(
            ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::apm::MESSAGE_ID::FENCE_POINT,
            ugcs::vsm::mavlink::apm::Extension>::Ptr);

        void
        On_mission_current(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::MISSION_CURRENT>::Ptr);

        void
        On_param_str_value(
            ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::sph::MESSAGE_ID::PARAM_STR_VALUE,
            ugcs::vsm::mavlink::sph::Extension>::Ptr);

        void
        Send_next_command();

        /** Status text recieved. */
        void
        On_status_text(
                ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::STATUSTEXT>::Ptr);

        /** Enable class and start command execution. */
        void
        Enable();

        /** Disable the activity. */
        void
        Disable(const std::string& status);

        void
        Disable_success();

        /** Schedule timer for retry operation. */
        void
        Schedule_timer();

        /** Schedule timer for verifying operation. */
        void
        Schedule_verify_timer();

        /** Register status text handler. */
        void
        Register_status_text();

        /** Unregister status text handler. */
        void
        Unregister_status_text();

        /** Get the value of custom mode corresponding to AUTO mode of the
         * current vehicle type. */
        uint32_t
        Get_custom_auto_mode();

        /** Get the value of custom mode corresponding to reasonable manual mode
         * of the current vehicle type. */
        uint32_t
        Get_custom_manual_mode();

        /** Get the value of custom mode corresponding to guided mode
         * of the current vehicle type. */
        uint32_t
        Get_custom_guided_mode();

        /** Current command request. */
        ugcs::vsm::Vehicle_command_request::Handle vehicle_command_request;

        /** new style command request. */
        ugcs::vsm::Ucs_request::Ptr ucs_request = nullptr;

        /** Mavlink messages to be sent to execute current command. */
        std::list<ugcs::vsm::mavlink::Payload_base::Ptr> cmd_messages;

        /** Remaining attempts towards vehicle. */
        size_t remaining_attempts = 0;

        /** Retry timer. */
        ugcs::vsm::Timer_processor::Timer::Ptr timer;

        /** Current timeout to use when scheduling timer. */
        std::chrono::milliseconds current_timeout;

        /** check if point is inside polygon.
         *  Algorithm was taken from ardupilot cause we have to check if the returning point
         *  for geofence is outside polygon exactly as ardupilot does.
         *  If returning point is outside polygon fence then fence will not work and there is
         *  no way to know about it from ardupilot
         */
        bool
        Is_Outside_Polygon(double check_point_latitude, double check_point_longitude,
                           ugcs::vsm::proto::List_value lats, ugcs::vsm::proto::List_value lngs);
    } vehicle_command;

    /** Data related to task upload processing. */
    class Task_upload: public Ardupilot_activity {
    public:
        using Ardupilot_activity::Ardupilot_activity;

        /** Calls appropriate prepare action based on type. */
        void
        Prepare_action(ugcs::vsm::Action::Ptr);

        /** Add mission item to prepared actions. Common mission item
         * initialization are made, like sequence number generation.
         */
        void
        Add_mission_item(ugcs::vsm::mavlink::Pld_mission_item::Ptr);

        //@{
        /** Prepare methods for different types of actions. These methods
         * create an item in the prepared actions list.
         * @return Created mission item. */

        void
        Prepare_move(ugcs::vsm::Action::Ptr&);

        void
        Prepare_wait(ugcs::vsm::Action::Ptr&);

        void
        Prepare_payload_steering(ugcs::vsm::Action::Ptr&);

        void
        Prepare_takeoff(ugcs::vsm::Action::Ptr&);

        void
        Prepare_landing(ugcs::vsm::Action::Ptr&);

        void
        Prepare_change_speed(ugcs::vsm::Action::Ptr&);

        void
        Prepare_set_home(ugcs::vsm::Action::Ptr&);

        void
        Prepare_POI(ugcs::vsm::Action::Ptr&);

        void
        Prepare_heading(ugcs::vsm::Action::Ptr&);

        void
        Prepare_panorama(ugcs::vsm::Action::Ptr&);

        void
        Prepare_camera_control(ugcs::vsm::Action::Ptr&);

        void
        Prepare_camera_series_by_distance(ugcs::vsm::Action::Ptr&);

        void
        Prepare_camera_series_by_time(ugcs::vsm::Action::Ptr&);

        void
        Prepare_camera_trigger(ugcs::vsm::Action::Ptr&);

        void
        Prepare_vtol_transition(ugcs::vsm::Action::Ptr&);

        void
        Prepare_set_servo(ugcs::vsm::Action::Ptr&);

        void
        Prepare_repeat_servo(ugcs::vsm::Action::Ptr&);

        //@}

        /** Build waypoint mission item based on move action. */
        ugcs::vsm::mavlink::Pld_mission_item::Ptr
        Build_wp_mission_item(ugcs::vsm::Action::Ptr&);

        /** Build ROI mission item based on given coordinates */
        ugcs::vsm::mavlink::Pld_mission_item::Ptr
        Build_roi_mission_item(const ugcs::vsm::Geodetic_tuple& coords);

        /** Build Heading mission item */
        ugcs::vsm::mavlink::Pld_mission_item::Ptr
        Build_heading_mission_item(
                float heading,
                float speed = 0.0,
                bool absolute_angle = true,
                bool clockwise = true);

        void
        Add_camera_trigger_item();

        /** Previous activity is completed, enable class and start task upload. */
        void
        Enable(ugcs::vsm::Vehicle_task_request::Handle);

        /** Disable this class and cancel any existing request. */
        virtual void
        On_disable() override;

        /** Filter unsupported actions. */
        void
        Filter_actions();

        /** Filter actions unsupported by copters. */
        void
        Filter_copter_actions();

        /** Filter actions unsupported by planes. */
        void
        Filter_plane_actions();

        /** Filter actions unsupported by rovers. */
        void
        Filter_rover_actions();

        /** Filter actions unsupported by other types of vehicles. */
        void
        Filter_other_actions();

        /** Prepare the task for uploading to the vehicle. */
        void
        Prepare_task();

        /** Prepare task attributes depending on the vehicle type. */
        void
        Prepare_task_attributes();

        /** Prepare copter task attributes. */
        void
        Prepare_copter_task_attributes();

        /** Prepare plane task attributes. */
        void
        Prepare_plane_task_attributes();

        /** Prepare rover task attributes. */
        void
        Prepare_rover_task_attributes();

        /** Prepare task attributes of other vehicles. */
        void
        Prepare_other_task_attributes();

        /** Task attributes upload handler. */
        void
        Task_atributes_uploaded(bool success, std::string);

        /** Task attributes upload handler. */
        void
        Upload_parameters(bool success, std::string);

        /** Task attributes upload handler. */
        void
        Task_commands_sent(bool success, std::string = "");

        /** Mission uploaded. */
        void
        Mission_uploaded(bool success, std::string);

        /** Mission downloaded. Calculate the mission_id. */
        void
        Mission_downloaded(bool success, std::string);

        /**
         * Fill coordinates into Mavlink message based on ugcs::vsm::Geodetic_tuple and
         * some other common mission item data structures.
         * @param msg Mavlink message.
         * @param tuple Geodetic tuple.
         * @param heading Vehicle heading.
         */
        void
        Fill_mavlink_mission_item_coords(ugcs::vsm::mavlink::Pld_mission_item& msg,
                const ugcs::vsm::Geodetic_tuple& tuple, double heading);

        /**
         * Fill Mavlink mission item common parameters.
         * @param msg Mavlink message.
         */
        void
        Fill_mavlink_mission_item_common(ugcs::vsm::mavlink::Pld_mission_item& msg);


        /** Current task for uploading, if any. */
        ugcs::vsm::Vehicle_task_request::Handle request;

        /** Prepared Mavlink actions to be uploaded to the vehicle and built based
         * on the actions from the original request. Original actions could be
         * extended/removed/updated to meet the Mavlink mission protocol
         * requirements. Example is adding of magical "dummy waypoints" and
         * special processing of waypoint zero.
         */
        std::vector<ugcs::vsm::mavlink::Payload_base::Ptr> prepared_actions;

        /** Task attributes to be written to the vehicle. */
        Write_parameters::List task_attributes;

        /** Previous move action, if any. */
        ugcs::vsm::Action::Ptr last_move_action;

        /** Active POI from mission. */
        ugcs::vsm::Optional<ugcs::vsm::Geodetic_tuple> current_mission_poi;

        /** Current heading from mission. */
        ugcs::vsm::Optional<float> current_mission_heading;

        /** Does current WP have POI action defined in mission*/
        bool first_mission_poi_set = false;

        /** Mission POI action must be added as it was cancelled by previous actions.*/
        bool restart_mission_poi = false;

        float current_heading = 0.0;

        float heading_to_this_wp = 0.0;

        /** CAMERA_SERIES_BY_DISTANCE was activated. */
        bool camera_series_by_dist_active = false,
        /** CAMERA_SERIES_BY_DISTANCE was activated in current waypoint. */
             camera_series_by_dist_active_in_wp = false,
        /** CAMERA_SERIES_BY_TIME was activated. */
             camera_series_by_time_active = false,
        /** CAMERA_SERIES_BY_DISTANCE was activated in current waypoint. */
             camera_series_by_time_active_in_wp = false;
    } task_upload;


    void
    On_adsb_state(
        ugcs::vsm::mavlink::Message<
            ugcs::vsm::mavlink::sph::SPH_ADSB_TRANSPONDER_STATE,
            ugcs::vsm::mavlink::sph::Extension>::Ptr message);

    void
    On_autopilot_version(ugcs::vsm::mavlink::Pld_autopilot_version ver);

    void
    On_parameter(ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::PARAM_VALUE>::Ptr);

    void
    On_string_parameter(
        ugcs::vsm::mavlink::Message<
            ugcs::vsm::mavlink::sph::MESSAGE_ID::PARAM_STR_VALUE,
            ugcs::vsm::mavlink::sph::Extension>::Ptr message);

    void
    On_mission_item(ugcs::vsm::mavlink::Pld_mission_item);

    void
    Report_home_location(double lat, double lon, float alt);

    void
    On_home_position(
        ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::HOME_POSITION>::Ptr m);


    void
    On_mission_downloaded(bool, const std::string);

    bool
    On_home_location_timer();

private:
    /** Flight modes of the Ardupilot Copter mode. */
    enum class Copter_flight_mode {
        /** Hold level position. */
        STABILIZE = 0,
        /** Rate control. */
        ACRO = 1,
        /** AUTO control. */
        ALT_HOLD = 2,
        /** AUTO control. */
        AUTO = 3,
        /** AUTO control. */
        GUIDED = 4,
        /** Hold a single location. */
        LOITER = 5,
        /** AUTO control. */
        RTL = 6,
        /** AUTO control. */
        CIRCLE = 7,
        /** AUTO control. */
        LAND = 9,
        /** Hold a single location using optical flow sensor. */
        OF_LOITER = 10,
        /** DRIFT mode (Note: 12 is no longer used). */
        DRIFT = 11,
        /** Earth frame rate control. */
        SPORT = 13,
        /** Flip the vehicle on the roll axis. */
        FLIP = 14,
        /** Autotune the vehicle's roll and pitch gains. */
        AUTOTUNE = 15
    };

    // @{
    /** Flight modes of the Ardupilot Plane mode. */
    enum class Plane_flight_mode {
        MANUAL = 0,
        CIRCLE = 1,
        STABILIZE = 2,
        TRAINING = 3,
        ACRO = 4,
        FLY_BY_WIRE_A = 5,
        FLY_BY_WIRE_B = 6,
        CRUISE = 7,
        AUTO = 10,
        RTL = 11,
        LOITER = 12,
        GUIDED = 15,
        INITIALISING = 16
    };
    // @}

    // @{
    /** Flight modes of the Ardupilot Rover mode. */
    enum class Rover_flight_mode {
        MANUAL = 0,
        LEARNING = 2,
        STEERING = 3,
        HOLD = 4,
        AUTO = 10,
        RTL = 11,
        GUIDED = 15,
        INITIALISING = 16
    };
    // @}

    /** Process heartbeat message by setting system status according to it. */
    virtual void
    Process_heartbeat(
            ugcs::vsm::mavlink::Message<ugcs::vsm::mavlink::MESSAGE_ID::HEARTBEAT>::Ptr) override;

    /** Map custom flight mode from the heartbeat to our flight mode. */
    void
    Update_flight_mode(int);

    const char*
    Get_native_flight_mode_name(int);

    Sys_status::Control_mode
    Map_control_mode(int);

    /** Updates current capabilities based on vehicle type. */
    void
    Update_capabilities();

    /** Updates current capability states based on vehicle type. */
    void
    Update_capability_states();

    /** Load parameters from configuration. */
    void
    Configure();

    void
    Get_home_location();

    void
    Download_mission();

    bool
    Is_home_position_valid();

    void
    Start_rc_override();

    void
    Send_rc_override();

    bool
    Send_rc_override_timer();

    bool
    Is_rc_override_active();

    void
    Stop_rc_override();

    void
    Set_rc_override(int p, int r, int t, int y);

    /**
     * Minimal waypoint acceptance radius to use.
     */
    constexpr static double ACCEPTANCE_RADIUS_MIN = 1;

    /** Recent connect time of the vehicle. */
    std::chrono::steady_clock::time_point recent_connect;

    /** Index of servo to use for camera trigger. */
    int camera_servo_idx;
    /** PWM value to set for camera trigger. */
    int camera_servo_pwm;
    /** Time to hold camera servo at the specified PWM when triggering. */
    float camera_servo_time;

    /** By default joystick mode is disabled for planes.
     * Turn on via an entry in conf file. */
    bool enable_joystick_control_for_fixed_wing = false;

    /** if true, then enable GND_ALT_OFFSET calibration during mission upload
     *  if false, set GND_ALT_OFFSET to zero */
    bool set_ground_alt_offset = true;

    /** by default autoheading is turned on */
    bool autoheading = true;

    /** If vehicle does not support ROI for multiple WPts then VSM must
     * generate POI commands for each WP until POI(none) received.
     * Leave it true for now until VSM is able to detect Ardupilot FW version.
     * Pre 3.2 needs POI for each WP
     * 3.2+ will keep pointing to current poi POI until POI(0,0,0) received.*/
    bool auto_generate_mission_poi = true;

    int current_native_flight_mode = -1;

    /** Ardupilot version 3.3.1+ does not have FS_GPS_ENABLE parameter
     * It uses FS_EKF_ACTION for that instead.
     */
    bool use_ekf_action_as_gps_failsafe = false;

    /** Ardupilot version 3.3.1+ requires Home position
     * to be sent as MAV_CMD instead of mission item. */
    bool send_home_position_as_mav_cmd = false;

    /** MAV_CMD_GET_HOME_POSITION command to retrieve HL from vehicle.
     * Ardupilot version 3.4.0+ supports MAV_CMD_GET_HOME_POSITION command.
     * For older version we read waypoint #0 to get HL which can
     * potentially interfere with route download thus failing the upload if
     * HL retrieval is in progress.
     */
    bool use_get_home_position = false;

    /** Ardupilot version 3.4+ allows to adjust raw altitude with GND_ALT_OFFSET
     * parameter. Previous versions either don't have this parameter or have a bug
     */
    bool gnd_alt_offset_allow = false;

    /** Poll for home location until it is nonzero. */
    ugcs::vsm::Timer_processor::Timer::Ptr home_location_timer;

    ugcs::vsm::Geodetic_tuple home_location {0, 0, 0};

    /** Joystick mode support */

    // Timer instance for sending rc_override messages.
    ugcs::vsm::Timer_processor::Timer::Ptr rc_override_timer = nullptr;

    // RC Override message which holds the latest joystick values.
    // Existence of this messages means that vehicle is in joystick mode.
    ugcs::vsm::mavlink::Pld_rc_channels_override::Ptr rc_override = nullptr;

    // Last time we sent the rc_overrride message. Used to limit rate at which
    // the joystick commands are sent to vehicle.
    std::chrono::time_point<std::chrono::steady_clock> rc_override_last_sent;

    // Last time we received joystick message from ucs. Used to fail over to
    // manual mode automatically if no messages received for RC_OVERRIDE_TIMEOUT.
    std::chrono::time_point<std::chrono::steady_clock> direct_vehicle_control_last_received;

    // Timeout when to revert back to manual mode if no joystick messages
    // received from ucs.
    constexpr static std::chrono::milliseconds RC_OVERRIDE_TIMEOUT {3000};

    // Poll for HL each 10 seconds until success.
    constexpr static std::chrono::seconds HL_POLLING_PERIOD {10};

    // Counter which governs the end of Joystick mode. To ensure ardupilot
    // exits rc_override we need to spam 0,0,0,0 messages.
    size_t rc_override_end_counter = 0;

    // How frequently to send the rc_overrride messages to vehicle.
    constexpr static std::chrono::milliseconds RC_OVERRIDE_PERIOD {200};

    // How many 0,0,0,0 rc_override messages to send to exit joystick mode.
    constexpr static size_t RC_OVERRIDE_END_COUNT = 15;

    ugcs::vsm::Property::Ptr t_adsb_altitude_internal;
    ugcs::vsm::Property::Ptr t_adsb_transponder_mode;
    ugcs::vsm::Property::Ptr t_adsb_altitude;
    ugcs::vsm::Property::Ptr t_adsb_icao;
    ugcs::vsm::Property::Ptr t_adsb_squawk;
    ugcs::vsm::Property::Ptr t_adsb_ident_active;
    ugcs::vsm::Property::Ptr t_adsb_flight;
    ugcs::vsm::Property::Ptr t_adsb_registration;
    ugcs::vsm::Property::Ptr t_adsb_error_xpdr;
    ugcs::vsm::Property::Ptr t_adsb_error_icao;
    ugcs::vsm::Property::Ptr t_adsb_error_gps;
    ugcs::vsm::Property::Ptr t_adsb_error_squitter;
    ugcs::vsm::Property::Ptr t_adsb_error_temperature;

    std::unordered_map<std::string, ugcs::vsm::Property::Ptr> adsb_parameter_map;
    std::unordered_map<std::string, ugcs::vsm::Property::Ptr> adsb_string_parameter_map;

    bool is_airborne = false;

    // Onboard transponder type if configured.
    ugcs::vsm::Optional<int> adsb_transponder_type;

    virtual bool
    Verify_parameter(const std::string& name, float value, ugcs::vsm::mavlink::MAV_PARAM_TYPE& type);

    float current_alt_offset = 0;

    // This is a plane capable of VTOL.
    bool vtol_plane = false;
};

#endif /* _ARDUPILOT_VEHICLE_H_ */
