#include "afs_gateway.h"

// Highlights Event Actions
//const std::string RED "\033[0;31m"
const std::string RED = "\033[31m";
const std::string RESET = "\033[0m";

const char* breach_status[] = {"Inside", "Outside"};
const char* breach_types[] = {"None", "Min Altitude", "Max Altitude", "Fence Boundary"};


AFS_Gateway::AFS_Gateway()
{    
    node = rclcpp::Node::make_shared("afs_gateway");

    // Setup QoS settings to match previous configuration for ROS1 demo
    // For MAVLink, queue size 20 needed as geofence status message is one of the many mvlink messages arriving at 120Hz and must be filtered at callback without loss
    rclcpp::QoS mavlink_qos(rclcpp::KeepLast(20));
    mavlink_qos.reliable();
    // queue size 10 is good enough to capture FCS Diagnostics message updates at < 1Hz
    rclcpp::QoS diagnostics_qos(rclcpp::KeepLast(10));
    diagnostics_qos.reliable();

    mavros_battery_subscriber = node->create_subscription<sensor_msgs::msg::BatteryState>("/mavros/battery", rclcpp::SensorDataQoS(), std::bind(&AFS_Gateway::mavros_battery_state_callback, this, std::placeholders::_1)); // queue size 2 is good enough to capture battery message updates at 4Hz
    mavlink_from_subscriber = node->create_subscription<mavros_msgs::msg::Mavlink>("/uas1/mavlink_source", mavlink_qos, std::bind(&AFS_Gateway::mavlink_callback, this, std::placeholders::_1)); // queuesize 20 needed as geofence status message is one of the many mvlink messages arriving at 120Hz and must be filtered at callback without loss
    mavros_gpsraw_subscriber = node->create_subscription<mavros_msgs::msg::GPSRAW>("/mavros/mavros/gps1/raw", rclcpp::SensorDataQoS(), std::bind(&AFS_Gateway::mavros_gps_status_callback, this, std::placeholders::_1)); // queue size 2 is good enough to capture gps status message updates at 4Hz
    flight_controls_mode = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    mavros_autopilotstate_subscriber = node->create_subscription<mavros_msgs::msg::State>("/mavros/state", rclcpp::SensorDataQoS(), std::bind(&AFS_Gateway::mavros_autopilotstate_callback, this, std::placeholders::_1)); // queue size 2 is good enough to capture mavros state message updates at 1Hz
    mavros_missionwaypoints_subscriber = node->create_subscription<mavros_msgs::msg::WaypointList>("/mavros/mavros/waypoints", rclcpp::SensorDataQoS(), std::bind(&AFS_Gateway::mavros_missionwaypoints_callback, this, std::placeholders::_1)); // queue size 2 is good enough to capture mavros mission waypoint message updates atvery slow < 0.5 Hz
    //mavros_globalposition_subscriber = node->create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global", rclcpp::SensorDataQoS(), std::bind(&AFS_Gateway::mavros_globalposition_callback, this, std::placeholders::_1)); // queue size 2 is good enough to capture global position from EKF with GPS Fix message updates at 4Hz
    mavros_diagnostics_subscriber = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", diagnostics_qos, std::bind(&AFS_Gateway::mavros_diagnostics_callback, this, std::placeholders::_1)); // queue size 10 is good enough to capture FCS Diagnostics message updates at < 1Hz
    previous_flight_controls_cmd_id = 0;
    previous_diagnostics_heartbeat_value = -1;
    current_heartbeat_loss_duration = 0.0;
    geofence_status_available = false;
    global_position_status_available = false;

    currentStatus.battery_status = "";
    currentStatus.autopilot_mode = "";
    currentStatus.copter_command = " ";
    currentStatus.current_waypoint = " ";
    currentStatus.global_position = "";
    currentStatus.gps_status = "";
    currentStatus.geofence_status = "";
    currentStatus.diagnostics = "";
    currentStatus.debug_data.mavlink_gps_info = "";
    currentStatus.debug_data.mavlink_fs_info = "";
    currentStatus.debug_data.error_msgs = "";
}

void AFS_Gateway::step(const radl_in_t* i, const radl_in_flags_t* i_f, radl_out_t* o, radl_out_flags_t* o_f)
{
    try {
        rclcpp::Time current_time = node->now();
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->base_mode = 0;
    
        rclcpp::spin_some(node);

        mavlink_message_t mmsg;

        if (this->battery_status_mailbox) {
            o->battery_status->remaining_percentage = (this->battery_status_mailbox->percentage * 100.0); // [0.0,1.0] to [0.0, 100.0]
            currentStatus.battery_status = "Battery Remaining: " + std::to_string((int) o->battery_status->remaining_percentage) + "% \n";
            this->battery_status_mailbox = nullptr;
            radl_turn_off(radl_STALE, &o_f->battery_status);
        } else {
            radl_turn_on(radl_STALE, &o_f->battery_status);
        }
        radl_turn_off(radl_TIMEOUT, &o_f->battery_status);

        if (this->geofence_status_available) {
            o->geofence_status->breach_status = this->geofence_status_mailbox.breach_status;
            o->geofence_status->breach_count = this->geofence_status_mailbox.breach_count;
            o->geofence_status->breach_type = this->geofence_status_mailbox.breach_type;
            o->geofence_status->breach_time = this->geofence_status_mailbox.breach_time;

            // MM TODO: for debugging only
            //cout << RED << "DEBUGGING ONLY: Breach Mitigation (none/velocity limiting/position limiting/landing/return to launch): " << (int) this->geofence_status_mailbox.breach_mitigation << RESET << endl;

            std::string color = (this->geofence_status_mailbox.breach_status == 1) ? RED : "";
            currentStatus.geofence_status = color + "Geofence Breach: Status = " + std::string(breach_status[o->geofence_status->breach_status]) + 
                                            "\n  # Breaches = " + std::to_string(o->geofence_status->breach_count) +
                                            ", Breach Type = " + std::string(breach_types[this->geofence_status_mailbox.breach_type]) +
                                            "\n, Breach Time = " + std::to_string(o->geofence_status->breach_time) + "ms (since boot of last breach)" + RESET + "\n";
            currentStatus.debug_data.mavlink_fs_info += "Fence Breach reached mailbox... ";

            this->geofence_status_available = false;
            radl_turn_off(radl_STALE, &o_f->geofence_status);
        } else {
            radl_turn_on(radl_STALE, &o_f->geofence_status);
        }
        radl_turn_off(radl_TIMEOUT, &o_f->geofence_status);

        if (this->gps_status_mailbox) {
            o->gps_status->fix_type = this->gps_status_mailbox->fix_type;
            o->gps_status->satellites_visible = this->gps_status_mailbox->satellites_visible;

            bool isGpsFixLoss = (o->gps_status->fix_type == static_cast<uint8_t>(GPS_FIX_TYPE_NO_GPS)) || 
                                (o->gps_status->fix_type == static_cast<uint8_t>(GPS_FIX_TYPE_NO_FIX));
        
            std::string color = isGpsFixLoss ? RED : "";
            std::string status = isGpsFixLoss ? "GPS FIX LOSS" : "GPS fine/normal";
            
            currentStatus.gps_status = color + status + " with visible satellites: " + 
                                        std::to_string(o->gps_status->satellites_visible) + RESET + "\n";

            this->gps_status_mailbox = nullptr;
            radl_turn_off(radl_STALE, &o_f->gps_status);
        } else {
            radl_turn_on(radl_STALE, &o_f->gps_status);
        }
        radl_turn_off(radl_TIMEOUT, &o_f->gps_status);

        if (this->autopilotstate_status_mailbox) {
            currentStatus.autopilot_mode = "Autopilot Mode: " + this->autopilotstate_status_mailbox->mode + "\n";
        } else {
            currentStatus.autopilot_mode = "Autopilot status mailbox is null\n";
        }

        if (this->missionwaypoints_status_mailbox) {
            int seq = (int)this->missionwaypoints_status_mailbox->current_seq;
            currentStatus.current_waypoint = "Current Waypoint " +
                    std::to_string(seq) + "/" + 
                    std::to_string((((int) *RADL_THIS->max_number_mission_waypoints) - 1)) + " (seq/total): " +
                    std::to_string((double) this->missionwaypoints_status_mailbox->waypoints[seq].x_lat) + "," +
                    std::to_string((double) this->missionwaypoints_status_mailbox->waypoints[seq].y_long) + "," +
                    std::to_string((double) this->missionwaypoints_status_mailbox->waypoints[seq].z_alt) + "(lat,long,alt)\n";
        }
        else {
            currentStatus.current_waypoint = "Mission Way Points status mailbox is null\n";
        }

        if (this->global_position_status_available) {
            try {
                currentStatus.debug_data.mavlink_gps_info += "Global Navigation Poition reached mailbox... ";
                currentStatus.global_position = "Global Navigation Position (x_lat,y_long,z_alt,z_alt_relative): " 
                        + to_string((double) this->globalposition_status_mailbox.lat * 1e-7) + ","
                        + to_string((double) this->globalposition_status_mailbox.lon * 1e-7) + ","
                        + to_string((double) this->globalposition_status_mailbox.alt * 1e-3) + ","
                        + to_string((double) this->globalposition_status_mailbox.relative_alt * 1e-3) + "\n";
                this->global_position_status_available = false;
            } catch (const std::exception& e) {
                currentStatus.debug_data.error_msgs += std::string("Exception in global position status check: ") + e.what() + "\n";
            } catch (...) {
                currentStatus.debug_data.error_msgs += "Unknown exception in global position status check\n";
            }
        } 

        if (this->diagnostics_status_mailbox) {
            try {
                    if (previous_diagnostics_heartbeat_value < 0){ 
                        if (this->diagnostics_status_mailbox->status.size() > 2) {
                            if (!this->diagnostics_status_mailbox->status[2].values.empty()) {
                                previous_diagnostics_status_time = this->diagnostics_status_mailbox->header.stamp;
                                previous_diagnostics_heartbeat_value = std::stoi(this->diagnostics_status_mailbox->status[2].values[0].value);
                            } else {
                                currentStatus.diagnostics = "Invalid status array values is empty\n";
                                return;
                            } 
                        } else {
                            currentStatus.diagnostics = "Invalid status size: " + std::to_string(this->diagnostics_status_mailbox->status.size()) + "\n";
                            return;
                        }
                    }

                elapsed_diagnostics_status_duration = ((double)this->diagnostics_status_mailbox->header.stamp.sec - (double)previous_diagnostics_status_time.seconds());
                elapsed_diagnostics_status_duration += (((double)this->diagnostics_status_mailbox->header.stamp.nanosec - (double)previous_diagnostics_status_time.nanoseconds())/1000000000.0);
                currentStatus.diagnostics = "elapsed_diagnostics_status_duration set...\n";
                if (this->diagnostics_status_mailbox->status.size() > 2) {
                    if (!this->diagnostics_status_mailbox->status[2].values.empty()) {
                        current_diagnostics_heartbeat_value = std::stoi(this->diagnostics_status_mailbox->status[2].values[0].value);
                        currentStatus.diagnostics += "current_diagnostics_heartbeat_value set...\n";
                    } else {
                        currentStatus.diagnostics += "Invalid status array values is empty\n";
                        return;
                    } 
                } else {
                    currentStatus.diagnostics += "Invalid status size: " + std::to_string(this->diagnostics_status_mailbox->status.size()) + "\n";
                    return;
                }
                
                if ((current_diagnostics_heartbeat_value - previous_diagnostics_heartbeat_value) >= ((int)(elapsed_diagnostics_status_duration))){
                    // No Heartbeat Loss
                    current_heartbeat_loss_duration = 0.0;
                } else {
                    // Heartbeat Loss
                    current_heartbeat_loss_duration += elapsed_diagnostics_status_duration;
                }

                currentStatus.diagnostics += "Diagnostic Status (name,heartbeat_key,heartbeat_value,prev_heartbeat_value,elapsed(s),loss_duration(s)): (" +
                        this->diagnostics_status_mailbox->status[2].name + "," +
                        this->diagnostics_status_mailbox->status[2].values[0].key + "," +
                        std::to_string(current_diagnostics_heartbeat_value)  + "," + 
                        std::to_string(previous_diagnostics_heartbeat_value) + "," +
                        std::to_string(elapsed_diagnostics_status_duration) + "," + 
                        std::to_string(current_heartbeat_loss_duration) + ") \n";
                previous_diagnostics_status_time = this->diagnostics_status_mailbox->header.stamp;
                previous_diagnostics_heartbeat_value = std::stoi(this->diagnostics_status_mailbox->status[2].values[0].value);
                this->diagnostics_status_mailbox = nullptr;
            } catch (const std::exception& e) {
                currentStatus.diagnostics += "Exception in diagnostics processing: " + std::string(e.what()) + "\n";
            }
        } else {
            currentStatus.diagnostics = "Diagnostics status mailbox is null\n";
        }

        //ignore staleness check as might have repeated send command to autopilot on failure
        //if (!radl_is_stale(i_f->copter_command) && !radl_is_timeout(i_f->copter_command)) {
        if (!radl_is_timeout(i_f->copter_command)) {
            //cout << "AFS Gateway at (" << formatTimestamp(current_time) << ") ";
            if (i->copter_command->cmd_id == 0) {
                currentStatus.copter_command = "Copter Command: None\n";
            } else if (i->copter_command->cmd_id == 1) {
                if (
                        (previous_flight_controls_cmd_id != i->copter_command->cmd_id) // new command
                        || ((previous_flight_controls_cmd_id == i->copter_command->cmd_id)
                                    && (this->autopilotstate_status_mailbox->mode.compare(mavros_msgs::msg::State::MODE_APM_COPTER_RTL) != 0)) // previous command succesfully sent in previous iteration but mode was not changed in autopilot
                    ){
                    request->custom_mode = "RTL";
                    flight_controls_mode->async_send_request(request);
                    previous_flight_controls_cmd_id = i->copter_command->cmd_id;
                }
                currentStatus.copter_command = "Copter Command: Return to Launch \n  i.e. Returns to above takeoff location and then landing\n";
            } else if (i->copter_command->cmd_id == 2){
                if (
                        (previous_flight_controls_cmd_id != i->copter_command->cmd_id) // new command
                        || ((previous_flight_controls_cmd_id == i->copter_command->cmd_id)
                                    && (this->autopilotstate_status_mailbox->mode.compare(mavros_msgs::msg::State::MODE_APM_COPTER_LAND) != 0)) // previous command succesfully sent in previous iteration but mode was not changed in autopilot
                    ){
                    request->custom_mode = "LAND";
                    flight_controls_mode->async_send_request(request);
                    previous_flight_controls_cmd_id = i->copter_command->cmd_id;
                }
                currentStatus.copter_command = "Copter Command: Land At Current Location \n  i.e. Reduces altitude to ground level, attempts to go straight down\n";
            } else if (i->copter_command->cmd_id == 3){
                if (
                        (previous_flight_controls_cmd_id != i->copter_command->cmd_id) // new command
                        || ((previous_flight_controls_cmd_id == i->copter_command->cmd_id)
                                    && (this->autopilotstate_status_mailbox->mode.compare(mavros_msgs::msg::State::MODE_APM_COPTER_GUIDED_NOGPS) != 0)) // previous command succesfully sent in previous iteration but mode was not changed in autopilot
                    ){
                    request->custom_mode = "ALT_HOLD";
                    flight_controls_mode->async_send_request(request);
                    request->custom_mode = "GUIDED_NOGPS";
                    flight_controls_mode->async_send_request(request);
                    previous_flight_controls_cmd_id = i->copter_command->cmd_id;
                }
                currentStatus.copter_command = "Copter Command: Hover At Current Location \n  i.e. AltHold -> holds altitude. Loiter mode not used and so position not held because it requires GPS. Instead using Guided_NoGPS after AltHold\n";
            } else if (i->copter_command->cmd_id == 4){
                if (
                        (previous_flight_controls_cmd_id != i->copter_command->cmd_id) // new command
                        || ((previous_flight_controls_cmd_id == i->copter_command->cmd_id)
                                    && (this->autopilotstate_status_mailbox->mode.compare(mavros_msgs::msg::State::MODE_APM_COPTER_AUTO) != 0)) // previous command succesfully sent in previous iteration but mode was not changed in autopilot
                    ){
                    request->custom_mode = "AUTO";
                    flight_controls_mode->async_send_request(request);
                    previous_flight_controls_cmd_id = i->copter_command->cmd_id;
                }
                currentStatus.copter_command = "Copter Command: Normal Flight \n  i.e. Auto mode -> Executes pre-defined mission and continue following next set of waypoints\n";
            }
        }

        // Print all status information
        
        cout << "\033[2J\033[1;1H";  // Clear screen
        cout << "AFS Gateway Status at " << formatTimestamp(current_time) << "\n"
                << "-----------------------------------------\n"
                << currentStatus.battery_status 
                << currentStatus.autopilot_mode
                << currentStatus.copter_command
                << currentStatus.current_waypoint
                << currentStatus.global_position
                << currentStatus.gps_status             
                << currentStatus.geofence_status
                << "..........................................\n"
                << "GPS Info: "
                << currentStatus.debug_data.mavlink_gps_info
                << "\nFence Status Info: "
                << currentStatus.debug_data.mavlink_fs_info
                << "\nError Msgs: "
                << currentStatus.debug_data.error_msgs;
        
    } catch (const std::exception& e) {
        currentStatus.debug_data.error_msgs += std::string("Exception in step function: ") + e.what() + "\n";
    } catch (...) {
        currentStatus.debug_data.error_msgs += "Unknown exception in step function\n";
    }
}

void AFS_Gateway::mavros_battery_state_callback(const sensor_msgs::msg::BatteryState::ConstSharedPtr bs)
{
    this->battery_status_mailbox = bs;
}

void AFS_Gateway::mavlink_callback(const mavros_msgs::msg::Mavlink::ConstSharedPtr msg)
{
    try{
        currentStatus.debug_data.error_msgs += "Made it to mavlink_callback... msgid = " + std::to_string(msg->msgid) + "\n";
        if (msg->msgid == 33)
        //if (msg->msgid == static_cast<uint8_t>(MAVLINK_MSG_ID_GLOBAL_POSITION_INT))
        {
            currentStatus.debug_data.mavlink_gps_info += "Found msgid = 33 (GPS location message), now to decode...\n";
            size_t gp_payload_size = MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN;
            if (msg->payload64.size() == gp_payload_size)
            {
                currentStatus.debug_data.mavlink_gps_info += "Found Global Position MAVLink message...\n";
                mavlink_message_t mavlink_gp_msg;
                mavlink_gp_msg.msgid = msg->msgid;
                mavlink_gp_msg.sysid = msg->sysid;
                mavlink_gp_msg.compid = msg->compid;

                const uint64_t* gp_data_ptr = &msg->payload64[0];
                memcpy(mavlink_gp_msg.payload64, gp_data_ptr, gp_payload_size);
                currentStatus.debug_data.mavlink_gps_info += "Global Position MAVLink message copied...\n";
                // for (size_t i = 0; i < msg->payload64.size() && i < sizeof(mavlink_gp_msg.payload64)/sizeof(mavlink_gp_msg.payload64[0]); ++i) {
                //     mavlink_gp_msg.payload64[i] = msg->payload64[i];
                // }
                
                mavlink_msg_global_position_int_decode(&mavlink_gp_msg, &this->globalposition_status_mailbox);
                currentStatus.debug_data.mavlink_gps_info += "Global Position MAVLink message decoded...\n";
                this->global_position_status_available = true;
                this->global_position_timestamp = this->node->now();    
            }
        }
        else if (msg->msgid == 162) // MAVLINK_MSG_ID_FENCE_STATUS
        //else if (msg->msgid == static_cast<uint8_t>(MAVLINK_MSG_ID_FENCE_STATUS))
        {
            currentStatus.debug_data.mavlink_fs_info += "Found msgid = 162 (Fence Breach message), now to decode...\n";
            size_t fs_payload_size = MAVLINK_MSG_ID_FENCE_STATUS_LEN;
            if (msg->payload64.size() == fs_payload_size)
            {
                currentStatus.debug_data.mavlink_fs_info += "Found Fence Status MAVLink message...\n";
                mavlink_message_t mavlink_fs_msg;
                mavlink_fs_msg.msgid = msg->msgid;
                mavlink_fs_msg.sysid = msg->sysid;
                mavlink_fs_msg.compid = msg->compid;

                const uint64_t* fs_data_ptr = &msg->payload64[0];
                memcpy(mavlink_fs_msg.payload64, fs_data_ptr, fs_payload_size);
                currentStatus.debug_data.mavlink_fs_info += "Fence Status MAVLink message copied...\n";
                // for (size_t i = 0; i < msg->payload64.size() && i < sizeof(mavlink_fs_msg.payload64)/sizeof(mavlink_fs_msg.payload64[0]); ++i) {
                //     mavlink_fs_msg.payload64[i] = msg->payload64[i];
                // }
    
                mavlink_msg_fence_status_decode(&mavlink_fs_msg, &this->geofence_status_mailbox);
                currentStatus.debug_data.mavlink_fs_info += "Fence Status MAVLink message decoded...\n";
                this->geofence_status_available = true;
                this->geofence_status_timestamp = this->node->now();    
            }
        }
    } catch (const std::runtime_error& e) {
        currentStatus.debug_data.error_msgs += std::string("Runtime error in MAVLink callback: ") + e.what() + "\n";
    } catch (const std::invalid_argument& e) {
        currentStatus.debug_data.error_msgs += std::string("Invalid argument in MAVLink callback: ") + e.what() + "\n";
    } catch (...) {
        currentStatus.debug_data.error_msgs += "Unknown exception in MAVLink callback function\n";
    }
}

void AFS_Gateway::mavros_gps_status_callback(const mavros_msgs::msg::GPSRAW::ConstSharedPtr gs)
{
    this->gps_status_mailbox = gs;
}

void AFS_Gateway::mavros_autopilotstate_callback(const mavros_msgs::msg::State::ConstSharedPtr aps)
{
    this->autopilotstate_status_mailbox = aps;
}

void AFS_Gateway::mavros_missionwaypoints_callback(const mavros_msgs::msg::WaypointList::ConstSharedPtr mws)
{
    if (!mws->waypoints.empty())
    {
        this->missionwaypoints_status_mailbox = mws;
    }
}

// void AFS_Gateway::mavros_globalposition_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr gps)
// {
//     this->globalposition_status_mailbox = gps;
// }

void AFS_Gateway::mavros_diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr das)
{
    this->diagnostics_status_mailbox = das;
}

std::string AFS_Gateway::formatTimestamp(const builtin_interfaces::msg::Time& stamp) 
{
    auto time_point = std::chrono::system_clock::time_point(
        std::chrono::seconds(stamp.sec) +
        std::chrono::nanoseconds(stamp.nanosec)
    );
    std::time_t time = std::chrono::system_clock::to_time_t(time_point);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
    return oss.str();
}