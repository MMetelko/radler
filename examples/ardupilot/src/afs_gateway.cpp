#include "afs_gateway.h"

AFS_Gateway::AFS_Gateway()
{	
	node = rclcpp::Node::make_shared("afs_gateway");

	mavros_battery_subscriber = node->create_subscription<sensor_msgs::msg::BatteryState>("/mavros/battery", rclcpp::SensorDataQoS(), std::bind(&AFS_Gateway::mavros_battery_state_callback, this, std::placeholders::_1)); // queue size 2 is good enough to capture battery message updates at 4Hz
	mavlink_from_subscriber = node->create_subscription<mavros_msgs::msg::Mavlink>("/mavlink/from", rclcpp::SensorDataQoS(), std::bind(&AFS_Gateway::mavlink_fence_status_callback, this, std::placeholders::_1)); // queuesize 20 needed as geofence status message is one of the many mvlink messages arriving at 120Hz and must be filtered at callback without loss
	mavros_gpsraw_subscriber = node->create_subscription<mavros_msgs::msg::GPSRAW>("/mavros/gpsstatus/gps1/raw", rclcpp::SensorDataQoS(), std::bind(&AFS_Gateway::mavros_gps_status_callback, this, std::placeholders::_1)); // queue size 2 is good enough to capture gps status message updates at 4Hz
	flight_controls_mode = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
	mavros_autopilotstate_subscriber = node->create_subscription<mavros_msgs::msg::State>("/mavros/state", rclcpp::SensorDataQoS(), std::bind(&AFS_Gateway::mavros_autopilotstate_callback, this, std::placeholders::_1)); // queue size 2 is good enough to capture mavros state message updates at 1Hz
	mavros_missionwaypoints_subscriber = node->create_subscription<mavros_msgs::msg::WaypointList>("/mavros/mission/waypoints", rclcpp::SensorDataQoS(), std::bind(&AFS_Gateway::mavros_missionwaypoints_callback, this, std::placeholders::_1)); // queue size 2 is good enough to capture mavros mission waypoint message updates atvery slow < 0.5 Hz
	mavros_globalposition_subscriber = node->create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global", rclcpp::SensorDataQoS(), std::bind(&AFS_Gateway::mavros_globalposition_callback, this, std::placeholders::_1)); // queue size 2 is good enough to capture global position from EKF with GPS Fix message updates at 4Hz
	mavros_diagnostics_subscriber = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::SensorDataQoS(), std::bind(&AFS_Gateway::mavros_diagnostics_callback, this, std::placeholders::_1)); // queue size 10 is good enough to capture FCS Diagnostics message updates at < 1Hz
	previous_flight_controls_cmd_id = 0;
	previous_diagnostics_heartbeat_value = -1;
	current_heartbeat_loss_duration = 0.0;
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
			cout << "AFS Gateway at (" << formatTimestamp(current_time) << ") "
					<< "battery remaining: " << o->battery_status->remaining_percentage << "% "
					<< "with status message at (" << formatTimestamp(this->battery_status_mailbox->header.stamp) << ") "
					<< endl;
			this->battery_status_mailbox = nullptr;
			radl_turn_off(radl_STALE, &o_f->battery_status);
		} else {
			radl_turn_on(radl_STALE, &o_f->battery_status);
		}
		radl_turn_off(radl_TIMEOUT, &o_f->battery_status);

		if (this->geofence_status_mailbox) {
			//Copying the payload from mavros_msgs::Mavlink to mavlink_message_t and filling in the geofence_status appropriately
			if (!this->geofence_status_mailbox->payload64.empty()) {
				memcpy(mmsg.payload64, &this->geofence_status_mailbox->payload64[0], 
					this->geofence_status_mailbox->payload64.size() * sizeof(uint64_t));
			}
			//std::copy(this->geofence_status_mailbox->payload64.begin(), this->geofence_status_mailbox->payload64.end(), mmsg.payload64);
			o->geofence_status->breach_status = mavlink_msg_fence_status_get_breach_status(&mmsg);
			o->geofence_status->breach_count = mavlink_msg_fence_status_get_breach_count(&mmsg);
			o->geofence_status->breach_type = mavlink_msg_fence_status_get_breach_type(&mmsg);
			o->geofence_status->breach_time = mavlink_msg_fence_status_get_breach_time(&mmsg);
			cout << "AFS Gateway at (" << formatTimestamp(current_time) << ") "
					<< "geofence breach (status: 0/1 inside fence or outside, count: # breaches,  breach_type: 0/1/2/3 for none/min_alt/max_alt/bundary, breach time (ms) since boot of last breach): "
					<< "==> (" << (int) o->geofence_status->breach_status << ", " << (int) o->geofence_status->breach_count << ", "
					<< (int) o->geofence_status->breach_type << ", " << (int) o->geofence_status->breach_time << ") "
					<< "with status message at (" << formatTimestamp(this->geofence_status_mailbox->header.stamp) << ") "
					<< endl;
			this->geofence_status_mailbox = nullptr;
			radl_turn_off(radl_STALE, &o_f->geofence_status);
		} else {
			radl_turn_on(radl_STALE, &o_f->geofence_status);
		}
		radl_turn_off(radl_TIMEOUT, &o_f->geofence_status);

		if (this->gps_status_mailbox) {
			o->gps_status->fix_type = this->gps_status_mailbox->fix_type;
			o->gps_status->satellites_visible = this->gps_status_mailbox->satellites_visible;
			if ((o->gps_status->fix_type == static_cast<uint8_t>(GPS_FIX_TYPE_NO_GPS)) || (o->gps_status->fix_type == static_cast<uint8_t>(GPS_FIX_TYPE_NO_FIX))){
				cout << "AFS Gateway at (" << formatTimestamp(current_time) << ") "
						<< "GPS FIX LOSS with visible satellites: " << (int) o->gps_status->satellites_visible  << " "
						<< "with status message at (" << formatTimestamp(this->gps_status_mailbox->header.stamp) << ") "
						<< endl;
			} else {
				cout << "AFS Gateway at (" << formatTimestamp(current_time) << ") "
						<< "GPS fine/normal with visible satellites: " << (int) o->gps_status->satellites_visible  << " "
						<< "with status message at (" << formatTimestamp(this->gps_status_mailbox->header.stamp) << ") "
						<< endl;
			}
			this->gps_status_mailbox = nullptr;
			radl_turn_off(radl_STALE, &o_f->gps_status);
		} else {
			radl_turn_on(radl_STALE, &o_f->gps_status);
		}
		radl_turn_off(radl_TIMEOUT, &o_f->gps_status);

		if (this->autopilotstate_status_mailbox) {
			cout << "AFS Gateway at (" << formatTimestamp(current_time) << ") "
					<< "Autopilot Mode: " << this->autopilotstate_status_mailbox->mode << " "
					<< "(connected,armed,guided,manual_input,system_status): " << "(" << (int)this->autopilotstate_status_mailbox->connected
					<< "," << (int)this->autopilotstate_status_mailbox->armed << "," << (int)this->autopilotstate_status_mailbox->guided
					<< "," << (int)this->autopilotstate_status_mailbox->manual_input << "," << (int)this->autopilotstate_status_mailbox->system_status << ") "
					<< "with status message at (" << formatTimestamp(this->autopilotstate_status_mailbox->header.stamp) << ") "
					<< endl;
		} else {
			cout << "Autopilot status mailbox is null" << endl;
		}


		if (this->missionwaypoints_status_mailbox) {
			int seq = (int)this->missionwaypoints_status_mailbox->current_seq;
			cout << "AFS Gateway at (" << formatTimestamp(current_time) << ") "
					<< "Current/Last reached Waypoint (seq/total,x_lat,y_long,z_alt): " << "("
					<< seq << "/" << (((int) *RADL_THIS->max_number_mission_waypoints) - 1) << ","
					<< (double) this->missionwaypoints_status_mailbox->waypoints[seq].x_lat << ","
					<< (double) this->missionwaypoints_status_mailbox->waypoints[seq].y_long << ","
					<< (double) this->missionwaypoints_status_mailbox->waypoints[seq].z_alt << ")"
					<< endl;
		}
		else {
			cout << "Mission Way Points status mailbox is null" << endl;
		}

		if (this->globalposition_status_mailbox) {
			cout << "AFS Gateway at (" << formatTimestamp(current_time) << ") "
					<< "Global Navigation Position (x_lat,y_long,z_alt): " << "("
					<< (double) this->globalposition_status_mailbox->latitude << ","
					<< (double) this->globalposition_status_mailbox->longitude << ","
					<< (double) this->globalposition_status_mailbox->altitude << ") "
					<< "with status message at (" << formatTimestamp(this->globalposition_status_mailbox->header.stamp) << ") "
					<< endl;
		this->globalposition_status_mailbox = nullptr;
		} else {
			cout << "Global Position status mailbox is null" << endl;
		}

		if (this->diagnostics_status_mailbox) {
			try {
				if (previous_diagnostics_heartbeat_value < 0){
					// first time
					cout << "First time diagnostics status message..." << endl;
					if (this->diagnostics_status_mailbox->status.size() > 2) {
						if (!this->diagnostics_status_mailbox->status[2].values.empty()) {
							previous_diagnostics_status_time = this->diagnostics_status_mailbox->header.stamp;
							previous_diagnostics_heartbeat_value = std::stoi(this->diagnostics_status_mailbox->status[2].values[0].value);
						} else {
							cout << "Invalid status array values is empty" << endl;
							return;
						} 
					} else {
						cout << "Invalid status size: " << this->diagnostics_status_mailbox->status.size() << endl;
						return;
					}
				}

				elapsed_diagnostics_status_duration = ((double)this->diagnostics_status_mailbox->header.stamp.sec - (double)previous_diagnostics_status_time.seconds());
				elapsed_diagnostics_status_duration += (((double)this->diagnostics_status_mailbox->header.stamp.nanosec - (double)previous_diagnostics_status_time.nanoseconds())/1000000000.0);
				cout << "elapsed_diagnostics_status_duration set..." << endl;
				if (this->diagnostics_status_mailbox->status.size() > 2) {
					if (!this->diagnostics_status_mailbox->status[2].values.empty()) {
						current_diagnostics_heartbeat_value = std::stoi(this->diagnostics_status_mailbox->status[2].values[0].value);
						cout << "current_diagnostics_heartbeat_value set..." << endl;
					} else {
						cout << "Invalid status array values is empty" << endl;
						return;
					} 
				} else {
					cout << "Invalid status size: " << this->diagnostics_status_mailbox->status.size() << endl;
					return;
				}
				
				if ((current_diagnostics_heartbeat_value - previous_diagnostics_heartbeat_value) >= ((int)(elapsed_diagnostics_status_duration))){
					// No Hearbeat Loss
					current_heartbeat_loss_duration = 0.0;
				} else {
					// Hearbeat Loss
					current_heartbeat_loss_duration += elapsed_diagnostics_status_duration;
				}

				cout << "AFS Gateway at (" << formatTimestamp(current_time) << ") "
						<< "Diagnostic Status (name,heartbeat_key,hearbeat_value,prev_hearbeat_value,elapsed(s),loss_duration(s)): " << "("
						<< this->diagnostics_status_mailbox->status[2].name << ","
						<< this->diagnostics_status_mailbox->status[2].values[0].key << ","
						<< current_diagnostics_heartbeat_value  << "," << previous_diagnostics_heartbeat_value << ","
						<< elapsed_diagnostics_status_duration << "," << current_heartbeat_loss_duration << ") "
						<< "with status message at (" << formatTimestamp(this->diagnostics_status_mailbox->header.stamp) << ") "
						<< endl;
				previous_diagnostics_status_time = this->diagnostics_status_mailbox->header.stamp;
				previous_diagnostics_heartbeat_value = std::stoi(this->diagnostics_status_mailbox->status[2].values[0].value);
				this->diagnostics_status_mailbox = nullptr;
			} catch (const std::exception& e) {
				cout << "Exception in diagnostics processing: " << e.what() << endl;
			}
		} else {
			cout << "Diagnostics status mailbox is null" << endl;
		}

		//ignore staleness check as might have repeated send command to autopilot on failure
		//if (!radl_is_stale(i_f->copter_command) && !radl_is_timeout(i_f->copter_command)) {
		if (!radl_is_timeout(i_f->copter_command)) {
			cout << "AFS Gateway at (" << formatTimestamp(current_time) << ") ";
			if (i->copter_command->cmd_id == 0) {
				cout << "Copter Command: None" << endl;
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
				cout << "Copter Command: Return to Launch i.e. Returns to above takeoff location and then landing" << endl;
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
				cout << "Copter Command: Land At Current Location i.e. Reduces altitude to ground level, attempts to go straight down" << endl;
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
				cout << "Copter Command: Hover At Current Location i.e. AltHold -> holds altitude. Loiter mode not used and so position not held because it requires GPS. Instead using Guided_NoGPS after AltHold" << endl;
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
				cout << "Copter Command: Normal Flight i.e. Auto mode -> Executes pre-defined mission and continue following next set of waypoints" << endl;
			}
		}
	} catch (const std::exception& e) {
		cout << "Exception in step function: " << e.what() << endl;
	} catch (...) {
		cout << "Unknown exception in step function" << endl;
	}
}

void AFS_Gateway::mavros_battery_state_callback(const sensor_msgs::msg::BatteryState::ConstSharedPtr bs)
{
	this->battery_status_mailbox = bs;
}

void AFS_Gateway::mavlink_fence_status_callback(const mavros_msgs::msg::Mavlink::ConstSharedPtr fs){
	if (fs->msgid == 162) { //FENCE_STATUS with msgid #162
		this->geofence_status_mailbox = fs;
	} // else skip processing the mavlink message
}

void AFS_Gateway::mavros_gps_status_callback(const mavros_msgs::msg::GPSRAW::ConstSharedPtr gs)
{
	this->gps_status_mailbox = gs;
}

void AFS_Gateway::mavros_autopilotstate_callback(const mavros_msgs::msg::State::ConstSharedPtr aps){
	this->autopilotstate_status_mailbox = aps;
}

void AFS_Gateway::mavros_missionwaypoints_callback(const mavros_msgs::msg::WaypointList::ConstSharedPtr mws){
	this->missionwaypoints_status_mailbox = mws;
}

void AFS_Gateway::mavros_globalposition_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr gps){
	this->globalposition_status_mailbox = gps;
}

void AFS_Gateway::mavros_diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr das){
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