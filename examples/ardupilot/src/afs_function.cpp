#include "afs_function.h"

AFS_Function::AFS_Function()
{
	node = rclcpp::Node::make_shared("afs_function");

	battery_remaining_percentage = 100.0;
	gps_fix_state = "Available";
	afs_state = "Normal Flight";
	gps_loss_count = 0;
	current_gps_loss_duration = -1.0;
	max_altitude_breach_event = "False";
	current_max_altitude_breach_duration = -1.0;
}

void AFS_Function::step(const radl_in_t * i, const radl_in_flags_t* i_f, radl_out_t * o, radl_out_flags_t* o_f)
{
	rclcpp::Time current_time = node->now();

	//auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
	//request->base_mode = 0;
  
	rclcpp::spin_some(node);
  
	if (!radl_is_stale(i_f->battery_status) && !radl_is_timeout(i_f->battery_status)) {
		battery_remaining_percentage = i->battery_status->remaining_percentage;
	}

	if (!radl_is_stale(i_f->gps_status) && !radl_is_timeout(i_f->gps_status)) {
		gps_fix_type = i->gps_status->fix_type;
		if ((gps_fix_type == static_cast<uint8_t>(GPS_FIX_TYPE::NO_GPS)) || (gps_fix_type == static_cast<uint8_t>(GPS_FIX_TYPE::NO_FIX))){
			gps_fix_state = "Lost";
			if (current_gps_loss_duration < 0.0) {
				// first time in the current stretch of potentially consecutive/coninuous loss
				current_gps_loss_duration = 0.0; // initialize current_gps_loss_duration for summing of duration of consecutive loss
				previous_gps_loss_time = current_time; // initialize previous_gps_loss_time for when the consecutive gps loss began
			}
			// Sum the consecutive GPS loss duration to keep track of accumulated time of GPS Loss consecutively
			current_gps_loss_duration = current_gps_loss_duration + ((double)current_time.seconds() - (double)previous_gps_loss_time.seconds());
			current_gps_loss_duration = current_gps_loss_duration + (((double)current_time.nanoseconds() - (double)previous_gps_loss_time.nanoseconds())/1000000000.0);
			previous_gps_loss_time = current_time; // override previous_gps_loss_time with current_time for tracking GPS Loss duration
		} else {
			gps_fix_state = "Available";
			// initialize current_gps_loss_duration to negative value to idenity the first time AFS function determines GPS loss for the
			// first time during a stretch of continuous loss in the future
			current_gps_loss_duration = -1.0;
		}
	}

	if (!radl_is_stale(i_f->geofence_status) && !radl_is_timeout(i_f->geofence_status)) {
		breach_status = i->geofence_status->breach_status;
		breach_type = i->geofence_status->breach_type;
		if ((breach_status == 1) && (breach_type == static_cast<uint8_t>(FENCE_BREACH::MAXALT)))
		{
			max_altitude_breach_event = "True";
			if (current_max_altitude_breach_duration < 0.0) {
				// first time in the current stretch of potentially consecutive/coninuous breaches
				current_max_altitude_breach_duration = 0.0; // initialize current_max_altitude_breach_duration for summing of duration of consecutive breaches
				previous_max_altitude_breach_time = current_time; // initialize previous_max_altitude_breach_time for when the consecutive gps loss began
			}
			// Sum the consecutive Max Altitude Breach duration to keep track of accumulated time of breaches consecutively
			current_max_altitude_breach_duration = current_max_altitude_breach_duration + ((double)current_time.seconds() - (double)previous_max_altitude_breach_time.seconds());
			current_max_altitude_breach_duration = current_max_altitude_breach_duration + (((double)current_time.nanoseconds() - (double)previous_max_altitude_breach_time.nanoseconds())/1000000000.0);
			previous_max_altitude_breach_time = current_time; // override previous_max_altitude_breach_time with current_time for tracking Max Altitude Breach duration
		} else {
			max_altitude_breach_event = "False";
			// initialize current_max_altitude_breach_duration to negative value to idenity the first time AFS function determines Max Altitude Breach for the
			// first time during a stretch of continuous max altitude breaches in the future
			current_max_altitude_breach_duration = -1.0;
		}
	}

	cout << "AFS Function at (" << current_time.seconds() << "s, " << current_time.nanoseconds() << "ns) ";
	cout << "remaining battery: " << battery_remaining_percentage << ", T_rtl: " << *RADL_THIS->battery_T_rtl
			 << ", T_land: " << *RADL_THIS->battery_T_land <<", max_GPS_losses_allowed: " << (int) *RADL_THIS->max_GPS_losses_allowed << endl;
	cout << ", AFS State: " << afs_state
			 << ", GPS Fix State: " << gps_fix_state << ", GPS Loss Count: " << gps_loss_count
			 << ", current gp loss duration: " << current_gps_loss_duration
			 << ", previous gps loss time: (" << previous_gps_loss_time.seconds() << "s, " << previous_gps_loss_time.nanoseconds() << "ns) "
			 << ", Max Altitude Breach Event: " << max_altitude_breach_event
			 << ", current max altitude breach duration: " << current_max_altitude_breach_duration
			 << ", previous max altitude breach event time: (" << previous_max_altitude_breach_time.seconds() << "s, " << previous_max_altitude_breach_time.nanoseconds() << "ns) "
			 << ", Copter Command: ";

	radl_turn_on(radl_STALE, &o_f->copter_command);
	radl_turn_on(radl_STALE, &o_f->gcs_message);
	if ((battery_remaining_percentage <= *RADL_THIS->battery_T_rtl) && (battery_remaining_percentage >= *RADL_THIS->battery_T_land)) {
		if (gps_fix_state.compare("Available") == 0){
			o->copter_command->cmd_id = 1;
			o->gcs_message->msg_id = 0;
			cout << "Copter Command: Return to Launch" << endl;
			afs_state = "Return to Launch Due to Low Battery";
			radl_turn_off(radl_STALE, &o_f->copter_command);
			radl_turn_off(radl_STALE, &o_f->gcs_message);
		} else if (gps_fix_state.compare("Lost") == 0){
			o->copter_command->cmd_id = 2;
			o->gcs_message->msg_id = 0;
			cout << "Copter Command: Land Immediately" << endl;
			afs_state = "Land Immediately Due to Low Battery and No GPS";
			radl_turn_off(radl_STALE, &o_f->copter_command);
			radl_turn_off(radl_STALE, &o_f->gcs_message);
		}
	} else if (battery_remaining_percentage < *RADL_THIS->battery_T_land) {
		o->copter_command->cmd_id = 2;
		o->gcs_message->msg_id = 0;
		cout << "Copter Command: Land Immediately" << endl;
		afs_state = "Land Immediately Due to Battery Critically Low";
		radl_turn_off(radl_STALE, &o_f->copter_command);
		radl_turn_off(radl_STALE, &o_f->gcs_message);
	}
	if (afs_state.compare("Return to Launch Due to Low Battery") == 0){
		if (gps_fix_state.compare("Lost") == 0){
			o->copter_command->cmd_id = 2;
			o->gcs_message->msg_id = 0;
			cout << "Copter Command: Land Immediately" << endl;
			afs_state = "Land Immediately Due to Low Battery and No GPS";
			radl_turn_off(radl_STALE, &o_f->copter_command);
			radl_turn_off(radl_STALE, &o_f->gcs_message);
		}
	}
	if ((afs_state.compare("Return to Launch Due to Low Battery") != 0)
		&& (afs_state.compare("Land Immediately Due to Battery Critically Low") != 0)
		&& (afs_state.compare("Land Immediately Due to Low Battery and No GPS") != 0)
		&& (battery_remaining_percentage > *RADL_THIS->battery_T_rtl)){
		if ((gps_fix_state.compare("Lost") == 0)
			&& (current_gps_loss_duration >= 3.0)
			&& (gps_loss_count < *RADL_THIS->max_GPS_losses_allowed)){
			o->copter_command->cmd_id = 3;
			o->gcs_message->msg_id = 0;
			cout << "Copter Command: Hover at Current Position, GCS Message: Copter transitioned to Hover" << endl;
			afs_state = "GPS Loss Hover";
			radl_turn_off(radl_STALE, &o_f->copter_command);
			radl_turn_off(radl_STALE, &o_f->gcs_message);
		}
	}
	if ((afs_state.compare("Return to Launch Due to Low Battery") != 0)
		&& (afs_state.compare("Land Immediately Due to Battery Critically Low") != 0)
		&& (afs_state.compare("Land Immediately Due to Low Battery and No GPS") != 0)
		&& (battery_remaining_percentage > *RADL_THIS->battery_T_rtl)){
		if ((gps_fix_state.compare("Lost") == 0)
			&& (current_gps_loss_duration >= 3.0)
			&& (gps_loss_count >= *RADL_THIS->max_GPS_losses_allowed)){
			o->copter_command->cmd_id = 2;
			o->gcs_message->msg_id = 0;
			cout << "Copter Command: Abandon Mission and Land Vertically Down" << endl;
			afs_state = "Mission Abandoned";
			radl_turn_off(radl_STALE, &o_f->copter_command);
			radl_turn_off(radl_STALE, &o_f->gcs_message);
		}
	}
	if ((afs_state.compare("GPS Loss Hover") == 0)
		&& (battery_remaining_percentage > *RADL_THIS->battery_T_rtl)){
		if (gps_fix_state.compare("Available") == 0){
			o->copter_command->cmd_id = 4;
			o->gcs_message->msg_id = 0;
			cout << "Copter Command: GPS Loss Transitions to GPS Fix and so AFS State transitions from GPS Loss Hover to Normal Flight" << endl;
			afs_state = "Normal Flight";
			gps_loss_count = gps_loss_count + 1;
			radl_turn_off(radl_STALE, &o_f->copter_command);
			radl_turn_off(radl_STALE, &o_f->gcs_message);
		}
	}
	if (((gps_fix_state.compare("Available") == 0) || ((gps_fix_state.compare("Lost") == 0) && (current_gps_loss_duration < 3.0)))
		&& (battery_remaining_percentage > *RADL_THIS->battery_T_rtl)
		&& (afs_state.compare("Normal Flight") == 0)){
			if (max_altitude_breach_event.compare("True") == 0){
				o->copter_command->cmd_id = 4;
				o->gcs_message->msg_id = 0;
				cout << "Copter Command: Drop to Target Altitude MAX_ALTITUDE-margin; Continue trying to command Auto mode for now to continue with mission" << endl;
				afs_state = "Max Altitude Breach";
				radl_turn_off(radl_STALE, &o_f->copter_command);
				radl_turn_off(radl_STALE, &o_f->gcs_message);
			}
	}
	if (((gps_fix_state.compare("Available") == 0) || ((gps_fix_state.compare("Lost") == 0) && (current_gps_loss_duration < 3.0)))
		&& (battery_remaining_percentage > *RADL_THIS->battery_T_rtl)
		&& (afs_state.compare("Max Altitude Breach") == 0)){
			if ((max_altitude_breach_event.compare("True") == 0)
				&& (current_max_altitude_breach_duration >= 100.0)){ // instead 5.0 second using 100.0 secs to disable this LAND mode
				o->copter_command->cmd_id = 2;
				o->gcs_message->msg_id = 0;
				cout << "Copter Command: Land Immediately" << endl;
				afs_state = "Land due to Max Altitude Breach";
				radl_turn_off(radl_STALE, &o_f->copter_command);
				radl_turn_off(radl_STALE, &o_f->gcs_message);
			}
	}
	if (((gps_fix_state.compare("Available") == 0) || ((gps_fix_state.compare("Lost") == 0) && (current_gps_loss_duration < 3.0)))
		&& (battery_remaining_percentage > *RADL_THIS->battery_T_rtl)
		&& (afs_state.compare("Max Altitude Breach") == 0)){
			if (max_altitude_breach_event.compare("False") == 0){
				o->copter_command->cmd_id = 4;
				o->gcs_message->msg_id = 0;
				cout << "Copter Command: Continue Original Mission; Send Auto Command for now" << endl;
				afs_state = "Normal Flight";
				radl_turn_off(radl_STALE, &o_f->copter_command);
				radl_turn_off(radl_STALE, &o_f->gcs_message);
			}
	}
	radl_turn_off(radl_TIMEOUT, &o_f->copter_command);
	radl_turn_off(radl_TIMEOUT, &o_f->gcs_message);
}
