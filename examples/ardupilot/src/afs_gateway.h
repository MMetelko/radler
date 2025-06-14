#include RADL_HEADER

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "mavros_msgs/msg/mavlink.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/waypoint_list.hpp"
#include "mavros_msgs/msg/gpsraw.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "mavlink/v2.0/common/mavlink.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <chrono>
#include <ctime>
#include <cmath>
#include <builtin_interfaces/msg/time.hpp>

using namespace std;


class AFS_Gateway
{
    public:
        AFS_Gateway();
        void step(const radl_in_t* i, const radl_in_flags_t* i_f, radl_out_t* o, radl_out_flags_t* o_f);
    private:
        // Debug Message structure
        struct DebugInfo {
            std::string mavlink_gps_info;
            std::string mavlink_fs_info;
            std::string error_msgs;
        };

        // Define a struct to hold all status information
        struct StatusInfo {
            std::string battery_status;
            std::string autopilot_mode;
            std::string copter_command;
            std::string current_waypoint;
            std::string global_position;
            std::string gps_status;
            std::string geofence_status;            
            std::string diagnostics; 
            DebugInfo debug_data;
        };

        struct GeofenceBreach {
            rclcpp::Time timestamp;
            uint8_t breach_type;
            uint32_t breach_count;
        };

        std::shared_ptr<rclcpp::Node> node;

        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr mavros_battery_subscriber;
        sensor_msgs::msg::BatteryState::ConstSharedPtr battery_status_mailbox;
        void mavros_battery_state_callback(const sensor_msgs::msg::BatteryState::ConstSharedPtr bs);

        rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr mavlink_from_subscriber;
        mavlink_fence_status_t geofence_status_mailbox;
        rclcpp::Time geofence_status_timestamp;
        bool geofence_status_available;
        bool geofence_breach_detected;
        rclcpp::Time last_breach_time;
        const std::chrono::seconds BREACH_MEMORY_DURATION;
        std::vector<GeofenceBreach> recent_breaches;
        const size_t MAX_BREACH_HISTORY;
        unsigned long total_mavlink_messages;
        unsigned long fence_status_messages;
        unsigned long gps_status_messages;

        mavlink_global_position_int_t globalposition_status_mailbox;
        rclcpp::Time global_position_timestamp;
        bool global_position_status_available;
        void mavlink_callback(const mavros_msgs::msg::Mavlink::ConstSharedPtr fs);
        std::string formatWaypointCoordinate(double value, double minValue, double maxValue);
        bool isValidCoordinate(double value, double minValue, double maxValue);

        rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr mavros_gpsraw_subscriber;
        mavros_msgs::msg::GPSRAW::ConstSharedPtr gps_status_mailbox;
        void mavros_gps_status_callback(const mavros_msgs::msg::GPSRAW::ConstSharedPtr gs);

        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr flight_controls_mode;

        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_autopilotstate_subscriber;
        mavros_msgs::msg::State::ConstSharedPtr autopilotstate_status_mailbox;
        void mavros_autopilotstate_callback(const mavros_msgs::msg::State::ConstSharedPtr aps);

        rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr mavros_missionwaypoints_subscriber;
        mavros_msgs::msg::WaypointList::ConstSharedPtr missionwaypoints_status_mailbox;
        void mavros_missionwaypoints_callback(const mavros_msgs::msg::WaypointList::ConstSharedPtr mws);

        //rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr mavros_globalposition_subscriber;
        //sensor_msgs::msg::NavSatFix::ConstSharedPtr globalposition_status_mailbox;
        //void mavros_globalposition_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr gps);

        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr mavros_diagnostics_subscriber;
        diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diagnostics_status_mailbox;
        void mavros_diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr das);
        std::string formatTimestamp(const builtin_interfaces::msg::Time& stamp);

        rclcpp::Time previous_diagnostics_status_time;
        int previous_diagnostics_heartbeat_value;
        int current_diagnostics_heartbeat_value;
        double elapsed_diagnostics_status_duration;
        double current_heartbeat_loss_duration;

        int previous_flight_controls_cmd_id;
        StatusInfo currentStatus;
};