#include RADL_HEADER

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "mavros_msgs/msg/gpsraw.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "mavlink/v2.0/common/mavlink.h"

#include <iostream>
#include <sstream>
#include <string>
#include <chrono>
#include <ctime>
#include <builtin_interfaces/msg/time.hpp>

using namespace std;


class AFS_Function
{
    public:
    AFS_Function();
        void step(const radl_in_t* i, const radl_in_flags_t* i_f, radl_out_t* o, radl_out_flags_t* o_f);

    private:
    std::string formatTimestamp(const builtin_interfaces::msg::Time& stamp)
    std::shared_ptr<rclcpp::Node> node;

    double battery_remaining_percentage;
        uint gps_fix_type;
    int gps_loss_count;
    double current_gps_loss_duration;
    rclcpp::Time previous_gps_loss_time;
    string gps_fix_state;
    string afs_state;
    uint breach_status;
    uint breach_type;
    string max_altitude_breach_event;
    double current_max_altitude_breach_duration;
    rclcpp::Time previous_max_altitude_breach_time;
};
