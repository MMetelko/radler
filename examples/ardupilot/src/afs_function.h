#include RADL_HEADER

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "mavros_msgs/msg/gpsraw.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "mavlink/v2.0/common/mavlink.h"

#include <iostream>
#include <string>

using namespace std;

// Pulled from mavlink/v2.0/common/common.hpp
// enum class FENCE_BREACH : uint8_t
// {
//     NONE = 0,     /* No last fence breach | */
//     MINALT = 1,   /* Breached minimum altitude | */
//     MAXALT = 2,   /* Breached maximum altitude | */
//     BOUNDARY = 3, /* Breached fence boundary | */
// };

/** Type of GPS fix */
// enum class GPS_FIX_TYPE : uint8_t
// {
//     NO_GPS=0, /* No GPS connected | */
//     NO_FIX=1, /* No position information, GPS is connected | */
//     TYPE_2D_FIX=2, /* 2D position | */
//     TYPE_3D_FIX=3, /* 3D position | */
//     DGPS=4, /* DGPS/SBAS aided 3D position | */
//     RTK_FLOAT=5, /* RTK float, 3D position | */
//     RTK_FIXED=6, /* RTK Fixed, 3D position | */
//     STATIC=7, /* Static fixed, typically used for base stations | */
//     PPP=8, /* PPP, 3D position. | */
// };


class AFS_Function
{
public:
  AFS_Function();
 	void step(const radl_in_t* i, const radl_in_flags_t* i_f, radl_out_t* o, radl_out_flags_t* o_f);

private:
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
