# Host-side timing for an accepted /cmd_ctbr command. ROS may assign its own
# sequence to header; command_seq preserves the source CTBR sequence explicitly.
# The copied header stamp remains the source command stamp for time correlation.
std_msgs/Header header
uint32 command_seq
float64 command_age_s
# ROS time from the command stamp to entry into the server callback.
float64 callback_duration_s
float64 send_duration_s
uint16 raw_thrust
bool command_accepted
bool unlock_packet
