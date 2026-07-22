# RTCM 3 type 1005/1006 reference-station antenna reference point.
std_msgs/Header header
string source
uint64 frame_index
uint16 message_type
uint16 station_id
uint8 itrf_realization
bool gps_indicator
bool glonass_indicator
bool galileo_indicator
bool reference_station_indicator
float64[3] position_ecef_m
bool antenna_height_valid
float64 antenna_height_m
