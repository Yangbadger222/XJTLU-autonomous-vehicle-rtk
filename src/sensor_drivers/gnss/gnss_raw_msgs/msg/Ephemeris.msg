uint8 CONSTELLATION_GPS=0
uint8 CONSTELLATION_GLONASS=1
uint8 CONSTELLATION_GALILEO=3
uint8 CONSTELLATION_BDS=4
uint8 CONSTELLATION_QZSS=5

uint8 MODEL_KEPLERIAN=1
uint8 MODEL_GLONASS=2

uint8 REFERENCE_FRAME_WGS84=1
uint8 REFERENCE_FRAME_PZ90_02=2
uint8 REFERENCE_FRAME_CGCS2000=3
uint8 REFERENCE_FRAME_GTRF=4
uint8 REFERENCE_FRAME_JGS=5

# Reception time is diagnostic. The broadcast reference time is week/toe below.
std_msgs/Header header
string source_port
uint64 frame_index
uint64 stream_offset
uint16 source_message_id
uint8 time_reference
uint8 time_status
uint16 header_week
uint32 header_milliseconds_of_week

uint8 constellation
uint16 prn
uint8 model
uint8 reference_frame
uint32 health
uint32 issue_of_data_ephemeris
uint32 issue_of_data_clock
uint32 week
float64 toe_s
float64 toc_s

# Broadcast Keplerian model (GPS, QZSS, BDS, Galileo).
float64 semi_major_axis_m
float64 delta_mean_motion_rad_s
float64 mean_anomaly_rad
float64 eccentricity
float64 argument_of_perigee_rad
float64 cuc_rad
float64 cus_rad
float64 crc_m
float64 crs_m
float64 cic_rad
float64 cis_rad
float64 inclination_rad
float64 inclination_rate_rad_s
float64 ascending_node_rad
float64 ascending_node_rate_rad_s
float64 clock_bias_s
float64 clock_drift_s_s
float64 clock_drift_rate_s_s2
float64 group_delay_1_s
float64 group_delay_2_s
bool group_delay_1_valid
bool group_delay_2_valid
float64 corrected_mean_motion_rad_s
float64 ura_variance_m2
bool ura_variance_valid
uint32 accuracy_index

# GLONASS state-vector model. PRN is normalized to slot + 37.
int16 glonass_frequency_channel
float64[3] position_ecef_m
float64[3] velocity_ecef_m_s
float64[3] acceleration_ecef_m_s2
float64 glonass_clock_bias_s
float64 glonass_relative_frequency_bias
float64 glonass_l1_l2_delay_s
uint32 glonass_frame_time_s
uint32 glonass_flags
