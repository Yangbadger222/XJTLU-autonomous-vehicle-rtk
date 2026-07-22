# Constellation values follow the UM982 channel tracking status field.
uint8 CONSTELLATION_GPS=0
uint8 CONSTELLATION_GLONASS=1
uint8 CONSTELLATION_SBAS=2
uint8 CONSTELLATION_GALILEO=3
uint8 CONSTELLATION_BDS=4
uint8 CONSTELLATION_QZSS=5
uint8 CONSTELLATION_IRNSS=6

uint8 constellation
uint16 prn
uint8 signal_type
uint8 channel_number

# Raw UM982 System Freq field. For GLONASS, channel = system_frequency - 7.
uint16 system_frequency
int16 glonass_frequency_channel

float64 pseudorange_m
float64 carrier_phase_cycles
float64 doppler_hz
float64 pseudorange_std_m
float64 carrier_phase_std_cycles
float64 cn0_db_hz
float64 lock_time_s

bool pseudorange_valid
bool carrier_phase_valid
bool l2c_signal
uint32 tracking_status
