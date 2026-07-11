uint8 RECEIVER_MASTER=1
uint8 RECEIVER_SECONDARY=2
uint8 RECEIVER_BASE=3

# Reception time remains diagnostic; week/TOW is the observation time.
std_msgs/Header header
string source_port
uint64 frame_index
uint64 stream_offset

uint8 receiver
uint16 source_message_id
uint8 time_reference
uint8 time_status
uint16 week
uint32 milliseconds_of_week
uint16 output_delay_ms

Observation[] observations
