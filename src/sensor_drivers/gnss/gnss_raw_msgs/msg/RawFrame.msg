# Reception time is diagnostic only. GNSS observation time comes from week/TOW below.
std_msgs/Header header

# Physical source and monotonic stream identity.
string source_port
uint64 frame_index
uint64 stream_offset

# Parsed Unicore 24-byte binary header.
uint8 cpu_idle_percent
uint16 message_id
uint16 payload_length
uint8 time_reference
uint8 time_status
uint16 week
uint32 milliseconds_of_week
uint32 format_version
uint8 leap_seconds
uint16 output_delay_ms

# Phase 1 publishes only checksum-valid frames, but keeps this explicit in recorded data.
bool crc_valid

# Complete binary frame: 24-byte header, payload, and little-endian 32-bit CRC.
uint8[] data
