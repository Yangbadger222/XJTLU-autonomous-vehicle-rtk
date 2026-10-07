# Acquisition-stamped, atomic evidence identity. No consumer may assign the
# currently received map version to an older grid from another DDS topic.
std_msgs/Header header
string map_version
string localization_session_id
string support_model
nav_msgs/OccupancyGrid grid
