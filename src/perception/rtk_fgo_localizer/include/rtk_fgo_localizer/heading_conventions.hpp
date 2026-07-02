#pragma once

namespace rtk_fgo_localizer
{

double compassHeadingDegToEnuYawRad(double heading_deg);

double headingQuaternionYawToEnuYaw(double quaternion_yaw_rad, bool quaternion_yaw_is_compass);

}  // namespace rtk_fgo_localizer
