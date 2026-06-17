#pragma once

#include <optional>
#include <string>
#include <vector>

namespace um982_rtk_driver
{

struct GgaData
{
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double mean_sea_level = 0.0;
  double hdop = 0.0;
  int fix_quality = 0;
  int satellites = 0;
  std::string utc_time;
};

struct RmcData
{
  bool valid = false;
  double speed_mps = 0.0;
  double course_deg = 0.0;
  std::string utc_time;
};

struct ThsData
{
  double heading_deg = 0.0;
  std::string mode;
};

struct HprData
{
  std::string utc_time;
  double heading_deg = 0.0;
  double pitch_deg = 0.0;
  double roll_deg = 0.0;
  int fix_type = 0;
  int satellites = 0;
  double differential_age = 0.0;
  std::string station_id;
};

struct UniHeadingData
{
  std::string solution_status;
  std::string position_type;
  double baseline_length = 0.0;
  double heading_deg = 0.0;
  double pitch_deg = 0.0;
  double heading_stddev = 0.0;
  double pitch_stddev = 0.0;
  std::string station_id;
  int satellites = 0;
  int solution_satellites = 0;
};

struct ParsedSentence
{
  std::string talker;
  std::string type;
  std::optional<GgaData> gga;
  std::optional<RmcData> rmc;
  std::optional<ThsData> ths;
  std::optional<HprData> hpr;
  std::optional<UniHeadingData> uniheading;
};

bool hasValidChecksum(const std::string & sentence);
std::optional<ParsedSentence> parseSentence(const std::string & sentence);
std::string buildNmeaSentence(const std::string & body);
std::string fixQualityText(int fix_quality);
double normalizeHeadingDeg(double heading_deg);
double applyHeadingOffsetDeg(double heading_deg, double offset_deg);

}  // namespace um982_rtk_driver
