#include "um982_rtk_driver/nmea_parser.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace um982_rtk_driver
{
namespace
{

std::string trim(const std::string & input)
{
  const auto begin = std::find_if_not(
    input.begin(), input.end(), [](unsigned char c) {
      return std::isspace(c) != 0;
    });
  const auto end = std::find_if_not(
    input.rbegin(), input.rend(), [](unsigned char c) {
      return std::isspace(c) != 0;
    }).base();
  if (begin >= end) {
    return "";
  }
  return std::string(begin, end);
}

bool isHex(char c)
{
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

int hexValue(char c)
{
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  return 0;
}

std::vector<std::string> split(const std::string & input, char delimiter)
{
  std::vector<std::string> fields;
  std::string current;
  std::istringstream stream(input);
  while (std::getline(stream, current, delimiter)) {
    fields.push_back(current);
  }
  if (!input.empty() && input.back() == delimiter) {
    fields.emplace_back();
  }
  return fields;
}

double nan()
{
  return std::numeric_limits<double>::quiet_NaN();
}

double safeDouble(const std::vector<std::string> & fields, std::size_t index)
{
  if (index >= fields.size() || fields[index].empty()) {
    return nan();
  }
  try {
    return std::stod(fields[index]);
  } catch (const std::exception &) {
    return nan();
  }
}

int safeInt(const std::vector<std::string> & fields, std::size_t index)
{
  if (index >= fields.size() || fields[index].empty()) {
    return 0;
  }
  try {
    return std::stoi(fields[index]);
  } catch (const std::exception &) {
    return 0;
  }
}

std::string safeString(const std::vector<std::string> & fields, std::size_t index)
{
  if (index >= fields.size()) {
    return "";
  }
  return fields[index];
}

double parseCoordinate(const std::string & value, const std::string & hemisphere)
{
  if (value.empty() || hemisphere.empty()) {
    return nan();
  }

  const auto dot = value.find('.');
  if (dot == std::string::npos || dot < 2) {
    return nan();
  }

  const auto degrees_length = dot - 2;
  try {
    const double degrees = std::stod(value.substr(0, degrees_length));
    const double minutes = std::stod(value.substr(degrees_length));
    double coordinate = degrees + minutes / 60.0;
    if (hemisphere == "S" || hemisphere == "W") {
      coordinate = -coordinate;
    }
    return coordinate;
  } catch (const std::exception &) {
    return nan();
  }
}

bool supportedTalker(const std::string & talker)
{
  static const std::vector<std::string> talkers = {
    "GP", "GN", "GL", "GA", "GB", "BD", "GQ", "IN"};
  return std::find(talkers.begin(), talkers.end(), talker) != talkers.end();
}

std::optional<ParsedSentence> parseGga(
  const std::string & talker,
  const std::vector<std::string> & fields)
{
  if (fields.size() < 12) {
    return std::nullopt;
  }

  ParsedSentence parsed;
  parsed.talker = talker;
  parsed.type = "GGA";
  GgaData data;
  data.utc_time = safeString(fields, 1);
  data.latitude = parseCoordinate(safeString(fields, 2), safeString(fields, 3));
  data.longitude = parseCoordinate(safeString(fields, 4), safeString(fields, 5));
  data.fix_quality = safeInt(fields, 6);
  data.satellites = safeInt(fields, 7);
  data.hdop = safeDouble(fields, 8);
  data.altitude = safeDouble(fields, 9);
  data.mean_sea_level = safeDouble(fields, 11);
  parsed.gga = data;
  return parsed;
}

std::optional<ParsedSentence> parseRmc(
  const std::string & talker,
  const std::vector<std::string> & fields)
{
  if (fields.size() < 9) {
    return std::nullopt;
  }

  ParsedSentence parsed;
  parsed.talker = talker;
  parsed.type = "RMC";
  RmcData data;
  data.utc_time = safeString(fields, 1);
  data.valid = safeString(fields, 2) == "A";
  data.speed_mps = safeDouble(fields, 7) * 0.514444444444;
  data.course_deg = safeDouble(fields, 8);
  parsed.rmc = data;
  return parsed;
}

std::optional<ParsedSentence> parseThs(
  const std::string & talker,
  const std::vector<std::string> & fields)
{
  if (fields.size() < 3) {
    return std::nullopt;
  }

  ParsedSentence parsed;
  parsed.talker = talker;
  parsed.type = "THS";
  ThsData data;
  data.heading_deg = safeDouble(fields, 1);
  data.mode = safeString(fields, 2);
  parsed.ths = data;
  return parsed;
}

std::optional<ParsedSentence> parseHpr(
  const std::string & talker,
  const std::vector<std::string> & fields)
{
  if (fields.size() < 9) {
    return std::nullopt;
  }

  ParsedSentence parsed;
  parsed.talker = talker;
  parsed.type = "HPR";
  HprData data;
  data.utc_time = safeString(fields, 1);
  data.heading_deg = safeDouble(fields, 2);
  data.pitch_deg = safeDouble(fields, 3);
  data.roll_deg = safeDouble(fields, 4);
  data.fix_type = safeInt(fields, 5);
  data.satellites = safeInt(fields, 6);
  data.differential_age = safeDouble(fields, 7);
  data.station_id = safeString(fields, 8);
  parsed.hpr = data;
  return parsed;
}

std::optional<ParsedSentence> parseUniHeading(const std::string & sentence)
{
  const std::string clean = trim(sentence);
  if (clean.rfind("#UNIHEADINGA,", 0) != 0) {
    return std::nullopt;
  }

  const auto semicolon = clean.find(';');
  const auto star = clean.find('*', semicolon == std::string::npos ? 0 : semicolon);
  if (semicolon == std::string::npos || star == std::string::npos || star <= semicolon + 1) {
    return std::nullopt;
  }

  const std::string payload = clean.substr(semicolon + 1, star - semicolon - 1);
  const auto fields = split(payload, ',');
  if (fields.size() < 10) {
    return std::nullopt;
  }

  ParsedSentence parsed;
  parsed.talker = "UN";
  parsed.type = "UNIHEADING";
  UniHeadingData data;
  data.solution_status = safeString(fields, 0);
  data.position_type = safeString(fields, 1);
  data.baseline_length = safeDouble(fields, 2);
  data.heading_deg = safeDouble(fields, 3);
  data.pitch_deg = safeDouble(fields, 4);
  data.heading_stddev = safeDouble(fields, 6);
  data.pitch_stddev = safeDouble(fields, 7);
  data.station_id = safeString(fields, 8);
  data.satellites = safeInt(fields, 9);
  data.solution_satellites = safeInt(fields, 10);
  parsed.uniheading = data;
  return parsed;
}

}  // namespace

bool hasValidChecksum(const std::string & sentence)
{
  const std::string clean = trim(sentence);
  const auto star = clean.find('*');
  if (clean.size() < 4 || clean.front() != '$' || star == std::string::npos ||
    star + 2 >= clean.size())
  {
    return false;
  }
  if (!isHex(clean[star + 1]) || !isHex(clean[star + 2])) {
    return false;
  }

  unsigned char checksum = 0;
  for (std::size_t i = 1; i < star; ++i) {
    checksum ^= static_cast<unsigned char>(clean[i]);
  }

  const unsigned char expected = static_cast<unsigned char>(
    hexValue(clean[star + 1]) * 16 + hexValue(clean[star + 2]));
  return checksum == expected;
}

std::optional<ParsedSentence> parseSentence(const std::string & sentence)
{
  const std::string clean = trim(sentence);
  if (!clean.empty() && clean.front() == '#') {
    return parseUniHeading(clean);
  }

  if (!hasValidChecksum(clean)) {
    return std::nullopt;
  }

  const auto star = clean.find('*');
  const std::string body = clean.substr(1, star - 1);
  const auto fields = split(body, ',');
  if (fields.empty() || fields[0].size() < 5) {
    return std::nullopt;
  }

  const std::string talker = fields[0].substr(0, 2);
  const std::string type = fields[0].substr(2);
  if (!supportedTalker(talker)) {
    return std::nullopt;
  }

  if (type == "GGA") {
    return parseGga(talker, fields);
  }
  if (type == "RMC") {
    return parseRmc(talker, fields);
  }
  if (type == "THS") {
    return parseThs(talker, fields);
  }
  if (type == "HPR") {
    return parseHpr(talker, fields);
  }
  return std::nullopt;
}

std::string buildNmeaSentence(const std::string & body)
{
  unsigned char checksum = 0;
  for (const char c : body) {
    checksum ^= static_cast<unsigned char>(c);
  }
  std::ostringstream stream;
  stream << "$" << body << "*" << std::uppercase << std::hex << std::setw(2) <<
    std::setfill('0') << static_cast<int>(checksum);
  return stream.str();
}

std::string fixQualityText(int fix_quality)
{
  switch (fix_quality) {
    case 0:
      return "No Fix";
    case 1:
      return "Single";
    case 2:
      return "DGPS";
    case 4:
      return "RTK Fixed";
    case 5:
      return "RTK Float";
    case 9:
      return "WAAS";
    default:
      return "Q" + std::to_string(fix_quality);
  }
}

}  // namespace um982_rtk_driver
