#include "um982_rtk_driver/ntrip_response.hpp"

#include <sstream>

namespace um982_rtk_driver
{
namespace
{

std::string firstLine(const std::string & response)
{
  const auto line_end = response.find('\n');
  std::string line = response.substr(0, line_end);
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  return line;
}

bool startsWith(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

}  // namespace

NtripResponseState evaluateNtripResponse(const std::string & response)
{
  const auto line = firstLine(response);
  if (line.empty()) {
    return NtripResponseState::NeedMore;
  }

  if (startsWith(line, "ICY 200")) {
    return NtripResponseState::Accepted;
  }
  if (startsWith(line, "HTTP/")) {
    const bool has_success_code =
      line.find(" 200 ") != std::string::npos ||
      (line.size() >= 12 && line.substr(9, 3) == "200");
    if (has_success_code) {
      if (response.find("\r\n\r\n") == std::string::npos && response.find("\n\n") == std::string::npos) {
        return NtripResponseState::NeedMore;
      }
      return NtripResponseState::Accepted;
    }
    return NtripResponseState::Rejected;
  }

  if (response.find('\n') == std::string::npos) {
    return NtripResponseState::NeedMore;
  }
  return NtripResponseState::Rejected;
}

}  // namespace um982_rtk_driver
