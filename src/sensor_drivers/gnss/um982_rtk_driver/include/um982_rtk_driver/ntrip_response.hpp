#pragma once

#include <string>

namespace um982_rtk_driver
{

enum class NtripResponseState
{
  NeedMore,
  Accepted,
  Rejected,
};

NtripResponseState evaluateNtripResponse(const std::string & response);

}  // namespace um982_rtk_driver
