#include <gtest/gtest.h>

#include "um982_rtk_driver/ntrip_response.hpp"

using um982_rtk_driver::NtripResponseState;
using um982_rtk_driver::evaluateNtripResponse;

TEST(NtripResponse, AcceptsIcy200HeaderWithoutBlankLine)
{
  EXPECT_EQ(evaluateNtripResponse("ICY 200 OK\r\n"), NtripResponseState::Accepted);
}

TEST(NtripResponse, WaitsForIcyLineTerminator)
{
  EXPECT_EQ(evaluateNtripResponse("ICY 200"), NtripResponseState::NeedMore);
}

TEST(NtripResponse, AcceptsHttp200Header)
{
  EXPECT_EQ(
    evaluateNtripResponse("HTTP/1.1 200 OK\r\nServer: NTRIP\r\n\r\n"),
    NtripResponseState::Accepted);
}

TEST(NtripResponse, RejectsNonSuccessHeader)
{
  EXPECT_EQ(
    evaluateNtripResponse("SOURCETABLE 200 OK\r\n\r\n"),
    NtripResponseState::Rejected);
}
