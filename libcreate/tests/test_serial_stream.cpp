/**
Software License Agreement (BSD)

\file      test_serial_stream.cpp
\authors   Jacob Perron <jperron@sfu.ca>
\copyright Copyright (c) 2018, Autonomy Lab (Simon Fraser University), All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of Autonomy Lab nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
#include "create/data.h"
#include "create/serial_stream.h"

#include "gtest/gtest.h"

#include <memory>
#include <utility>
#include <vector>

namespace {

class ByteSerialStream : public create::SerialStream {
public:
  explicit ByteSerialStream(const std::shared_ptr<create::Data>& data)
    : create::SerialStream(data, create::util::STREAM_HEADER, false)
  {
    // Initialize the requested frame length without connecting to hardware.
    startSensorStream();
  }

  using create::SerialStream::processByte;

  void feed(const std::vector<uint8_t>& bytes)
  {
    for (const auto byte : bytes) processByte(byte);
  }
};

// A complete V3 stream response: header, payload size, every requested packet,
// and a checksum making the unsigned byte sum zero modulo 256.
std::vector<uint8_t> sensorFrame(create::Data& data, uint16_t left, uint16_t right)
{
  std::vector<uint8_t> frame = {
    create::util::STREAM_HEADER,
    static_cast<uint8_t>(data.getTotalDataBytes() + data.getNumPackets())
  };
  for (const auto id : data.getPacketIDs()) {
    frame.push_back(id);
    const uint16_t value = id == create::ID_LEFT_ENC ? left :
                          id == create::ID_RIGHT_ENC ? right :
                          static_cast<uint16_t>((id << 8) | (255 - id));
    if (data.getPacket(id)->nbytes == 2) frame.push_back(value >> 8);
    frame.push_back(value & 0xff);
  }
  unsigned int sum = 0;
  for (const auto byte : frame) sum += byte;
  frame.push_back(static_cast<uint8_t>(0u - sum));
  return frame;
}

}  // namespace

TEST(SerialStreamTest, FullV3FrameDecodesBothEncodersAfterChecksum)
{
  auto data = std::make_shared<create::Data>(create::V_3);
  ByteSerialStream stream(data);
  const auto frame = sensorFrame(*data, 0x12a5, 0xbc34);
  for (size_t i = 0; i + 1 < frame.size(); ++i) stream.processByte(frame[i]);
  EXPECT_EQ(data->getPacket(create::ID_LEFT_ENC)->getData(), 0);
  EXPECT_EQ(data->getPacket(create::ID_RIGHT_ENC)->getData(), 0);

  stream.processByte(frame.back());
  EXPECT_EQ(data->getPacket(create::ID_LEFT_ENC)->getData(), 0x12a5);
  EXPECT_EQ(data->getPacket(create::ID_RIGHT_ENC)->getData(), 0xbc34);
  EXPECT_EQ(stream.getTotalPackets(), 1);
  EXPECT_EQ(stream.getNumCorruptPackets(), 0);
}

TEST(SerialStreamTest, EncoderProgressionAndWrapRemainIndependent)
{
  auto data = std::make_shared<create::Data>(create::V_3);
  ByteSerialStream stream(data);
  const std::vector<std::pair<uint16_t, uint16_t>> samples = {
    {0xfff0, 0x8005}, {0xfffe, 0x8001}, {0x0003, 0x7ffc}, {0x0108, 0x0000}
  };
  for (const auto& sample : samples) {
    stream.feed(sensorFrame(*data, sample.first, sample.second));
    EXPECT_EQ(data->getPacket(create::ID_LEFT_ENC)->getData(), sample.first);
    EXPECT_EQ(data->getPacket(create::ID_RIGHT_ENC)->getData(), sample.second);
  }
  EXPECT_EQ(stream.getTotalPackets(), samples.size());
  EXPECT_EQ(stream.getNumCorruptPackets(), 0);
}

TEST(SerialStreamTest, CapturedRoomba675FrameDecodesBothEncoders)
{
  auto data = std::make_shared<create::Data>(create::V_3);
  ByteSerialStream stream(data);
  // Exact frame captured from the robot, independent of the frame generator.
  stream.feed({
    0x13, 0x48, 0x07, 0x00, 0x08, 0x00, 0x09, 0x00, 0x0a, 0x00,
    0x0b, 0x00, 0x0c, 0x00, 0x0d, 0x00, 0x0e, 0x00, 0x0f, 0x00,
    0x11, 0x00, 0x12, 0x00, 0x15, 0x00, 0x16, 0x3c, 0x6c, 0x17,
    0xff, 0x99, 0x18, 0x11, 0x19, 0x06, 0xf3, 0x1a, 0x08, 0x14,
    0x23, 0x02, 0x2b, 0x00, 0x3f, 0x2c, 0x67, 0xcc, 0x2d, 0x00,
    0x2e, 0x00, 0x01, 0x2f, 0x00, 0x01, 0x30, 0x00, 0x0a, 0x31,
    0x00, 0x01, 0x32, 0x00, 0x04, 0x33, 0x00, 0x00, 0x34, 0x00,
    0x35, 0x00, 0x3a, 0x02, 0x38
  });
  EXPECT_EQ(data->getPacket(create::ID_LEFT_ENC)->getData(), 63);
  EXPECT_EQ(data->getPacket(create::ID_RIGHT_ENC)->getData(), 26572);
  EXPECT_EQ(stream.getTotalPackets(), 1);
  EXPECT_EQ(stream.getNumCorruptPackets(), 0);
}

TEST(SerialStreamTest, CorruptFrameKeepsBothEncodersAndNextFrameResynchronizes)
{
  auto data = std::make_shared<create::Data>(create::V_3);
  ByteSerialStream stream(data);
  stream.feed(sensorFrame(*data, 0x1234, 0x5678));

  auto corrupt = sensorFrame(*data, 0xabcd, 0xef01);
  corrupt.back() ^= 1;
  stream.feed(corrupt);
  EXPECT_EQ(data->getPacket(create::ID_LEFT_ENC)->getData(), 0x1234);
  EXPECT_EQ(data->getPacket(create::ID_RIGHT_ENC)->getData(), 0x5678);
  EXPECT_EQ(stream.getNumCorruptPackets(), 1);

  // Noise and a header with an invalid payload length precede a complete frame.
  stream.feed({0xaa, 0x00, create::util::STREAM_HEADER, 0x00});
  stream.feed(sensorFrame(*data, 0x2345, 0x6789));
  EXPECT_EQ(data->getPacket(create::ID_LEFT_ENC)->getData(), 0x2345);
  EXPECT_EQ(data->getPacket(create::ID_RIGHT_ENC)->getData(), 0x6789);
  EXPECT_EQ(stream.getTotalPackets(), 3);
  EXPECT_EQ(stream.getNumCorruptPackets(), 1);
}

TEST(SerialStreamTest, Constructor)
{
  std::shared_ptr<create::Data> data_ptr = std::make_shared<create::Data>();
  create::SerialStream serial_stream(data_ptr);
}

TEST(SerialStreamTest, Connected)
{
  std::shared_ptr<create::Data> data_ptr = std::make_shared<create::Data>();
  create::SerialStream serial_stream(data_ptr);

  // Did not call connect and nothing to connect to, so expect false
  EXPECT_FALSE(serial_stream.connected());
}

TEST(SerialStreamTest, Disconnect)
{
  std::shared_ptr<create::Data> data_ptr = std::make_shared<create::Data>();
  create::SerialStream serial_stream(data_ptr);

  // Not connected, but should not fail
  serial_stream.disconnect();
}

TEST(SerialStreamTest, NumPackets)
{
  std::shared_ptr<create::Data> data_ptr = std::make_shared<create::Data>();
  create::SerialStream serial_stream(data_ptr);

  // Not connected, so zero packets should have been received
  EXPECT_EQ(serial_stream.getNumCorruptPackets(), 0);
  EXPECT_EQ(serial_stream.getTotalPackets(), 0);
}

TEST(SerialStreamTest, Send)
{
  std::shared_ptr<create::Data> data_ptr = std::make_shared<create::Data>();
  create::SerialStream serial_stream(data_ptr);

  // Some bytes to send (to set date)
  uint8_t bytes[4] = { create::OC_DATE, 0, 1, 2 };
  // Not connected, so failure expected
  EXPECT_FALSE(serial_stream.send(bytes, 4));
}

TEST(SerialStreamTest, SendOpcode)
{
  std::shared_ptr<create::Data> data_ptr = std::make_shared<create::Data>();
  create::SerialStream serial_stream(data_ptr);

  // Not connected, so failure expected
  EXPECT_FALSE(serial_stream.sendOpcode(create::OC_POWER));
}
