/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Eric Perko, Chad Rockey
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "../include/xv_11_driver/xv11_laser.h"
#include <chrono>

static unsigned long nowMillis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>
           (std::chrono::steady_clock::now().time_since_epoch()).count();
}

namespace xv_11_driver
{
	XV11Laser::XV11Laser(const std::string &port, uint32_t baud_rate, uint32_t firmware, boost::asio::io_service &io) : port_(port),
																														baud_rate_(baud_rate), firmware_(firmware), shutting_down_(false), serial_(io, port_)
	{
		serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

	}


	bool XV11Laser::IsValidPacket(uint8_t *packet, int packetLength)
	{
		unsigned long chk32 = 0;
		unsigned long checksum;
		const int bytesToCheck = packetLength - 2;
		const int calcCRCLen = bytesToCheck / 2;
		unsigned int calcCRC[calcCRCLen];

		for (int ix = 0; ix < calcCRCLen; ix++)
			calcCRC[ix] = 0;

		// Build 'calcCRC' array: ghép cặp byte thành một số 16 bit
		for (int ix = 0; ix < bytesToCheck; ix += 2)
			calcCRC[ix / 2] = packet[ix] + (packet[ix + 1] << 8);

		for (int ix = 0; ix < calcCRCLen; ix++)
			chk32 = (chk32 << 1) + calcCRC[ix];

		checksum = ((chk32 & 0x7FFF) + (chk32 >> 15)) & 0x7FFF;
		uint8_t b1a = checksum & 0xFF;
		uint8_t b2a = checksum >> 8;
		uint8_t b1b = packet[OFFSET_TO_CRC_L];
		uint8_t b2b = packet[OFFSET_TO_CRC_M];

		return (b1a == b1b && b2a == b2b);
	}

	void XV11Laser::poll(sensor_msgs::msg::LaserScan *scan)
	{
		uint8_t temp_char;
		uint8_t start_count = 0;
		bool got_scan = false;

		if (firmware_ == 1)
		{ // This is for the old driver, the one that only outputs speed once per revolution
			boost::array<uint8_t, 1440> raw_bytes;
			while (!shutting_down_ && !got_scan)
			{
				// Wait until the start sequence 0x5A, 0xA5, 0x00, 0xC0 comes around
				boost::asio::read(serial_, boost::asio::buffer(&temp_char, 1));
				if (start_count == 0)
				{
					if (temp_char == 0x5A)
					{
						start_count = 1;
					}
				}
				else if (start_count == 1)
				{
					if (temp_char == 0xA5)
					{
						start_count = 2;
					}
				}
				else if (start_count == 2)
				{
					if (temp_char == 0x00)
					{
						start_count = 3;
					}
				}
				else if (start_count == 3)
				{
					if (temp_char == 0xC0)
					{
						start_count = 0;
						// Now that entire start sequence has been found, read in the rest of the message
						got_scan = true;
						// Now read speed
						boost::asio::read(serial_, boost::asio::buffer(&motor_speed_, 2));

						// Read in 360*4 = 1440 chars for each point
						boost::asio::read(serial_, boost::asio::buffer(&raw_bytes, 1440));

						scan->angle_min = 0.0;
						scan->angle_max = 2.0 * M_PI;
						scan->angle_increment = (2.0 * M_PI / 360.0);
						scan->time_increment = motor_speed_ / 1e8;
						scan->range_min = 0.06;
						scan->range_max = 5.0;
						scan->ranges.reserve(360);
						scan->intensities.reserve(360);

						for (uint16_t i = 0; i < raw_bytes.size(); i = i + 4)
						{
							// Four bytes per reading
							uint8_t byte0 = raw_bytes[i];
							uint8_t byte1 = raw_bytes[i + 1];
							uint8_t byte2 = raw_bytes[i + 2];
							uint8_t byte3 = raw_bytes[i + 3];
							// First two bits of byte1 are status flags
							// uint8_t flag1 = (byte1 & 0x80) >> 7;  // No return/max range/too low of reflectivity
							// uint8_t flag2 = (byte1 & 0x40) >> 6;  // Object too close, possible poor reading due to proximity kicks in at < 0.6m
							// Remaining bits are the range in mm
							uint16_t range = ((byte1 & 0x3F) << 8) + byte0;
							// Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
							uint16_t intensity = (byte3 << 8) + byte2;

							scan->ranges.push_back(range / 1000.0);
							scan->intensities.push_back(intensity);
						}
					}
				}
			}
		}
		else if (firmware_ == 2)
		{ // This is for the newer driver that outputs packets 4 pings at a time
			boost::array<uint8_t, 1980> raw_bytes;
			uint8_t good_sets = 0;
			uint32_t motor_speed = 0;
			rpms = 0;
			int index;
			while (!shutting_down_ && !got_scan)
			{
				// Wait until first data sync of frame: 0xFA, 0xA0
				boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count], 1));
				if (start_count == 0)
				{
					if (raw_bytes[start_count] == 0xFA)
					{
						start_count = 1;
					}
				}
				else if (start_count == 1)
				{
					if (raw_bytes[start_count] == 0xA0)
					{
						start_count = 0;

						// Now that entire start sequence has been found, read in the rest of the message
						got_scan = true;

						boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[2], 1978));

						scan->angle_min = 0.0;
						scan->angle_max = 2.0 * M_PI;
						scan->angle_increment = (2.0 * M_PI / 360.0);
						scan->range_min = 0.06;
						scan->range_max = 5.0;
						scan->ranges.resize(360);
						scan->intensities.resize(360);

						// read data in sets of 4
						for (uint16_t i = 0; i < raw_bytes.size(); i = i + 22)
						{
							if (raw_bytes[i] == 0xFA && raw_bytes[i + 1] == (0xA0 + i / 22) /*&& IsValidPacket(&raw_bytes[i], PACKET_LENGTH)*/)
							{ //&& CRC check
								good_sets++;
								motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2]; // accumulate count for avg. time increment
								rpms = (raw_bytes[i + 3] << 8 | raw_bytes[i + 2]) / 64;
								// measured_rpm = static_cast<float>(rpms);

								for (uint16_t j = i + 4; j < i + 20; j = j + 4)
								{
									index = (4 * i) / 22 + (j - 4 - i) / 4;
									// Four bytes per reading
									uint8_t byte0 = raw_bytes[j];
									uint8_t byte1 = raw_bytes[j + 1];
									uint8_t byte2 = raw_bytes[j + 2];
									uint8_t byte3 = raw_bytes[j + 3];
									// First two bits of byte1 are status flags
									// uint8_t flag1 = (byte1 & 0x80) >> 7;  // No return/max range/too low of reflectivity
									// uint8_t flag2 = (byte1 & 0x40) >> 6;  // Object too close, possible poor reading due to proximity kicks in at < 0.6m
									// Remaining bits are the range in mm
									uint16_t range = ((byte1 & 0x3F) << 8) + byte0;
									// Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
									uint16_t intensity = (byte3 << 8) + byte2;

									scan->ranges[index] = range / 1000.0;
									scan->intensities[index] = intensity;
								}
							}
						}

						scan->time_increment = motor_speed / good_sets / 1e8;   // Tính toán thời gian giữa các điểm quét liên tiếp trong một vòng quét laser
					}
				}
			}
		}
	}


}
