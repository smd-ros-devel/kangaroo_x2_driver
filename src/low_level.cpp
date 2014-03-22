#include "kangaroo_driver/kangaroo_driver.hpp"

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>

#include <iostream>

namespace kangaroo
{

bool kangaroo::set_channel_speed(double speed, unsigned char address, char channel)
{
	std::cout << "Sending " << speed << " to Channel " << channel << std::endl;

	//boost::mutex::scoped_lock(io_mutex);
	if (!is_open() && !open())
		return false;

	uint8_t buffer[18];

	//Send
	int num_of_bytes = writeKangarooSpeedCommand(address, channel, speed, buffer);
	if (0 > write(fd, buffer, num_of_bytes))
	{
		ROS_ERROR("Failed to update channel %c: %s", channel, strerror(errno));
		close();
		return false;
	}
	return true;
}

bool kangaroo::send_get_request(unsigned char address, char channel, uint8_t desired_parameter)
{
	if (!is_open() && !open())
		return false;

	uint8_t buffer[18];

	if (!(desired_parameter == 1 || desired_parameter == 2))
	{
		ROS_ERROR("Invalid parameter for get request.");
		return false;
	}

	int num_of_bytes = writeKangarooGetCommand(address, channel, desired_parameter, buffer);
	if (0 > write(fd, buffer, num_of_bytes))
	{
		ROS_ERROR("Failed to write to the thingy.");
		close();
		return false;
	}

	return true;
}

/*! Bit-packs a number.
* \param buffer The buffer to write into.
* \param number The number to bit-pack. Should be between -(2^29-1) and 2^29-1.
* \return How many bytes were written (1 to 5). */
size_t kangaroo::bitpackNumber(uint8_t* buffer, int32_t number)
{
	size_t i = 0;
	if (number < 0)
	{
		number = -number;
		number <<= 1;
		number |= 1;
	}
	else
	{
		number <<= 1;
	}
	while (i < 5)
	{
		buffer[i++] = (number & 0x3f) | (number >= 0x40 ? 0x40 : 0x00);
		number >>= 6;
		if (!number)
		{
			break;
		}
	}
	return i;
}

int kangaroo::un_bitpack_number(uint8_t* data, size_t num_of_bytes)
{
        int bit_packed_number = 0;
        for (size_t i = 0; i < num_of_bytes; i++)
        {
                int temp = data[i] & 0x3f;
                bit_packed_number += (temp << (i * 6));
        }

        if (bit_packed_number % 2 == 1)
        {
                bit_packed_number--;
                bit_packed_number = -bit_packed_number;
        }
        bit_packed_number = bit_packed_number >> 1;
        return bit_packed_number;
}


/*! Computes a 14-bit CRC.
* \param data The buffer to compute the CRC of.
* \param length The length of the data.
* \return The CRC. */
uint16_t kangaroo::crc14(const uint8_t* data, size_t length)
{
	uint16_t crc = 0x3fff;
	for (size_t i = 0; i < length; i++)
	{
		crc ^= data[i] & 0x7f;
		for (size_t bit = 0; bit < 7; bit++)
		{
			if (crc & 1)
			{
				crc >>= 1;
				crc ^= 0x22f0;
			}
			else
			{
				crc >>= 1;
			}
		}
	}
	return crc ^ 0x3fff;
}

/*! Writes a Packet Serial command into a buffer.
\param address The address of the Kangaroo. By default, this is 128.
\param command The command number
\param data The command data.
\param length The number of bytes of data.
\param buffer The buffer to write into.
\return How many bytes were written. This always equals 4 + length. */
size_t kangaroo::writeKangarooCommand(uint8_t address, uint8_t command, const uint8_t* data, uint8_t length, uint8_t* buffer)
{
	uint16_t crc;
	buffer[0] = address;
	buffer[1] = command;
	buffer[2] = length;
	for (size_t i = 0; i < length; i++)
	{
		buffer[3 + i] = data[i];
	}
	crc = crc14(buffer, length + 3);
	buffer[3 + length] = crc & 0x7f;
	buffer[4 + length] = (crc >> 7) & 0x7f;

	return 5 + length;
}

/*! Writes a Move command for Position into a buffer.
* This could have many, many more options, but I've kept it basic
* to make the example easier to read.
* \param address The address of the Kangaroo. By default, this is 128.
* \param channel The channel name.
* By default, for Independent Mode, these are '1' and '2'.
* For Mixed Mode, these are 'D' and 'T'.
* \param position The position to go to.
* \param speedLimit The speed limit to use. Negative numbers use the default.
* \param buffer The buffer to write into.
* \return How many bytes were written (at most 18). */
size_t kangaroo::writeKangarooPositionCommand(uint8_t address, char channel, int32_t position, int32_t speedLimit, uint8_t* buffer)
{
	uint8_t data[14];
	size_t length = 0;
	data[length] = (uint8_t)channel;
	length++;
	data[length] = 0;  // move flags
	length++;
	data[length] = 1;  // Position
	length++;
	length += bitpackNumber(&data[length], position);
	if (speedLimit >= 0)
	{
		data[length++] = 2; // Speed (Limit if combined with Position)
		length += bitpackNumber(&data[length], speedLimit);
	}
	return writeKangarooCommand(address, 36, data, length, buffer);
}

size_t kangaroo::writeKangarooSpeedCommand(uint8_t address, char channel, int32_t speed, uint8_t* buffer)
{
	uint8_t data[9] = { 0 };
	size_t length = 0;
	data[length] = (uint8_t)channel;
	length++;
	data[length] = 0;  // move flags
	length++;
	data[length] = 2;  // Speed
	length++;
	length += bitpackNumber(&data[length], speed);
	return writeKangarooCommand(address, 36, data, length, buffer);
}

size_t kangaroo::writeKangarooStartCommand(uint8_t address, char channel, uint8_t* buffer)
{
	uint8_t data[2] = { 0 };
	size_t length = 0;
	data[length] = channel;
	length++;
	data[length] = 0;  // flags
	length++;
	return writeKangarooCommand(address, 32, data, length, buffer);
}

size_t kangaroo::writeKangarooGetCommand(uint8_t address, char channel, char parameter, uint8_t* buffer)
{
	uint8_t data[3];
	size_t length = 0;
	data[length++] = (uint8_t)channel;
	data[length++] = 0;
	data[length++] = parameter;
	return writeKangarooCommand(address, 35, data, length, buffer);
}

} // end namespace kangaroo
