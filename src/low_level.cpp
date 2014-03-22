// A C library for accessing a kangaroo controller

//#include "kangaroo_driver/kangaroo_driver.hpp"

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>

#include <iostream>

size_t bitpack_number(unsigned char* buffer, int number);
int un_bitpack_number(unsigned char* data, size_t num_of_bytes);
uint16_t crc14(const unsigned char* data, size_t length);
size_t write_kangaroo_position_command(unsigned char address, char channel, int position, int speedLimit, unsigned char* buffer);
size_t write_kangaroo_speed_command(unsigned char address, char channel, int speed, unsigned char* buffer);
size_t write_kangaroo_start_command(unsigned char address, char channel, unsigned char* buffer);
size_t write_kangaroo_get_command(unsigned char address, char channel, char parameter, unsigned char* buffer);
size_t write_kangaroo_command(unsigned char address, unsigned char command, const unsigned char* data, unsigned char length, unsigned char* buffer);


/*! Bit-packs a number.
* \param buffer The buffer to write into.
* \param number The number to bit-pack. Should be between -(2^29-1) and 2^29-1.
* \return How many bytes were written (1 to 5). */
size_t bitpackNumber(unsigned char* buffer, int number)
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

int un_bitpack_number(unsigned char* data, size_t num_of_bytes)
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
uint16_t crc14(const unsigned char* data, size_t length)
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
size_t write_kangaroo_position_command(unsigned char address, char channel, int position, int speedLimit, unsigned char* buffer)
{
	unsigned char data[14];
	size_t length = 0;
	data[length] = (unsigned char)channel;
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
	return write_kangaroo_command(address, 36, data, length, buffer);
}

size_t write_kangaroo_speed_command(unsigned char address, char channel, int speed, unsigned char* buffer)
{
	unsigned char data[9] = { 0 };
	size_t length = 0;
	data[length] = (unsigned char)channel;
	length++;
	data[length] = 0;  // move flags
	length++;
	data[length] = 2;  // Speed
	length++;
	length += bitpackNumber(&data[length], speed);
	return write_kangaroo_command(address, 36, data, length, buffer);
}

size_t write_kangaroo_start_command(unsigned char address, char channel, unsigned char* buffer)
{
	unsigned char data[2] = { 0 };
	size_t length = 0;
	data[length] = channel;
	length++;
	data[length] = 0;  // flags
	length++;
	return write_kangaroo_command(address, 32, data, length, buffer);
}

size_t write_kangaroo_get_command(unsigned char address, char channel, char parameter, unsigned char* buffer)
{
	unsigned char data[3];
	size_t length = 0;
	data[length++] = (unsigned char)channel;
	data[length++] = 0;
	data[length++] = parameter;
	return write_kangaroo_command(address, 35, data, length, buffer);
}

/*! Writes a Packet Serial command into a buffer.
\param address The address of the Kangaroo. By default, this is 128.
\param command The command number
\param data The command data.
\param length The number of bytes of data.
\param buffer The buffer to write into.
\return How many bytes were written. This always equals 4 + length. */
size_t write_kangaroo_command(unsigned char address, unsigned char command, const unsigned char* data, unsigned char length, unsigned char* buffer)
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