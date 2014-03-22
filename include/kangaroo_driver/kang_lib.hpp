#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <iostream>

size_t bitpack_number(unsigned char* buffer, int number);
int un_bitpack_number(unsigned char* data, size_t num_of_bytes);
uint16_t crc14(const unsigned char* data, size_t length);
size_t write_kangaroo_position_command(unsigned char address, char channel, int position, int speedLimit, unsigned char* buffer);
size_t write_kangaroo_speed_command(unsigned char address, char channel, int speed, unsigned char* buffer);
size_t write_kangaroo_start_command(unsigned char address, char channel, unsigned char* buffer);
size_t write_kangaroo_get_command(unsigned char address, char channel, char parameter, unsigned char* buffer);
size_t write_kangaroo_command(unsigned char address, unsigned char command, const unsigned char* data, unsigned char length, unsigned char* buffer);

