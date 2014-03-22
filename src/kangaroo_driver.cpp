#include "kangaroo_driver/kangaroo_driver.hpp"
#include "kangaroo_driver/kang_lib.hpp"

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>

#include <iostream>

//namespace kangaroo
//{

kangaroo::kangaroo( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv ) :
	port( "" ),
	ch1_joint_name( "1" ),
	ch2_joint_name( "2" ),
	fd( -1 ),
	nh( _nh ),
	nh_priv( _nh_priv )
{
	ROS_INFO( "Initializing" );
	nh_priv.param( "port", port, (std::string)"/dev/ttyUSB0" );
	nh_priv.param( "ch1_joint_name", ch1_joint_name, (std::string)"1" );
	nh_priv.param( "ch2_joint_name", ch2_joint_name, (std::string)"2" );
}

bool kangaroo::open()
{
	struct termios fd_options;

	if( is_open() )
	{
		ROS_INFO( "Port is already open - Closing to re-open" );
		close();
	}

	fd = ::open( port.c_str( ), O_RDWR | O_NOCTTY | O_NDELAY );
	if( fd < 0 )
	{
		ROS_FATAL( "Failed to open port: %s", strerror( errno ) );
		return false;
	}
	if( 0 > fcntl( fd, F_SETFL, 0 ) )
	{
		ROS_FATAL( "Failed to set port descriptor: %s", strerror( errno ) );
		return false;
	}
	if( 0 > tcgetattr( fd, &fd_options ) )
	{
		ROS_FATAL( "Failed to fetch port attributes: %s", strerror( errno ) );
		return false;
	}
	if( 0 > cfsetispeed( &fd_options, B9600) )
	{
		ROS_FATAL( "Failed to set input baud: %s", strerror( errno ) );
		return false;
	}
	if( 0 > cfsetospeed( &fd_options, B9600 ) )
	{
		ROS_FATAL( "Failed to set output baud: %s", strerror( errno ) );
		return false;
	}
	fd_options.c_cflag &= ~HUPCL;
	fd_options.c_lflag &= ~ICANON;
	fd_options.c_cc[VTIME] = 1;
	fd_options.c_cc[VMIN] = 0;
	if( 0 > tcsetattr( fd, TCSANOW, &fd_options ) )
	{
		ROS_FATAL( "Failed to set port attributes: %s", strerror( errno ) );
		return false;
	}

	return true;
}

void kangaroo::close( )
{
	ROS_INFO( "Closing Port" );

	::close( fd );
}

bool kangaroo::start( )
{
	if( !is_open() && !open() )
		return false;
	ROS_INFO( "Starting" );
	if( !joint_traj_sub )
		joint_traj_sub = nh.subscribe( "joint_trajectory", 1, &kangaroo::JointTrajCB, this );
	if( !joint_state_pub)
		joint_state_pub = nh.advertise<sensor_msgs::JointState>( "joint_state", 2);

	send_start_signals((unsigned char)128);
	return true;
}

void kangaroo::stop( )
{
	ROS_INFO( "Stopping" );

	if( joint_traj_sub )
		joint_traj_sub.shutdown( );

	close( );
}

bool kangaroo::is_open( ) const
{
	return ( fd >= 0 );
}

void kangaroo::JointTrajCB(const trajectory_msgs::JointTrajectoryPtr &msg)
{
	int ch1_idx = -1;
	int ch2_idx = -1;

	for (size_t i = 0; i < msg->joint_names.size(); i++)
	{
		if (msg->joint_names[i] == ch1_joint_name)
			ch1_idx = i;
		if (msg->joint_names[i] == ch2_joint_name)
			ch2_idx = i;
	}

	if (0 > ch1_idx && 0 > ch2_idx)
	{
		ROS_WARN("Got a JointTrajectory message with no valid joints");
		return;
	}

	if (1 > msg->points.size())
	{
		ROS_WARN("Got a JointTrajectory message with no valid points");
		return;
	}

	if (msg->joint_names.size() != msg->points[0].velocities.size())
	{
		ROS_WARN("Got a JointTrajectory message whose points have no velocities");
		return;
	}

	ROS_WARN("Got a valid JointTrajectory message");

	tcflush(fd, TCOFLUSH);

	boost::mutex::scoped_lock io_lock(io_mutex);
	set_channel_speed(msg->points[0].velocities[ch1_idx], 128, '1');
	set_channel_speed(msg->points[0].velocities[ch2_idx], 128, '2');
}

bool kangaroo::send_start_signals(unsigned char address)
{
	unsigned char buffer[7];
	int num_of_bytes_1 = write_kangaroo_start_command(address, '1', buffer);
	if (0 > write(fd, buffer, num_of_bytes_1))
	{
		ROS_ERROR("Failed to update channel 1: %s", strerror(errno));
		close();
		return false;
	}

	int num_of_bytes_2 = write_kangaroo_start_command(address, '2', buffer);
	if (0 > write(fd, buffer, num_of_bytes_2))
	{
		ROS_ERROR("Failed to update channel 2: %s", strerror(errno));
		close();
		return false;
	}

	return true;
}

bool kangaroo::set_channel_speed(double speed, unsigned char address, char channel)
{
	std::cout << "Sending " << speed << " to Channel " << channel << std::endl;

	if (!is_open() && !open())
		return false;

	// lock the mutex, since we will be writing to the serial port
	boost::mutex::scoped_lock(io_mutex);

	unsigned char buffer[18];

	//Send
	int num_of_bytes = write_kangaroo_speed_command(address, channel, speed, buffer);
	if (0 > write(fd, buffer, num_of_bytes))
	{
		ROS_ERROR("Failed to update channel %c: %s", channel, strerror(errno));
		close();
		return false;
	}
	return true;
}

bool kangaroo::send_get_request(unsigned char address, char channel, unsigned char desired_parameter)
{
	if (!is_open() && !open())
		return false;

	unsigned char buffer[18];

	if (!(desired_parameter == 1 || desired_parameter == 2))
	{
		ROS_ERROR("Invalid parameter for get request.");
		return false;
	}

	// lock the mutex, since we will be writing to the serial port
	boost::mutex::scoped_lock(io_mutex);

	int num_of_bytes = write_kangaroo_get_command(address, channel, desired_parameter, buffer);
	if (0 > write(fd, buffer, num_of_bytes))
	{
		ROS_ERROR("Failed to write to the thingy.");
		close();
		return false;
	}

	return true;
}

void kangaroo::read_thread()
{
	while (ros::ok)
	{
		unsigned char byte;
		if (0 > read(fd, &byte, 1))
			ROS_WARN("An error occurred while reading. - Main read thread.");
		if (byte == (unsigned char)128)
			read_message(byte);
	}
}

bool kangaroo::read_message(unsigned char address)
{
	// note: we aren't guaranteed the exact number of bytes that the
	//   kangaroo will respond with, so we must read the header and 
	//   data separately
	bool one_byte_ok = true;
	unsigned char header[3] = { 0 };
	unsigned char data[8] = { 0 }; 	// 1 for channel name, 1 for flags, 
	// 1 for parameter, 5 (max) for bit-packed value
	unsigned char crc[2] = { 0 };		// crc is 2 bytes long

	header[0] = address;
	for (size_t i = 1; i < 3; i++)
	{
		header[i] = read_one_byte(one_byte_ok);
		// bail if reading one byte fails
		if (!one_byte_ok)
		{
			ROS_ERROR("Error when reading header from kangaroo.");
			//ok = false;
			return false;
		}
	}
	std::cout << "Header: " << (int)header[0] << ", " << (int)header[1] << ", " << (int)header[2] << std::endl;

	// Read the data
	for (size_t i = 0; i < header[2]; i++)
	{
		data[i] = read_one_byte(one_byte_ok);
		// bail if reading one byte fails
		if (!one_byte_ok)
		{
			ROS_ERROR("Error when reading *data* from kangaroo.");
			//ok = false;
			return 0;
		}
	}
	// print the data ( DEBUG ONLY)
	std::cout << "Data: ";
	for (size_t i = 0; i < header[2]; i++)
		std::cout << (int)data[i] << ", ";
	std::wcout << std::endl;
	
	// read the error correcting code
	for (size_t i = 0; i < 2; i++)
	{
		crc[i] = read_one_byte(one_byte_ok);
		// bail if reading one byte fails
		if (!one_byte_ok)
		{
			ROS_ERROR("Error when reading *data* from kangaroo.");
			//ok = false;
			return 0;
		}
	}
	// Print the CRC (Debug only)
	std::cout << "CRC: " << (int)crc[0] << ", " << (int)crc[1] << std::endl;

	bool ok = true;
	evaluate_kangaroo_response(address, header, data, ok);
	return ok;
}

unsigned char kangaroo::read_one_byte(bool& ok)
{
	unsigned char buffer;

	int bits_read = read(fd, &buffer, 1);
	if (bits_read > 0)
		return buffer;
	if (bits_read < 0)
	{
		ROS_ERROR("Reading from serial critically failed: %s", strerror(errno));
		close();
		ok = false;
		return 0;
	}
	
	if (bits_read == 0)
	{
		// try to read the byte again if it fails
		bits_read = read(fd, &buffer, 1);
		if (bits_read < 0)
		{
			ROS_ERROR("Reading from serial critically failed: %s", strerror(errno));
			close();
			ok = false;
			return 0;
		}
		if (bits_read == 0)
		{
			ROS_ERROR("Failed to read from the serial.");
			ok = false;
			return 0;
		}
	}

	return buffer;
}

int kangaroo::evaluate_kangaroo_response( unsigned char address, unsigned char* header, unsigned char* data, bool& ok)
{
	ROS_INFO( "Evaluating the response." );
	size_t data_size = header[2];
	size_t value_size = data_size - 3;      // there are 3 bits that aren't the value
	size_t value_offset = 3;	// offset of the value in data
	//bool echo_code = false;

	if( data[1] & (1 << 5 ))	// testing the flag to see if there is an echo code
	{
		value_size--;	// if there is an echo code, then there will be 1 extra byte
		value_offset++;
		//echo_code = true;
		ROS_INFO("There was an echo code.");
	}

	int value = un_bitpack_number(&data[value_offset], value_size);
	std::cout << "The value is: " << value << std::endl;
	if( data[1] & 1 )	// testing the flag to see if there is an error
	{
		handle_errors(address, value);
		ok = false;
		return 0;
	}

	if(data[1] & (1 << 1) )
		ROS_WARN( "The controller is in motion." );

	sensor_msgs::JointStatePtr msg(new sensor_msgs::JointState);
	msg->name.resize(1);
	if(data[0] == '1')
		msg->name[0] = ch1_joint_name;
	else
		msg->name[0] = ch2_joint_name;

	if(data[2] == 1)
	{
		msg->position.resize(1);
		msg->position[0] = value;
	}
	else
	{
		msg->velocity.resize(1);
		msg->position[0] = value;
	}

	joint_state_pub.publish(msg);
	return value;
}

void kangaroo::handle_errors(unsigned char address, int error_code)
{
	switch (error_code)
	{
	case 1:
		send_start_signals(address);
		ROS_ERROR("Channels have not been started, attempted to start them.");
		break;
	case 2:
		ROS_ERROR("Channel needs to be homed.");
		break;
	case 3:
		send_start_signals(address);
		ROS_ERROR("Control error occurred. Sent start signals to clear.");
		break;
	case 4:
		ROS_ERROR("Controller is in the wrong mode.  Change the switch.");
		break;
	case 5:
		ROS_ERROR("The given parameter is unknown.");
		break;
	case 6:
		ROS_ERROR("Serial timeout occurred.");
		break;
	}
	return;
}

//}

//void kangaroo::poll_kangaroo(unsigned char address)
//{
//	sensor_msgs::JointStatePtr msg(new sensor_msgs::JointState);
//	msg->name.resize(2);
//	msg->velocity.resize(2);
//	msg->position.resize(2);
//	msg->name[0] = ch1_joint_name;
//	msg->name[1] = ch2_joint_name;
//
//	// set the lock
//	boost::mutex::scoped_lock io_lock(io_mutex);
//
//	// flush the input buffer
//	tcflush(fd, TCIFLUSH);
//	tcflush(fd, TCOFLUSH);
//	// construct the message
//	bool ok = true;
//	// channel 1
//	msg->velocity[0] = get_speed(address, '1', ok);
//	if (!ok)
//		return;
//
//	tcflush(fd, TCIFLUSH);
//	tcflush(fd, TCOFLUSH);
//	msg->position[0] = get_position(address, '1', ok);
//	if (!ok)
//		return;
//
//	// channel 2
//	tcflush(fd, TCIFLUSH);
//	tcflush(fd, TCOFLUSH);
//	msg->velocity[1] = get_speed(address, '2', ok);
//	if (!ok)
//		return;
//	tcflush(fd, TCIFLUSH);
//	tcflush(fd, TCOFLUSH);
//	msg->position[1] = get_position(address, '2', ok);
//	if (!ok)
//		return;
//
//	joint_state_pub.publish(msg);
//	//delete msg;
//}

//int kangaroo::get_speed(unsigned char address, char channel, bool& ok)
//{
//	ok = send_get_request(address, channel, 2);		// 2 is for velocity
//	if (!ok)
//		return 0;
//	tcflush(fd, TCOFLUSH);	// TODO: verify
//	return read_from_serial(address, channel, ok);
//}

//int kangaroo::get_position(unsigned char address, char channel, bool& ok)
//{
//	ok = send_get_request(address, channel, 1);		// 1 is for position
//	if (!ok)
//		return 0;
//	tcflush(fd, TCOFLUSH);	// TODO: verify
//	return read_from_serial(address, channel, ok);
//}

//int kangaroo::read_from_serial(unsigned char address, char channel, bool& ok)
//{
//	// note: we aren't guaranteed the exact number of bytes that the
//	//   kangaroo will respond with, so we must read the header and 
//	//   data separately
//	bool one_byte_ok = true;
//	unsigned char header[3] = { 0 };
//	unsigned char data[10] = { 0 }; // 1 for channel name, 1 for flags, 
//				// 1 for parameter, 5 (max) for bit-packed value, 2 for crc
//
//	for (size_t i = 0; i < 3; i++)
//	{
//		header[i] = read_one_byte(one_byte_ok);
//		// bail if reading one byte fails
//		if (!one_byte_ok)
//		{
//			ROS_ERROR("Error when reading header from kangaroo.");
//			ok = false;
//			return 0;
//		}
//	}
//	std::cout << "Header: " << (int)header[0] << ", " << (int)header[1] << ", " << (int)header[2] << std::endl;
//
//	for (size_t i = 0; i < header[2] + 2; i++)
//	{
//		data[i] = read_one_byte(one_byte_ok);
//		// bail if reading one byte fails
//		if (!one_byte_ok)
//		{
//			ROS_ERROR("Error when reading *data* from kangaroo.");
//			ok = false;
//			return 0;
//		}
//	}
//
//	// evaluate the read data
//	bool evaluate_ok = true;
//	int value = evaluate_kangaroo_response(address, header, data, evaluate_ok);
//	ok = evaluate_ok;
//	return value;
//}
