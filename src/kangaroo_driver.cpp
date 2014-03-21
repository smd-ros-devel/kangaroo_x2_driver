#include "kangaroo_driver/kangaroo_driver.hpp"

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>

#include <iostream>

namespace kangaroo
{

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

bool kangaroo::open( )
{
	struct termios fd_options;
	//unsigned char baud_autodetect = 0xAA;

	if( is_open( ) )
	{
		ROS_INFO( "Port is already open - Closing to re-open" );
		close( );
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
	if( !is_open( ) && !open( ) )
		return false;

	ROS_INFO( "Starting" );

	if( !joint_traj_sub )
		joint_traj_sub = nh.subscribe( "joint_trajectory", 1, &kangaroo::JointTrajCB, this );

	if( !joint_state_pub)
		joint_state_pub = nh.advertise<sensor_msgs::JointState>( "joint_state", 2);

	send_start_signals((uint8_t)128);

	send_speed_request( (uint8_t)128, '1');

	return true;
}

bool kangaroo::send_start_signals(uint8_t address)
{
	uint8_t buffer[7];
	int num_of_bytes_1 = writeKangarooStartCommand(address, '1', buffer);
        if( 0 > write( fd, buffer, num_of_bytes_1 ) )
        {
                ROS_ERROR( "Failed to update channel 1: %s", strerror( errno ) );
                close( );
                return false;
        }

	int num_of_bytes_2 = writeKangarooStartCommand(address, '2', buffer);
        if( 0 > write( fd, buffer, num_of_bytes_2 ) )
        {
                ROS_ERROR( "Failed to update channel 2: %s", strerror( errno ) );
                close( );
                return false;
        }

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

void kangaroo::JointTrajCB( const trajectory_msgs::JointTrajectoryPtr &msg )
{
	int ch1_idx = -1;
	int ch2_idx = -1;

	for( size_t i = 0; i < msg->joint_names.size( ); i++ )
	{
		if( msg->joint_names[i] == ch1_joint_name )
			ch1_idx = i;
		if( msg->joint_names[i] == ch2_joint_name )
			ch2_idx = i;
	}

	if( 0 > ch1_idx && 0 > ch2_idx )
	{
		ROS_WARN( "Got a JointTrajectory message with no valid joints" );
		return;
	}

	if( 1 > msg->points.size( ) )
	{
		ROS_WARN( "Got a JointTrajectory message with no valid points" );
		return;
	}

	if( msg->joint_names.size( ) != msg->points[0].velocities.size( ) )
	{
		ROS_WARN( "Got a JointTrajectory message whose points have no velocities" );
		return;
	}

	ROS_WARN( "Got a valid JointTrajectory message" );

	set_ch1( msg->points[0].velocities[ch1_idx], 128 );
	set_ch2( msg->points[0].velocities[ch2_idx], 128 );
	send_speed_request( 128, '1');	
}

bool kangaroo::set_ch1( double speed, unsigned char address = 128 )
{
	std::cout << "Sending " << speed << " to Channel 1" << std::endl;

	boost::mutex::scoped_lock(io_mutex);
	if( !is_open( ) && !open( ) )
		return false;

	uint8_t buffer[18];

	//Send
	int num_of_bytes = writeKangarooSpeedCommand(address, '1', speed, buffer);
	if( 0 > write( fd, buffer, num_of_bytes ) )
	{
		ROS_ERROR( "Failed to update channel 1: %s", strerror( errno ) );
		close( );
		return false;
	}
	return true;
}


bool kangaroo::set_ch2( double speed, unsigned char address = 128 )
{
	std::cout << "Sending " << speed << " to Channel 2" << std::endl;
	boost::mutex::scoped_lock(io_mutex);
	if( !is_open( ) && !open( ) )
		return false;

	uint8_t buffer[18];

	//Send
	int num_of_bytes = writeKangarooSpeedCommand(address, '2', speed, buffer);
	if( 0 > write( fd, buffer, num_of_bytes ) )
	{
		ROS_ERROR( "Failed to update channel 2: %s", strerror( errno ) );
		close( );
		return false;
	}

	return true;
}

void kangaroo::poll_kangaroo( unsigned char address )
{
	// note: we aren't guaranteed the exact number of bytes that the
	//   kangaroo will respond with, so we must read the header and 
	//   data separately
	uint8_t header[3] = {0};
	uint8_t data[10] = {0};	// 1 for channel name, 1 for flags, 
				// 1 for parameter, 5 (max) for bit-packed value, 2 for crc

	//ROS_INFO( "Constructing JointStatePtr msg." );
	sensor_msgs::JointStatePtr msg(new sensor_msgs::JointState);
	msg->name.resize(2);
	msg->name[0] = ch1_joint_name;
	msg->name[1] = ch2_joint_name;
	
	msg->velocity.resize(2);
	msg->position.resize(2);
	//ROS_INFO( "Finished Construction." );

	// set the lock
	boost::mutex::scoped_lock io_lock( io_mutex );

	// flush the input buffer
	tcflush( fd, TCIFLUSH );	

	// -------------- channel 1 ---------------
	// ***************Position*****************
	
	// send the request for the position of channel 1
	if (send_speed_request( address, '1' ) == false)
        {
                ROS_ERROR("Error when sending data to kangaroo.");
                return;
        }	
	// evaluate the data that the kangaroo sent
	if( read(fd, header, 3) != 3 )
	{
		ROS_ERROR("Error when reading response from kangaroo.");
		return;
	}
	if( 0 > read(fd, data, header[1]) )
	{
		ROS_ERROR("Error when reading response from kangaroo.");
		return;
	}
	// evaluate the response received from the kangaroo
	if( evaluate_kangaroo_response( header, data, address, msg, '1', 1 ) == false )
	{
		ROS_ERROR("Bad information received from kangaroo.");
		return;
	}

	// ***************Speed*****************
	// send the request for the speed of channel 1
	if (send_speed_request( address, '1' ) == false)
        {
                ROS_ERROR("Error when sending data to kangaroo.");
                return;
        }
	// read the data that the kangaroo sent
	if( read(fd, header, 3) != 3 )
        {
                ROS_ERROR("Error when reading response from kangaroo.");
                return;
        }
        if( 0 > read(fd, data, header[1]) )
        {
                ROS_ERROR("Error when reading response from kangaroo.");
                return;
        }
	// evaluate the response received from the kangaroo
	if( evaluate_kangaroo_response( header, data, address, msg, '1', 2 ) == false )
        {
                ROS_ERROR("Bad information received from kangaroo.");
                return;
        }

	// --------------Channel 2-----------------
	// ***************Position*****************

	// send the request for the position of channel 2
	if (send_speed_request( address, '2' ) == false)
        {
                ROS_ERROR("Error when sending data to kangaroo.");
                return;
        }
	// read the data that the kangaroo sent
	if( read(fd, header, 3) != 3 )
        {
                ROS_ERROR("Error when reading response from kangaroo.");
                return;
        }
        if( 0 > read(fd, data, header[1]) )
        {
                ROS_ERROR("Error when reading response from kangaroo.");
                return;
        }
	// evaluate the read data
	if( evaluate_kangaroo_response( header, data, address, msg, '2', 1 ) == false )
        {
                ROS_ERROR("Bad information received from kangaroo.");
                return;
        }

	// ***************Speed*****************
	// send the request for the speed of channel 2
	if (send_speed_request( address, '2' ) == false)
	{
		ROS_ERROR("Error when sending data to kangaroo.");
		return;
	}
	// read the data that the kangaroo sent
	if( read(fd, header, 3) != 3 )
        {
                ROS_ERROR("Error when reading response from kangaroo.");
                return;
        }
        if( 0 > read(fd, data, header[1]) )
        {
                ROS_ERROR("Error when reading response from kangaroo.");
                return;
        }
	// evaluate the received data
	if( evaluate_kangaroo_response( header, data, address, msg, '2', 2 ) == false )
        {
                ROS_ERROR("Bad information received from kangaroo.");
                return;
        }

	joint_state_pub.publish(msg);
}

//bool read_speed( uint8_t address, char channel )
//{
//
//}

//bool read_position( uint8_t address, char channel )
//{
//
//}

//bool read_from_serial( uint8_t address, 

int kangaroo::un_bitpack_number( uint8_t* data, size_t num_of_bytes )
{
	int bit_packed_number = 0;
	for( size_t i = 0; i < num_of_bytes; i++ )
	{
		bit_packed_number += (data[i] << (i * 6));
	}

	if( bit_packed_number % 2 == 1 )
	{
		bit_packed_number--;
		bit_packed_number = -bit_packed_number;
	}
	bit_packed_number >> 2;
	return bit_packed_number;
}

bool kangaroo::evaluate_kangaroo_response( uint8_t* header, uint8_t* data, 
	uint8_t address, sensor_msgs::JointStatePtr& msg, char channel, uint8_t parameter)
{
	ROS_INFO( "Evaluating the response." );
	size_t data_size = header[1];
	size_t value_size = data_size - 5;      // there are 5 bits that aren't the value	
	bool echo_code = false;

	if( data[1] & (1 << 5 ))	// testing the flag to see if there is an echo code
	{
		value_size--;	// if there is an echo code, then there will be 1 extra byte
		echo_code = true;
	}

	int value;
	if(echo_code)
                value = un_bitpack_number(&data[5], value_size);
        else
                value = un_bitpack_number(&data[4], value_size);


	if( data[1] & 1 )	// testing the flag to see if there is an error
	{
		int error_code = value;
		//if(echo_code)
		//	error_code = un_bitpack_number(&data[5], value_size);
		//else
		//	error_code = un_bitpack_number(&data[4], value_size);
		
		if(error_code == 1)
		{
			send_start_signals( address );
			ROS_ERROR( "Channels have not been started, attempted to start them." );
			return false;
		}
		if(error_code == 2)
		{
			ROS_ERROR( "Channel needs to be homed." );
			return false;
		}
		if(error_code == 3)
		{
			send_start_signals( address );
			ROS_ERROR( "Control error occurred. Sent start signals to clear." );
			return false;
		}
		if(error_code == 4)
		{
			ROS_ERROR( "Controller is in the wrong mode.  Change the switch." );
			return false;
		}
		if(error_code == 5)
		{
			ROS_ERROR( "The given parameter is unknown." );
			return false;
		}
		if(error_code == 6)
		{
			ROS_ERROR( "Serial timeout occurred." );
			return false;
		}
	}

	if(data[1] & (1 << 1) )
	{
		ROS_WARN( "The controller is in motion." );
	}

	if(echo_code)
        	value = un_bitpack_number(&data[5], value_size);
       	else
          	value = un_bitpack_number(&data[4], value_size);

	

	// update the message
	int channel_index;
	if( channel == '1')
		channel_index = 0;
	else if (channel == '2')
		channel_index = 1;
	else
	{
		ROS_ERROR("Invalid channel.");
		return false;
	}

	if( parameter == 1 ) // 1 is position
		msg->position[channel_index] = value;
	else if ( parameter == 2 ) // 2 is velocity
		msg->velocity[channel_index] = value;
	else
	{
		ROS_ERROR("Invalid parameter");
		return false;
	}
	return true;
}

bool kangaroo::send_position_request( unsigned char address, char channel )
{
	return send_get_request( address, channel, 1 );
}

bool kangaroo::send_speed_request( unsigned char address, char channel )
{
	return send_get_request( address, channel, 2 );
}

bool kangaroo::send_get_request( unsigned char address, char channel, uint8_t desired_parameter )
{
	if( !is_open( ) && !open() )
		return false;

	uint8_t buffer[18];
	
	if( !(desired_parameter == 1 || desired_parameter == 2))
	{
		ROS_ERROR( "Invalid parameter for get request." );
		return false;
	}
	
	int num_of_bytes = writeKangarooGetCommand(address, channel, desired_parameter, buffer);
	if( 0 > write( fd, buffer, num_of_bytes) )
	{
		ROS_ERROR( "Failed to write to the thingy." );
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
		buffer[i ++] = (number & 0x3f) | (number >= 0x40 ? 0x40 : 0x00);
		number >>= 6;
		if (!number) 
		{
			break; 
		}
	}
	return i;
}

/*! Computes a 14-bit CRC.
 * \param data The buffer to compute the CRC of.
 * \param length The length of the data.
 * \return The CRC. */
uint16_t kangaroo::crc14( const uint8_t* data, size_t length)
{
	uint16_t crc = 0x3fff;
	for(size_t i = 0; i < length; i ++)
	{
		crc ^= data[i] & 0x7f;
		for(size_t bit = 0; bit < 7; bit ++)
		{
			if(crc & 1) 
			{ 
				crc >>= 1; 
				crc ^= 0x22f0; 
			}
			else
			{ crc >>= 1; }
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
size_t kangaroo::writeKangarooCommand( uint8_t address, uint8_t command, const uint8_t* data, uint8_t length, uint8_t* buffer )
{ 
	uint16_t crc;
	buffer[0] = address; 
	buffer[1] = command;
	buffer[2] = length;
	for(size_t i = 0; i < length; i ++) 
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
		data[length ++] = 2; // Speed (Limit if combined with Position)
		length += bitpackNumber(&data[length], speedLimit);
	}
	return writeKangarooCommand(address, 36, data, length, buffer);
}

size_t kangaroo::writeKangarooSpeedCommand(uint8_t address, char channel, int32_t speed, uint8_t* buffer)
{
	uint8_t data[9] = {0};
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
	uint8_t data[2] = {0};
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

}
