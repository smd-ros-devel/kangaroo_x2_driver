#include "robot_test/robot_test.hpp"

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>

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
	nh_priv.param( "port", port, (std::string)"/dev/ttyACM0" );
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
	if( 0 > cfsetispeed( &fd_options, B9600 ) )
	{
		ROS_FATAL( "Failed to set input baud: %s", strerror( errno ) );
		return false;
	}
	if( 0 > cfsetospeed( &fd_options, B9600 ) )
	{
		ROS_FATAL( "Failed to set output baud: %s", strerror( errno ) );
		return false;
	}
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

	set_ch1( msg->points[0].velocities[ch1_idx], 128);
	set_ch2( msg->points[0].velocities[ch2_idx], 128 );
}

bool kangaroo::set_ch1( double speed, unsigned char address = 128 )
{
	if( !is_open( ) && !open( ) )
		return false;

	uint8_t buffer[18];

	speed *= 127;

	// Normalize 
	if( speed > 127 )
		speed = 127;
	if( speed < -127)
		speed = -127;

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
	if( !is_open( ) && !open( ) )
		return false;

	uint8_t buffer[18];

	speed *= 127;

	// Normalize
	if( speed > 127 )
		speed = 127;
	if( speed < -127)
		speed = -127;

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
	size_t i, bit;
	for(i = 0; i < length; i ++)
	{
		crc ^= data[i] & 0x7f;
		for(bit = 0; bit < 7; bit ++)
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
size_t kangaroo::writeKangarooCommand( uint8_t address, uint8_t command, const uint8_t* data, uint8_t length, uint8_t* buffer)
{
	size_t i; 
	uint16_t crc;
	buffer[0] = address; 
	buffer[1] = command;
	for(i = 0; i < length; i ++) 
	{ 
		buffer[2 + i] = data[i]; 
	}
	crc = crc14(data, 2 + length);
	buffer[2 + length] = crc & 0x7f;
	buffer[3 + length] = (crc >> 7) & 0x7f;
	return 4 + length;
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
	data[length ++] = (uint8_t)channel;
	data[length ++] = 0;  // move flags
	data[length ++] = 1;  // Position
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
	uint8_t data[14];
	size_t length = 0;
	data[length++] = (uint8_t)channel;
	data[length++] = 0;  // move flags
	data[length++] = 2;  // Speed
	length += bitpackNumber(&data[length], speed);
	return writeKangarooCommand(address, 36, data, length, buffer);
}

}
