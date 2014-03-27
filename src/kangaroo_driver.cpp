#include "kangaroo_driver/kangaroo_driver.hpp"
#include "kangaroo_driver/kang_lib.hpp"

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <string>
#include <cmath>

//namespace kangaroo
//{

kangaroo::kangaroo( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv ) :
	port( "" ),
	ch1_joint_name( "1" ),
	ch2_joint_name( "2" ),
	fd( -1 ),
	nh( _nh ),
	nh_priv( _nh_priv ),
	encoder_lines_per_revolution(3200)
	//diameter_of_wheels(.117475)
	//msg(new sensor_msgs::JointState)
{
	ROS_INFO( "Initializing" );
	nh_priv.param( "port", port, (std::string)"/dev/ttyUSB0" );
	nh_priv.param( "ch1_joint_name", ch1_joint_name, (std::string)"1" );
	nh_priv.param( "ch2_joint_name", ch2_joint_name, (std::string)"2" );

	poll_timer = nh.createWallTimer( ros::WallDuration(0.02), &kangaroo::JointStateCB, this );

	//circumference_of_wheels = diameter_of_wheels * M_PI;
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
	if( 0 > cfsetispeed( &fd_options, B38400) )
	{
		ROS_FATAL( "Failed to set input baud: %s", strerror( errno ) );
		return false;
	}
	if( 0 > cfsetospeed( &fd_options, B38400 ) )
	{
		ROS_FATAL( "Failed to set output baud: %s", strerror( errno ) );
		return false;
	}

	fd_options.c_cflag |= ( CREAD | CLOCAL | CS8 );
	fd_options.c_cflag &= ~( PARODD | CRTSCTS | CSTOPB | PARENB );
	//fd_options.c_iflag |= ( );
	fd_options.c_iflag &= ~( IUCLC | IXANY | IMAXBEL | IXON | IXOFF | IUTF8 | ICRNL | INPCK );
	fd_options.c_oflag |= ( NL0 | CR0 | TAB0 | BS0 | VT0 | FF0 );
	fd_options.c_oflag &= ~( OPOST | ONLCR | OLCUC | OCRNL | ONOCR | ONLRET | OFILL | OFDEL | NL1 | CR1 | CR2 | TAB3 | BS1 | VT1 | FF1 );
	fd_options.c_lflag |= ( NOFLSH );
	fd_options.c_lflag &= ~( ICANON | IEXTEN | TOSTOP | ISIG | ECHOPRT | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE );
	fd_options.c_cc[VINTR] = 0x03;
	fd_options.c_cc[VQUIT] = 0x1C;
	fd_options.c_cc[VERASE] = 0x7F;
	fd_options.c_cc[VKILL] = 0x15;
	fd_options.c_cc[VEOF] = 0x04;
	fd_options.c_cc[VTIME] = 0x01;
	fd_options.c_cc[VMIN] = 0x00;
	fd_options.c_cc[VSWTC] = 0x00;
	fd_options.c_cc[VSTART] = 0x11;
	fd_options.c_cc[VSTOP] = 0x13;
	fd_options.c_cc[VSUSP] = 0x1A;
	fd_options.c_cc[VEOL] = 0x00;
	fd_options.c_cc[VREPRINT] = 0x12;
	fd_options.c_cc[VDISCARD] = 0x0F;
	fd_options.c_cc[VWERASE] = 0x17;
	fd_options.c_cc[VLNEXT] = 0x16;
	fd_options.c_cc[VEOL2] =  0x00;

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

	ROS_INFO("...Done!");
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

	//ROS_WARN("Got a valid JointTrajectory message");

	tcflush(fd, TCOFLUSH);

	double channel_1_speed = msg->points[0].velocities[ch1_idx];
	double channel_2_speed = msg->points[0].velocities[ch2_idx];

	channel_1_speed = radians_to_encoder_lines(channel_1_speed);
	channel_2_speed = radians_to_encoder_lines(channel_2_speed);

	// lock the output_mutex
	boost::mutex::scoped_lock output_lock(output_mutex);
	set_channel_speed(channel_1_speed, 128, '1');
	set_channel_speed(channel_2_speed, 128, '2');
}

bool kangaroo::send_start_signals(unsigned char address)
{
	// send the start signal to channel 1
	unsigned char buffer[7];
	int num_of_bytes_1 = write_kangaroo_start_command(address, '1', buffer);
	if (0 > write(fd, buffer, num_of_bytes_1))
	{
		ROS_ERROR("Failed to update channel 1: %s", strerror(errno));
		close();
		return false;
	}

	// send the start signal to channel 2
	int num_of_bytes_2 = write_kangaroo_start_command(address, '2', buffer);
	if (0 > write(fd, buffer, num_of_bytes_2))
	{
		ROS_ERROR("Failed to update channel 2: %s", strerror(errno));
		close();
		return false;
	}

	return true;
}

// sends the kangaroo a speed command for the given channel at the given address
bool kangaroo::set_channel_speed(double speed, unsigned char address, char channel)
{
	//std::cout << "Sending " << speed << " to Channel " << channel << std::endl;

	if (!is_open() && !open())
		return false;

	// send
	unsigned char buffer[18];
	int num_of_bytes = write_kangaroo_speed_command(address, channel, speed, buffer);
	if (0 > write(fd, buffer, num_of_bytes))
	{
		ROS_ERROR("Failed to update channel %c: %s", channel, strerror(errno));
		close();
		return false;
	}
	return true;
}

// callback for getting the Joint State from the kangaroo
void kangaroo::JointStateCB( const ros::WallTimerEvent &e )
{
	try
	{
		sensor_msgs::JointStatePtr msg(new sensor_msgs::JointState);
		msg->name.resize(2);
		msg->name[0] = ch1_joint_name;
		msg->name[1] = ch2_joint_name;
		msg->position.resize(2);
		msg->velocity.resize(2);

		// get_position and get_velocity might throw an exception if either
		//   request or the read fails
		msg->position[0] = get_parameter((unsigned char)128, '1', (unsigned char)1);	// position for ch1
		msg->velocity[0] = get_parameter((unsigned char)128, '1', (unsigned char)2);	// velocity for ch1
		msg->position[1] = get_parameter((unsigned char)128, '2', (unsigned char)1);	// position for ch2
		msg->velocity[1] = get_parameter((unsigned char)128, '2', (unsigned char)2);	// velocity for ch2

		msg->position[0] = encoder_lines_to_radians(msg->position[0]);
		msg->velocity[0] = encoder_lines_to_radians(msg->velocity[0]);
		msg->position[1] = encoder_lines_to_radians(msg->position[1]);
		msg->velocity[1] = encoder_lines_to_radians(msg->velocity[1]);

		joint_state_pub.publish(msg);
	}
	catch (std::string s)
	{
		ROS_ERROR(s.c_str());
		return;
	}
}

// send the kangaroo a request to get the position, then read from the serial to get its response
int kangaroo::get_parameter(unsigned char address, char channel, unsigned char desired_parameter)
{
	// lock both the input and output mutex-es
	output_mutex.lock();
	input_mutex.lock();

	// send the request to the kangaroo.  if it fails, free the locks and throw an exception
	if (!send_get_request(address, channel, desired_parameter))
	{
		output_mutex.unlock();
		input_mutex.unlock();
		throw std::string("Failed to send the get request.");
	}

	// unlock the output mutex
	output_mutex.unlock();

	// used to check if the read process fails
	bool ok = true;

	// try to read the message
	int result = read_message(address, ok);
	// unlock the input mutex
	input_mutex.unlock();

	// if the read fails, the throw an excpetion
	if (!ok)
	{
		throw std::string("Failed to read from the serial.");
	}
	return result;
}

bool kangaroo::send_get_request(unsigned char address, char channel, unsigned char desired_parameter)
{
	if (!is_open() && !open())
		return false;

	unsigned char buffer[18];
	int num_of_bytes = write_kangaroo_get_command(address, channel, desired_parameter, buffer);
	if (0 > write(fd, buffer, num_of_bytes))
	{
		ROS_ERROR("Failed to write to serial.");
		return false;
	}

	return true;
}

// bool& ok is used to indicate if the reading failed or succeeded
int kangaroo::read_message(unsigned char address, bool& ok)
{
	if(!is_open() && !open())
	{
		ROS_ERROR("Serial is not open.");
		ok = false;
		return 0;
	}
	//ROS_INFO("Found header, and beginning to read.");
	// note: we aren't guaranteed the exact number of bytes that the
	//   kangaroo will respond with, so we must read the header and 
	//   data separately
	unsigned char header[3] = { 0 };
	unsigned char data[8] = { 0 }; 	// 1 for channel name, 1 for flags, 
									// 1 for parameter, 5 (max) for bit-packed value
	unsigned char crc[2] = { 0 };		// crc is 2 bytes long


	//std::cout << "Header: ";
	for (size_t i = 0; i < 3; i++)
	{
		header[i] = read_one_byte(ok);
		// bail if reading one byte fails
		if (!ok)
		{
			ROS_ERROR("Failed to get the header.");
			return 0;
		}
		//std::cout << (int)header[i] << ", ";
	}
	//std::cout << std::endl;

	// Read the data
	//std::cout << "DATA: ";
	for (size_t i = 0; i < header[2]; i++)
	{
		data[i] = read_one_byte(ok);
		// bail if reading one byte fails
		if (!ok)
		{
			ROS_ERROR("Failed to get the Data.");
			return 0;
		}
		//std::cout << (int)data[i] << ", ";
	}
	//std::cout << std::endl;
	
	// read the error correcting code
	//std::cout << "CRC: ";
	for (size_t i = 0; i < 2; i++)
	{
		crc[i] = read_one_byte(ok);
		// bail if reading one byte fails
		if (!ok)
		{
			ROS_ERROR("Failed to get the CRC.");
			return 0;
		}
		//std::cout << (int)crc[i] << ", ";
	}
	//std::cout << std::endl;

	// note: evaluate_kangaroo_response will modify "ok" if something bad happens.
	return evaluate_kangaroo_response(address, header, data, ok);
}

int kangaroo::evaluate_kangaroo_response( unsigned char address, unsigned char* header, unsigned char* data, bool& ok)
{
	size_t data_size = header[2];
	size_t value_size = data_size - 3;      // there are 3 bits that aren't the value
	size_t value_offset = 3;	// offset of the value in data

	if (data[1] & (1 << 1))
	{
		//ROS_INFO("The joint is under acceleration.");
	}

	if( data[1] & (1 << 4 ))	// testing the flag to see if there is an echo code
	{
		value_size--;	// if there is an echo code, then there will be 1 extra byte
		value_offset++;
		//echo_code = true;
		ROS_INFO("There was an echo code.");
	}

	if( data[1] & (1 << 6) )
	{
		value_size--;
		value_offset++;
		ROS_INFO("There was a sequence code.");
	}

	int value = un_bitpack_number(&data[value_offset], value_size);
	//std::cout << "The value is: " << value << std::endl;
	if( data[1] & 1 )	// testing the flag to see if there is an error
	{
		handle_errors(address, value);
		ok = false;
		return 0;
	}

	return value;
}

inline double kangaroo::encoder_lines_to_radians( int encoder_lines  )
{
	return (encoder_lines * 2 * M_PI / encoder_lines_per_revolution);
}

//inline double kangaroo::encoder_lines_to_meters( int encoder_lines )
//{
// 	return (encoder_lines * circumference_of_wheels / encoder_lines_per_revolution);
//}

inline int kangaroo::radians_to_encoder_lines( double radians )
{
	return (radians * encoder_lines_per_revolution / ( 2 * M_PI ));
}

//inline int kangaroo::meters_to_encoder_lines( double meters )
//{
//	return (meters * encoder_lines_per_revolution / circumference_of_wheels);
//}

unsigned char kangaroo::read_one_byte(bool& ok)
{
	unsigned char buffer;

	int bits_read = read(fd, &buffer, 1);

	if (bits_read == 0)
	{
		// try once more to read from the serial communication
		bits_read = read(fd, &buffer, 1);
		if (bits_read < 0)
		{
			ROS_ERROR("Reading from serial critically failed: %s", strerror(errno));
			//close();
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

	if (bits_read > 0)
		return buffer;
	if (bits_read < 0)
	{
		ROS_ERROR("Reading from serial critically failed: %s", strerror(errno));
		close();
		ok = false;
		return 0;
	}

	return buffer;
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
	default:
		ROS_ERROR("Something very very bad happened.  (ERROR)");
	}
	return;
}
