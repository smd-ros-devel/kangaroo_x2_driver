#ifndef _kangaroo_hpp
#define _kangaroo_hpp

#include <ros/ros.h>
#include <ros/timer.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <stdint.h>
#include <boost/thread.hpp>
#include <string>

//namespace kangaroo
//{

class kangaroo
{
public:
	kangaroo( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv );
	//~kangaroo();
	bool open( );
	void close( );
	bool start( );
	void stop( );

	// continually reads from the serial port.  Meant to be called as a thread
	void read_thread();

	void requestCB( const ros::WallTimerEvent &e );

	sensor_msgs::JointStatePtr msg;

private:
	bool is_open( ) const;
	void JointTrajCB( const trajectory_msgs::JointTrajectoryPtr &msg );

	//bool read_message_debug(unsigned char address);
	// functions for sending information to the kangaroo
	bool send_get_request(unsigned char address, char channel, unsigned char desired_parameter);
	bool set_channel_speed(double speed, unsigned char address, char channel);
	bool send_start_signals(uint8_t address);

	// functions used in the read_thread
	bool read_message(unsigned char address);
	uint8_t read_one_byte(bool& ok);
	int evaluate_kangaroo_response( uint8_t address, uint8_t* header, uint8_t* data, bool& ok);
	void handle_errors(uint8_t address, int error_code);

	// address of the serial port
	std::string port;
	// the joints names for the two motors
	std::string ch1_joint_name;
	std::string ch2_joint_name;
	// file descriptor, which is used for accessing serial ports in C.
	//   it's essentially an address to the serial port
	int fd;
	int request_written;
	// Node handles
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;
	ros::WallTimer poll_timer;
	ros::Subscriber joint_traj_sub;
	ros::Publisher joint_state_pub;
	// mutex that is used for allowing only one message to be sent at a time
	boost::mutex io_mutex;

	//Joint State pointer, used to publish a joint state message
	//sensor_msgs::JointStatePtr msg;

	//int read_from_serial(uint8_t address, char channel, bool& ok);
	//int get_speed( uint8_t address, char channel, bool& ok );
	//int get_position( unsigned char address, char channel, bool& ok );
	//void poll_kangaroo( unsigned char address );
	//int read_from_serial(uint8_t address, char channel, bool& ok);
};

//}

#endif /* _kangaroo_hpp */

