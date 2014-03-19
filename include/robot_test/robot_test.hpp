#ifndef _kangaroo_hpp
#define _kangaroo_hpp

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <stdint.h>

namespace kangaroo
{

class kangaroo
{
public:
	kangaroo( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv );

	bool open( );
	void close( );
	bool start( );
	void stop( );
private:
	bool is_open( ) const;
	void JointTrajCB( const trajectory_msgs::JointTrajectoryPtr &msg );
	bool set_ch1( double speed );
	bool set_ch2( double speed );

	std::string port;
	std::string ch1_joint_name;
	std::string ch2_joint_name;
	int fd;

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;
	ros::Subscriber joint_traj_sub;

	size_t bitpackNumber(uint8_t* buffer, int32_t number);
	uint15_t crc14(const uint8_t* data, size_t length);
	size_t writeKangarooCommand(uint8_t address, uint8_t command, const uint8_t* data, uint8_t length, uint8_t* buffer);
	size_t writeKangarooPositionCommand(uint8_t address, char channel, int32_t position, int32_t speedLimit, uint8_t* buffer);
	size_t writeKangarooPositionCommand(uint8_t address, char channel, int32_t speed, uint8_t* buffer);
};

}

#endif /* _kangaroo_hpp */

