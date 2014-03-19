#include <robot_test/robot_test.hpp>

int main( int argc, char *argv[] )
{
	ros::NodeHandle *nh = NULL;
	ros::NodeHandle *nh_priv = NULL;
	kangaroo::kangaroo *qik = NULL;

	ros::init( argc, argv, "kangaroo_node" );

	nh = new ros::NodeHandle( );
	if( !nh )
	{
		ROS_FATAL( "Failed to initialize NodeHandle" );
		ros::shutdown( );
		return -1;
	}
	nh_priv = new ros::NodeHandle( "~" );
	if( !nh_priv )
	{
		ROS_FATAL( "Failed to initialize private NodeHandle" );
		delete nh;
		ros::shutdown( );
		return -2;
	}
	qik = new kangaroo::kangaroo( *nh, *nh_priv );
	if( !qik )
	{
		ROS_FATAL( "Failed to initialize driver" );
		delete nh_priv;
		delete nh;
		ros::shutdown( );
		return -3;
	}
	if( !qik->start( ) )
		ROS_ERROR( "Failed to start the driver" );

	ros::spin( );

	delete qik;
	delete nh_priv;
	delete nh;

	return 0;
}
