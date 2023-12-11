#include <ros/ros.h>
#include <triangulation/triangulation.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "triangulation_node");
	ros::NodeHandle nh;

	triangulation::triangulator t;
	t.initTriangulator(nh);

	ros::spin();

	return 0;
}
