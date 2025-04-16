#include <ros/ros.h>
#include <map_manager/reconstruction.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "reconstruction_node");
	ros::NodeHandle nh;

	mapManager::reconstruct r;
	r.initMap(nh);
	
	r.runBag();

	ros::spin();

	r.savePointCloud();

	return 0;
}