#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <vector>
#include <string>
#include <queue>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>

// Livox
#include <livox_ros_driver/CustomMsg.h>
// Dynamic obstacle remove
#include <visualization_msgs/MarkerArray.h>
#include <onboard_detector/dynamicDetector.h>

using std::cout; using std::endl;
namespace mapManager{
	class reconstruct{
    protected:
        std::string ns_;
		std::string hint_;

        ros::NodeHandle nh_;
        ros::Timer visTimer_;
        ros::Timer reconstructTimer_;
		ros::Publisher depthVisPub_;
		ros::Publisher colorVisPub_;
		ros::Publisher poseVisPub_;
		ros::Publisher cloudVisPub_;
		ros::Publisher mapVisPub_;
		ros::Publisher dynamicBBoxesPub_; 

        // ros::Subscriber colorSub_;
		rosbag::Bag bag_;

		int sensorInputMode_;
		int localizationMode_;
		std::string depthTopicName_; // depth image topic
        std::string colorTopicName_; // color image topic
		std::string pointcloudTopicName_; // point cloud topic
		std::string poseTopicName_;  // pose topic
		std::string odomTopicName_; // odom topic 

		// parameters

		// CAMERA
		double fx_, fy_, cx_, cy_; // depth camera intrinsics
		double depthScale_; // value / depthScale
		double depthMinValue_, depthMaxValue_;
		int depthFilterMargin_, skipPixel_; // depth filter margin
		int imgCols_, imgRows_;
		Eigen::Matrix4d body2Cam_; // from body frame to camera frame
        double raycastMaxLength_;
		std::string prebuiltMapDir_;
		std::string saveMapDir_;
		std::string rosbagDir_;

		// data
		// -----------------------------------------------------------------
		// SENSOR DATA
		cv::Mat depthImage_;
        cv::Mat colorImage_;
		pcl::PointCloud<pcl::PointXYZ> pointcloud_;
		Eigen::Vector3d position_; // current position
		Eigen::Matrix3d orientation_; // current orientation
		Eigen::Vector3i localBoundMin_, localBoundMax_; // sensor data range


		// MAP DATA
		int projPointsNum_ = 0;
		std::vector<Eigen::Vector3d> projPoints_; // projected points from depth image
        pcl::PointCloud<pcl::PointXYZ> depthCloud_;
        pcl::PointCloud<pcl::PointXYZRGB> coloredDepthCloud_;
        pcl::PointCloud<pcl::PointXYZRGB> prebuiltMap_;
		pcl::PointCloud<pcl::PointXYZRGB> map_;

	public:
		// std::thread visWorker_;

		reconstruct(); // empty constructor
		reconstruct(const ros::NodeHandle& nh);
		virtual ~reconstruct() = default;
		void initMap(const ros::NodeHandle& nh);

		void initParam();
		void initPrebuiltMap();
		void registerCallback();
		void registerPub();

		// main function
		void runBag();

		// callback
        // void colorCB(const sensor_msgs::ImageConstPtr& img);
		void depthColorPoseCB(const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::ImageConstPtr& color, const geometry_msgs::PoseStampedConstPtr& pose);
		void depthColorPoseLidarCB(const sensor_msgs::Image::ConstPtr& depthMsg,  const sensor_msgs::Image::ConstPtr& colorMsg,  const geometry_msgs::PoseStamped::ConstPtr& poseMsg, const sensor_msgs::PointCloud2ConstPtr& lidarCloud);
		void visCB(const ros::TimerEvent&);
        void reconstructCB(const ros::TimerEvent&);

		void publishCloud();
        void publishMap();
		void publish3dBox(const std::vector<onboardDetector::box3D>& bboxes, const ros::Publisher& publisher, double r, double g, double b);
		// core function
		void projectDepthImage();
		void cleanPointCloud();
        
		void getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseMatrix);
		void getCameraPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& camPoseMatrix);
        void savePointCloud();

		


	private:

    	
		// dynamicobstacle filter
		bool dynamicObstacleFilterEnable_;
		ros::Subscriber dynamicBboxSub_;
    	visualization_msgs::MarkerArray dynamicBboxes_;
    	std::mutex dynamicBboxesMutex_;
		bool isPointInDynamicBbox(const pcl::PointXYZRGB &pt);
		void dynamicBboxCallback(const visualization_msgs::MarkerArray::ConstPtr &msg);
		onboardDetector::dynamicDetector detector_; 
		std::vector<onboardDetector::box3D> dynamicObstacles; 
    	// === Livox ===
    	
		std::string lidarTopicName_;
    	std::vector<livox_ros_driver::CustomMsg::ConstPtr> livoxCache_;
    	uint64_t TO_MERGE_CNT_ = 1;  

    	// --------------- 姿态及外参 ---------------
    	Eigen::Matrix4d body2Lid_;    

    	// --------------- 函数 ---------------
    	void processLivoxMsg(const livox_ros_driver::CustomMsgConstPtr& livoxMsg);
    	void convertLivoxToCloud(const std::vector<livox_ros_driver::CustomMsgConstPtr>& livoxVec,
                             sensor_msgs::PointCloud2::Ptr &cloudMsgPtr);

    	void processPoseMsg(const geometry_msgs::PoseStampedConstPtr& poseMsg);
    	void processOdomMsg(const nav_msgs::OdometryConstPtr& odomMsg);

    	pcl::PointCloud<pcl::PointXYZRGB> transformCloudXYZToXYZRGB(const pcl::PointCloud<pcl::PointXYZ> &inCloud);
		pcl::PointCloud<pcl::PointXYZ> transformLidarToWorld(
            const pcl::PointCloud<pcl::PointXYZ>& inputCloud,
            const Eigen::Matrix4d& body2Lid,
            const geometry_msgs::PoseStamped::ConstPtr& poseMsg);
    

    };
}

#endif
