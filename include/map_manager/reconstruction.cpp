#include <map_manager/reconstruction.h>
#include <visualization_msgs/MarkerArray.h> 
#include <rosbag/view.h>
#include <livox_ros_driver/CustomMsg.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <pcl/common/transforms.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// onboard detector
#include <onboard_detector/dynamicDetector.h>
#include <onboard_detector/utils.h>

namespace mapManager{
	reconstruct::reconstruct(){
		this->ns_ = "reconstruction";
		this->hint_ = "[Reconstruct]";
	}

	reconstruct::reconstruct(const ros::NodeHandle& nh) : nh_(nh){
		this->ns_ = "reconstruction";
		this->hint_ = "[Reconstruct]";
		this->initParam();
		this->initPrebuiltMap();
		this->registerPub();
		this->registerCallback();
	}

	void reconstruct::initMap(const ros::NodeHandle& nh){
		this->nh_ = nh;
		this->initParam();
		this->detector_.initDetectorMap(this->nh_);
		this->initPrebuiltMap();
		this->registerPub();
		this->registerCallback();
	}


	void reconstruct::initParam(){
		// sensor input mode
		if (not this->nh_.getParam(this->ns_ + "/sensor_input_mode", this->sensorInputMode_)){
			this->sensorInputMode_ = 0;
			cout << this->hint_ << ": No sensor input mode option. Use default: depth image" << endl;
		}
		else{
			cout << this->hint_ << ": Sensor input mode: depth image (0)/pointcloud (1). Your option: " << this->sensorInputMode_ << endl;
		}		

		// localization mode
		if (not this->nh_.getParam(this->ns_ + "/localization_mode", this->localizationMode_)){
			this->localizationMode_ = 0;
			cout << this->hint_ << ": No localization mode option. Use default: pose" << endl;
		}
		else{
			cout << this->hint_ << ": Localizaiton mode: pose (0)/odom (1). Your option: " << this->localizationMode_ << endl;
		}

		// dynamic obstacle enable mode 
		if (!this->nh_.getParam(this->ns_ + "/dynamic_obstacle_filter_enable", dynamicObstacleFilterEnable_)) {
			dynamicObstacleFilterEnable_ = false; 
			ROS_WARN_STREAM(this->hint_ << ": dynamic_obstacle_filter_enable not set, default to false.");
		} else {
			ROS_INFO_STREAM(this->hint_ << ": dynamic_obstacle_filter_enable: " << dynamicObstacleFilterEnable_);
		}

		// lidar topic name 
		if (not this->nh_.getParam(this->ns_ + "/lidar_topic", this->lidarTopicName_)){
        	this->lidarTopicName_ = "/livox/lidar";
			cout << this->hint_ << ": No lidar topic name. Use default: /livox/lidar" << endl;
		}
		else{
			cout << this->hint_ << ": Lidar topic: " << this->lidarTopicName_ << endl;
    	}

		// depth topic name
		if (not this->nh_.getParam(this->ns_ + "/depth_image_topic", this->depthTopicName_)){
			this->depthTopicName_ = "/camera/depth/image_raw";
			cout << this->hint_ << ": No depth image topic name. Use default: /camera/depth/image_raw" << endl;
		}
		else{
			cout << this->hint_ << ": Depth topic: " << this->depthTopicName_ << endl;
		}

        // color topic name
		if (not this->nh_.getParam(this->ns_ + "/color_image_topic", this->colorTopicName_)){
			this->colorTopicName_ = "/camera/color/image_raw";
			cout << this->hint_ << ": No depth image topic name. Use default: /camera/color/image_raw" << endl;
		}
		else{
			cout << this->hint_ << ": Color topic: " << this->colorTopicName_ << endl;
		}

		// pointcloud topic name
		if (not this->nh_.getParam(this->ns_ + "/point_cloud_topic", this->pointcloudTopicName_)){
			this->pointcloudTopicName_ = "/camera/depth/points";
			cout << this->hint_ << ": No poincloud topic name. Use default: /camera/depth/points" << endl;
		}
		else{
			cout << this->hint_ << ": Pointcloud topic: " << this->pointcloudTopicName_ << endl;
		}

		if (this->localizationMode_ == 0){
			// odom topic name
			if (not this->nh_.getParam(this->ns_ + "/pose_topic", this->poseTopicName_)){
				this->poseTopicName_ = "/CERLAB/quadcopter/pose";
				cout << this->hint_ << ": No pose topic name. Use default: /CERLAB/quadcopter/pose" << endl;
			}
			else{
				cout << this->hint_ << ": Pose topic: " << this->poseTopicName_ << endl;
			}			
		}

		if (this->localizationMode_ == 1){
			// pose topic name
			if (not this->nh_.getParam(this->ns_ + "/odom_topic", this->odomTopicName_)){
				this->odomTopicName_ = "/CERLAB/quadcopter/odom";
				cout << this->hint_ << ": No odom topic name. Use default: /CERLAB/quadcopter/odom" << endl;
			}
			else{
				cout << this->hint_ << ": Odom topic: " << this->odomTopicName_ << endl;
			}
		}

		// std::vector<double> robotSizeVec (3);
		// if (not this->nh_.getParam(this->ns_ + "/robot_size", robotSizeVec)){
		// 	robotSizeVec = std::vector<double>{0.5, 0.5, 0.3};
		// }
		// else{
		// 	cout << this->hint_ << ": robot size: " << "[" << robotSizeVec[0]  << ", " << robotSizeVec[1] << ", "<< robotSizeVec[2] << "]" << endl;
		// }
		// this->robotSize_(0) = robotSizeVec[0]; this->robotSize_(1) = robotSizeVec[1]; this->robotSize_(2) = robotSizeVec[2];

		std::vector<double> depthIntrinsics (4);
		if (not this->nh_.getParam(this->ns_ + "/depth_intrinsics", depthIntrinsics)){
			cout << this->hint_ << ": Please check camera intrinsics!" << endl;
			exit(0);
		}
		else{
			this->fx_ = depthIntrinsics[0];
			this->fy_ = depthIntrinsics[1];
			this->cx_ = depthIntrinsics[2];
			this->cy_ = depthIntrinsics[3];
			cout << this->hint_ << ": fx, fy, cx, cy: " << "["  << this->fx_ << ", " << this->fy_  << ", " << this->cx_ << ", "<< this->cy_ << "]" << endl;
		}

		// depth scale factor
		if (not this->nh_.getParam(this->ns_ + "/depth_scale_factor", this->depthScale_)){
			this->depthScale_ = 1000.0;
			cout << this->hint_ << ": No depth scale factor. Use default: 1000." << endl;
		}
		else{
			cout << this->hint_ << ": Depth scale factor: " << this->depthScale_ << endl;
		}

		// depth min value
		if (not this->nh_.getParam(this->ns_ + "/depth_min_value", this->depthMinValue_)){
			this->depthMinValue_ = 0.2;
			cout << this->hint_ << ": No depth min value. Use default: 0.2 m." << endl;
		}
		else{
			cout << this->hint_ << ": Depth min value: " << this->depthMinValue_ << endl;
		}

		// depth max value
		if (not this->nh_.getParam(this->ns_ + "/depth_max_value", this->depthMaxValue_)){
			this->depthMaxValue_ = 5.0;
			cout << this->hint_ << ": No depth max value. Use default: 5.0 m." << endl;
		}
		else{
			cout << this->hint_ << ": Depth depth max value: " << this->depthMaxValue_ << endl;
		}

		// depth filter margin
		if (not this->nh_.getParam(this->ns_ + "/depth_filter_margin", this->depthFilterMargin_)){
			this->depthFilterMargin_ = 0;
			cout << this->hint_ << ": No depth filter margin. Use default: 0." << endl;
		}
		else{
			cout << this->hint_ << ": Depth filter margin: " << this->depthFilterMargin_ << endl;
		}

		// depth skip pixel
		if (not this->nh_.getParam(this->ns_ + "/depth_skip_pixel", this->skipPixel_)){
			this->skipPixel_ = 1;
			cout << this->hint_ << ": No depth skip pixel. Use default: 1." << endl;
		}
		else{
			cout << this->hint_ << ": Depth skip pixel: " << this->skipPixel_ << endl;
		}

		// ------------------------------------------------------------------------------------
		// depth image columns
		if (not this->nh_.getParam(this->ns_ + "/image_cols", this->imgCols_)){
			this->imgCols_ = 640;
			cout << this->hint_ << ": No depth image columns. Use default: 640." << endl;
		}
		else{
			cout << this->hint_ << ": Depth image columns: " << this->imgCols_ << endl;
		}

		// depth skip pixel
		if (not this->nh_.getParam(this->ns_ + "/image_rows", this->imgRows_)){
			this->imgRows_ = 480;
			cout << this->hint_ << ": No depth image rows. Use default: 480." << endl;
		}
		else{
			cout << this->hint_ << ": Depth image rows: " << this->imgRows_ << endl;
		}
		this->projPoints_.resize(this->imgCols_ * this->imgRows_ / (this->skipPixel_ * this->skipPixel_));
		// ------------------------------------------------------------------------------------


		// transform matrix: body to camera
		std::vector<double> body2CamVec (16);
		if (not this->nh_.getParam(this->ns_ + "/body_to_camera", body2CamVec)){
			ROS_ERROR("[reconstruct]: Please check body to camera matrix!");
		}
		else{
			for (int i=0; i<4; ++i){
				for (int j=0; j<4; ++j){
					this->body2Cam_(i, j) = body2CamVec[i * 4 + j];
				}
			}
			// cout << this->hint_ << ": from body to camera: " << endl;
			// cout << this->body2Cam_ << endl;
		}

		// transform matrix: body to lidar
		std::vector<double> body2LidVec (16);
		if (not this->nh_.getParam(this->ns_ + "/body_to_lidar", body2LidVec)){
			ROS_ERROR("[reconstruct]: Please check body to lidar matrix!");
		}
		else{
			for (int i=0; i<4; ++i){
				for (int j=0; j<4; ++j){
					this->body2Lid_(i, j) = body2LidVec[i * 4 + j];
				}
			}
		}

        // Raycast max length
		if (not this->nh_.getParam(this->ns_ + "/raycast_max_length", this->raycastMaxLength_)){
			this->raycastMaxLength_ = 5.0;
			cout << this->hint_ << ": No raycast max length. Use default: 5.0." << endl;
		}
		else{
			cout << this->hint_ << ": Raycast max length: " << this->raycastMaxLength_ << endl;
		}
        
        // absolute dir of prebuilt map file (.pcd)
		if (not this->nh_.getParam(this->ns_ + "/prebuilt_map_directory", this->prebuiltMapDir_)){
			this->prebuiltMapDir_ = "";
			cout << this->hint_ << ": Not using prebuilt map." << endl;
		}
		else{
			cout << this->hint_ << ": the prebuilt map absolute dir is found: " << this->prebuiltMapDir_ << endl;
		}

		// absolute dir of saving map file (.pcd)
		if (not this->nh_.getParam(this->ns_ + "/save_map_directory", this->saveMapDir_)){
			this->saveMapDir_ = "./";
			// cout << this->hint_ << ": Not using prebuilt map." << endl;
		}
		else{
			cout << this->hint_ << ": the save map absolute dir is found: " << this->saveMapDir_ << endl;
		}

		// absolute dir of rosbag
		if (not this->nh_.getParam(this->ns_ + "/rosbag_directory", this->rosbagDir_)){
			this->rosbagDir_ = "";
			cout << this->hint_ << ": No rosbag directory." << endl;
		}
		else{
			cout << this->hint_ << ": the rosbag absolute dir is found: " << this->rosbagDir_ << endl;
		}

		// 初始化一下 position_, orientation_
    	this->position_.setZero();
    	this->orientation_.setIdentity();

		// TODO: if you want to set TO_MERGE_CNT_ from param
    	// if (!nh_.getParam(this->ns_ + "/merge_count", this->TO_MERGE_CNT_)) {
    	//     this->TO_MERGE_CNT_ = 1;
    	// }
	}

	void reconstruct::initPrebuiltMap(){
        pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
        cout << this->hint_ << ": Loading PCD..." << endl;
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (prebuiltMapDir_, *map) == -1) //* load the file
        {
            cout << this->hint_ << ": No prebuilt map found/not using the prebuilt map." << endl;
        }
        else {
            cout << this->hint_ << ": Map loaded with " << map->width * map->height << " data points. " << endl;			
        }

        for (const auto& point : map->points) {
            pcl::PointXYZRGB point_rgb;
            point_rgb.x = point.x;
            point_rgb.y = point.y;
            point_rgb.z = point.z;
            
            // Set RGB color (e.g., red)
            point_rgb.r = 200;  // Red
            point_rgb.g = 200;    // Green
            point_rgb.b = 200;    // Blue
            
            this->prebuiltMap_.points.push_back(point_rgb);
        }
	}

	void reconstruct::registerCallback(){
        this->visTimer_ = this->nh_.createTimer(ros::Duration(0.01), &reconstruct::visCB, this);
        // this->reconstructTimer_ = this->nh_.createTimer(ros::Duration(0.033), &reconstruct::reconstructCB, this);
        this->dynamicBboxSub_ = this->nh_.subscribe("/dynamic_bboxes", 1, &reconstruct::dynamicBboxCallback, this);
    }

	void reconstruct::registerPub(){
		this->depthVisPub_ = this->nh_.advertise<sensor_msgs::Image>(this->ns_ + "/depth_image", 10);
		this->colorVisPub_ = this->nh_.advertise<sensor_msgs::Image>(this->ns_ + "/color_image", 10);
		this->poseVisPub_ = this->nh_.advertise<geometry_msgs::PoseStamped>(this->ns_ + "/pose", 10);
		
        this->dynamicBBoxesPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/dynamic_bboxes", 10);
		this->mapVisPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/map", 10);
		this->cloudVisPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/depth_cloud", 10);
    }

	void reconstruct::runBag() {
		this->bag_.open(this->rosbagDir_, rosbag::bagmode::Read);
		rosbag::View view(this->bag_);

		std::vector<sensor_msgs::Image::ConstPtr> depthQueue;
		std::vector<sensor_msgs::Image::ConstPtr> colorQueue;
		std::vector<geometry_msgs::PoseStamped::ConstPtr> poseQueue;
		std::vector<livox_ros_driver::CustomMsg::ConstPtr> lidarQueue;

		double maxAllowedTimeDiff = 0.02; // 允许的最大时间误差 (s)
		double totalPoseError = 0.0, totalDepthError = 0.0, totalColorError = 0.0;
		int matchedCount = 0;

		for (const rosbag::MessageInstance &m : view) {
			// 读取 Lidar 数据
			if (m.getTopic() == this->lidarTopicName_) {
				livox_ros_driver::CustomMsg::ConstPtr livoxMsg = m.instantiate<livox_ros_driver::CustomMsg>();
				if (!livoxMsg) continue;

				ros::Time lidarTime = livoxMsg->header.stamp;

				geometry_msgs::PoseStamped::ConstPtr bestPose;
				sensor_msgs::Image::ConstPtr bestDepth;
				sensor_msgs::Image::ConstPtr bestColor;

				// **寻找最接近的 Pose**
				while (poseQueue.size() > 1 && poseQueue.front()->header.stamp < lidarTime - ros::Duration(maxAllowedTimeDiff)) {
					poseQueue.erase(poseQueue.begin());
				}
				if (!poseQueue.empty() && fabs((poseQueue.front()->header.stamp - lidarTime).toSec()) < maxAllowedTimeDiff) {
					bestPose = poseQueue.front();
					poseQueue.erase(poseQueue.begin());
				}

				// **寻找最接近的 Depth**
				while (depthQueue.size() > 1 && depthQueue.front()->header.stamp < lidarTime - ros::Duration(maxAllowedTimeDiff)) {
					depthQueue.erase(depthQueue.begin());
				}
				if (!depthQueue.empty() && fabs((depthQueue.front()->header.stamp - lidarTime).toSec()) < maxAllowedTimeDiff) {
					bestDepth = depthQueue.front();
					depthQueue.erase(depthQueue.begin());
				}

				// **寻找最接近的 Color**
				while (colorQueue.size() > 1 && colorQueue.front()->header.stamp < lidarTime - ros::Duration(maxAllowedTimeDiff)) {
					colorQueue.erase(colorQueue.begin());
				}
				if (!colorQueue.empty() && fabs((colorQueue.front()->header.stamp - lidarTime).toSec()) < maxAllowedTimeDiff) {
					bestColor = colorQueue.front();
					colorQueue.erase(colorQueue.begin());
				}

				// **如果成功找到所有匹配数据，计算误差**
				if (bestPose && bestDepth && bestColor) {
					double poseError = fabs(bestPose->header.stamp.toSec() - lidarTime.toSec());
					double depthError = fabs(bestDepth->header.stamp.toSec() - lidarTime.toSec());
					double colorError = fabs(bestColor->header.stamp.toSec() - lidarTime.toSec());

					totalPoseError += poseError;
					totalDepthError += depthError;
					totalColorError += colorError;
					matchedCount++;

					// 先转换 Lidar 点云格式
					sensor_msgs::PointCloud2::Ptr rosCloudPtr(new sensor_msgs::PointCloud2);
					std::vector<livox_ros_driver::CustomMsg::ConstPtr> livoxVec;
					livoxVec.push_back(livoxMsg);
					this->convertLivoxToCloud(livoxVec, rosCloudPtr);

					// **检查 convertLivoxToCloud 的输出是否有点**
					std::cout << "[DEBUG] After convertLivoxToCloud: "
							<< " width=" << rosCloudPtr->width 
							<< " height=" << rosCloudPtr->height 
							<< " (point step=" << rosCloudPtr->point_step << ")\n";

					// rosmsg->pcl
					pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>());
					pcl::fromROSMsg(*rosCloudPtr, *inputCloud);

					// **打印 inputCloud 大小**
					std::cout << "[DEBUG] inputCloud size=" << inputCloud->size() << std::endl;


					// **是否启用动态障碍物过滤**
					pcl::PointCloud<pcl::PointXYZ>::Ptr staticCloud(new pcl::PointCloud<pcl::PointXYZ>());
					if (this->dynamicObstacleFilterEnable_) {

						// 传入回调函数，进行动态障碍物检测 (深度+彩色图+位姿+点云)
						this->depthColorPoseLidarCB(bestDepth, bestColor, bestPose, rosCloudPtr);
						// **获取最新的动态障碍物信息**
						std::cout << "[DEBUG] Before getDynamicBBoxes()" << std::endl;
						std::vector<onboardDetector::box3D> dynamicObstacles = detector_.getDynamicBBoxes();
						
						pcl::PointCloud<pcl::PointXYZ> worldCloud = transformLidarToWorld(*inputCloud, this->body2Lid_, bestPose);
						std::cout << "[DEBUG] Transformed inputCloud size=" << worldCloud.size() << std::endl;

						inputCloud->clear();
						*inputCloud = worldCloud;
						// 在进行动态障碍物检测前，先可视化当前帧的点云
						this->coloredDepthCloud_.clear();
						for (const auto& point : inputCloud->points) {
							pcl::PointXYZRGB coloredPoint;
							coloredPoint.x = point.x;
							coloredPoint.y = point.y;
							coloredPoint.z = point.z;

							coloredPoint.r = 255;
							coloredPoint.g = 255;
							coloredPoint.b = 255;

							this->coloredDepthCloud_.points.push_back(coloredPoint);
						}
						this->coloredDepthCloud_.width = this->coloredDepthCloud_.points.size();
						this->coloredDepthCloud_.height = 1;
						this->coloredDepthCloud_.is_dense = true;

						std::cout << "[DEBUG] coloredDepthCloud_ updated, size=" << this->coloredDepthCloud_.size() << std::endl;

						// **确保在动态障碍物检测前，先发布点云**
						this->publishCloud();
						this->publish3dBox(dynamicObstacles, this->dynamicBBoxesPub_, 0, 0, 1);
						std::cout << "[DEBUG] dynamicObstacles.size() = " << dynamicObstacles.size() << std::endl;

						// **移除动态障碍物点云**
						for (const auto &point : inputCloud->points) {
							bool isDynamic = false;
							for (const auto &bbox : dynamicObstacles) {
								// 检查点是否落在动态障碍物的 bounding box 内
								if (point.x >= bbox.x - bbox.x_width / 2 && point.x <= bbox.x + bbox.x_width / 2 &&
									point.y >= bbox.y - bbox.y_width / 2 && point.y <= bbox.y + bbox.y_width / 2 &&
									point.z >= bbox.z - bbox.z_width / 2 && point.z <= bbox.z + bbox.z_width / 2) {
									isDynamic = true;
									ROS_INFO("[DEBUG] Checking point (%.2f, %.2f, %.2f)", point.x, point.y, point.z);
									ROS_INFO("[DEBUG] bbox center=(%.2f, %.2f, %.2f), size=(%.2f, %.2f, %.2f)",
									bbox.x, bbox.y, bbox.z, bbox.x_width, bbox.y_width, bbox.z_width);
									break;
								}
							}
							if (!isDynamic) {
								staticCloud->points.push_back(point);
							}
						}
					} else {
						// 不做过滤，直接将所有点视为静态
						*staticCloud = *inputCloud;
					}

					// **打印 staticCloud 大小**
					std::cout << "[DEBUG] After obstacle filtering: staticCloud size=" 
							<< staticCloud->size() << std::endl;

					// **更新全局地图**
					pcl::PointCloud<pcl::PointXYZRGB> worldStaticCloud =
    				this->transformCloudXYZToXYZRGB(*staticCloud);
					
					this->map_ += worldStaticCloud;
					std::cout << "[DEBUG] map_ current total size=" << this->map_.size() << std::endl;

					// 输出时间戳和误差
					cout << "[Frame " << matchedCount << "]" << endl;
					cout << "  Lidar  Timestamp: " << lidarTime.toSec() << " s" << endl;
					cout << "  Pose   Timestamp: " << bestPose->header.stamp.toSec() << " s  (Error: " << poseError << " s)" << endl;
					cout << "  Depth  Timestamp: " << bestDepth->header.stamp.toSec() << " s  (Error: " << depthError << " s)" << endl;
					cout << "  Color  Timestamp: " << bestColor->header.stamp.toSec() << " s  (Error: " << colorError << " s)" << endl;
					cout << "  Static Cloud Size: " << staticCloud->size() << " points" << endl;
					cout << "-------------------------------------------------------------" << endl;
				} else {
					cout << "[runBag]: No full match found for Lidar at " << lidarTime << ", skipping frame." << endl;
				}
			}

			// 读取 Pose 数据
			if (m.getTopic() == this->poseTopicName_) {
				geometry_msgs::PoseStamped::ConstPtr poseMsg = m.instantiate<geometry_msgs::PoseStamped>();
				if (poseMsg) poseQueue.push_back(poseMsg);
			}

			// 读取 Depth 数据
			if (m.getTopic() == this->depthTopicName_) {
				sensor_msgs::Image::ConstPtr depthMsg = m.instantiate<sensor_msgs::Image>();
				if (depthMsg) depthQueue.push_back(depthMsg);
			}

			// 读取 Color 数据
			if (m.getTopic() == this->colorTopicName_) {
				sensor_msgs::Image::ConstPtr colorMsg = m.instantiate<sensor_msgs::Image>();
				if (colorMsg) colorQueue.push_back(colorMsg);
			}
		}

		// **保存最终地图**
		this->savePointCloud();
		this->bag_.close();
		cout << this->hint_ << ": Finished." << endl;
	}

	bool reconstruct::isPointInDynamicBbox(const pcl::PointXYZRGB &pt) {
		std::lock_guard<std::mutex> lock(dynamicBboxesMutex_);
		for (const auto &marker : dynamicBboxes_.markers) {
			// 假设 marker.pose.position 为包围盒中心，marker.scale 为包围盒尺寸
			double cx = marker.pose.position.x;
			double cy = marker.pose.position.y;
			double cz = marker.pose.position.z;
			double dx = marker.scale.x;
			double dy = marker.scale.y;
			double dz = marker.scale.z;

			if (pt.x >= cx - dx/2.0 && pt.x <= cx + dx/2.0 &&
				pt.y >= cy - dy/2.0 && pt.y <= cy + dy/2.0 &&
				pt.z >= cz - dz/2.0 && pt.z <= cz + dz/2.0) {
				return true;
			}
		}
		return false;
	}

	void reconstruct::convertLivoxToCloud(const std::vector<livox_ros_driver::CustomMsgConstPtr>& livoxVec,
										sensor_msgs::PointCloud2::Ptr &cloudMsgPtr) {
		// 参考 LivoxMsgCbk1 的做法
		// 只不过现在是一次性合并 livoxVec 中的多帧
		// point 类型: x, y, z, intensity, curvature(可存时间等)

		typedef pcl::PointXYZINormal PointType;
		pcl::PointCloud<PointType>::Ptr pcl_in(new pcl::PointCloud<PointType>);

		// 合并
		for (const auto &livox_msg : livoxVec) {
			if (livox_msg->point_num == 0) continue;

			auto time_end = livox_msg->points.back().offset_time;
			for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
				PointType pt;
				pt.x = livox_msg->points[i].x;
				pt.y = livox_msg->points[i].y;
				pt.z = livox_msg->points[i].z;
				float s = livox_msg->points[i].offset_time / static_cast<float>(time_end);

				pt.intensity = livox_msg->points[i].line + livox_msg->points[i].reflectivity / 10000.0;
				pt.curvature = s * 0.1f;
				pcl_in->push_back(pt);
			}
		}

		// 时间戳: 这里简单用第一帧的 timebase
		unsigned long timebase_ns = livoxVec[0]->timebase;
		sensor_msgs::PointCloud2 cloudMsg;
		cloudMsg.header.stamp.fromNSec(timebase_ns);
		cloudMsg.header.frame_id = "livox_init"; // 你可自定义

		// 转成 sensor_msgs
		pcl::toROSMsg(*pcl_in, cloudMsg);

		// 使用 boost::make_shared 创建 ConstPtr
		cloudMsgPtr = boost::make_shared<sensor_msgs::PointCloud2>(cloudMsg);
	}
	
	
	void reconstruct::depthColorPoseLidarCB(
		const sensor_msgs::Image::ConstPtr& depthMsg,  
		const sensor_msgs::Image::ConstPtr& colorMsg,  
		const geometry_msgs::PoseStamped::ConstPtr& poseMsg,  
		const sensor_msgs::PointCloud2ConstPtr& lidarCloud) {

		std::cout << "[DEBUG] Enter depthColorPoseLidarCB" << std::endl;
		if (!depthMsg || !colorMsg || !poseMsg || !lidarCloud) {
			std::cout << "[ERROR] depthColorPoseLidarCB: input pointer is null!" << std::endl;
			return;
		}
		std::cout << "[DEBUG] depth->width=" << depthMsg->width << ", height=" << depthMsg->height << std::endl;
		std::cout << "[DEBUG] color->width=" << colorMsg->width << ", height=" << colorMsg->height << std::endl;
		std::cout << "[DEBUG] lidarCloud->width=" << lidarCloud->width 
				<< ", height=" << lidarCloud->height << std::endl;
		
		// 调用 dynamicDetector 的 update 方法
		detector_.update(depthMsg, colorMsg, poseMsg, lidarCloud);

		detector_.detectionCB();
		detector_.lidarDetectionCB();
		detector_.trackingCB();
		detector_.classificationCB();
		
	}

	void reconstruct::processPoseMsg(const geometry_msgs::PoseStampedConstPtr& poseMsg){
    	// -------------- 把 Pose/odom 写入到 position_, orientation_ --------------
		// 把 Pose 转到内部存储
    	Eigen::Quaterniond quat(
        	poseMsg->pose.orientation.w,
        	poseMsg->pose.orientation.x,
        	poseMsg->pose.orientation.y,
        	poseMsg->pose.orientation.z
    	);
    	this->orientation_ = quat.toRotationMatrix();
    	this->position_ << poseMsg->pose.position.x,
                       	poseMsg->pose.position.y,
                       	poseMsg->pose.position.z;
	}

	void reconstruct::processOdomMsg(const nav_msgs::OdometryConstPtr& odomMsg){
    	// 同理
    	Eigen::Quaterniond quat(
        	odomMsg->pose.pose.orientation.w,
        	odomMsg->pose.pose.orientation.x,
        	odomMsg->pose.pose.orientation.y,
        	odomMsg->pose.pose.orientation.z
    	);
    	this->orientation_ = quat.toRotationMatrix();
    	this->position_ << odomMsg->pose.pose.position.x,
                       	odomMsg->pose.pose.position.y,
                       	odomMsg->pose.pose.position.z;
	}

	void reconstruct::dynamicBboxCallback(const visualization_msgs::MarkerArray::ConstPtr &msg) {
		std::lock_guard<std::mutex> lock(dynamicBboxesMutex_);
		dynamicBboxes_ = *msg;
	}

	pcl::PointCloud<pcl::PointXYZRGB> reconstruct::transformCloudXYZToXYZRGB(const pcl::PointCloud<pcl::PointXYZ> &inCloud){
		
		std::cout << "[DEBUG] transformCloudXYZToXYZRGB: input size=" 
				<< inCloud.size() << std::endl;

		// (1) 直接转换格式
		pcl::PointCloud<pcl::PointXYZRGB> outCloud;
		outCloud.reserve(inCloud.size());

		for (const auto &pt : inCloud.points) {
			pcl::PointXYZRGB ptRGB;
			ptRGB.x = pt.x;
			ptRGB.y = pt.y;
			ptRGB.z = pt.z;

			// 赋予固定颜色 (白色)
			ptRGB.r = 255; 
			ptRGB.g = 255;
			ptRGB.b = 255;

			outCloud.push_back(ptRGB);
		}

		std::cout << "[DEBUG] transformCloudXYZToXYZRGB: output size=" 
				<< outCloud.size() << std::endl;

		return outCloud;
	}

	pcl::PointCloud<pcl::PointXYZ> reconstruct::transformLidarToWorld(const pcl::PointCloud<pcl::PointXYZ>& inputCloud,const Eigen::Matrix4d& body2Lid,const geometry_msgs::PoseStamped::ConstPtr& poseMsg){
		
		// (1) 提取机器人位姿（PoseStamped -> Eigen）
		Eigen::Quaterniond orientation(
			poseMsg->pose.orientation.w,
			poseMsg->pose.orientation.x,
			poseMsg->pose.orientation.y,
			poseMsg->pose.orientation.z
		);
		Eigen::Vector3d position(
			poseMsg->pose.position.x,
			poseMsg->pose.position.y,
			poseMsg->pose.position.z
		);

		// (2) 计算 body -> world 变换矩阵
		Eigen::Matrix4d body2world = Eigen::Matrix4d::Identity();
		body2world.block<3,3>(0,0) = orientation.toRotationMatrix();
		body2world.block<3,1>(0,3) = position;

		// (3) 计算 lidar -> world 变换矩阵
		Eigen::Matrix4d lidar2world = body2world * body2Lid;

		// (4) 变换点云
		pcl::PointCloud<pcl::PointXYZ> transformedCloud;
    	pcl::transformPointCloud(inputCloud, transformedCloud, lidar2world.cast<float>());

		return transformedCloud;
	}

	void reconstruct::depthColorPoseCB(const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::ImageConstPtr& color, const geometry_msgs::PoseStampedConstPtr& pose){
		// store current depth image
		cv_bridge::CvImagePtr depthPtr = cv_bridge::toCvCopy(depth, depth->encoding);
		if (depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
			(depthPtr->image).convertTo(depthPtr->image, CV_16UC1, this->depthScale_);
		}
		depthPtr->image.copyTo(this->depthImage_);

		cv_bridge::CvImagePtr colorPtr = cv_bridge::toCvCopy(color, color->encoding);
		if (color->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
			(colorPtr->image).convertTo(colorPtr->image, CV_16UC1, this->depthScale_);
		}
		colorPtr->image.copyTo(this->colorImage_);

		// store current position and orientation (camera)
		Eigen::Matrix4d camPoseMatrix;
		this->getCameraPose(pose, camPoseMatrix);

		this->position_(0) = camPoseMatrix(0, 3);
		this->position_(1) = camPoseMatrix(1, 3);
		this->position_(2) = camPoseMatrix(2, 3);
		this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

		this->depthVisPub_.publish(depth);
		this->colorVisPub_.publish(color);
		this->poseVisPub_.publish(pose);
		cout<<"topic time stamp: "<<pose->header.stamp<<endl;

	}

    void reconstruct::visCB(const ros::TimerEvent&){
        // this->publishMap();
		this->publishCloud();
    }

    void reconstruct::reconstructCB(const ros::TimerEvent&){
        this->projectDepthImage();
        if (this->coloredDepthCloud_.size()>0){
            this->map_ += this->coloredDepthCloud_;
        }
        // cout<<this->map_.points.size()<<"points"<<endl;
    }

	void reconstruct::publishCloud() {
		if (this->coloredDepthCloud_.empty()) {
			std::cout << "[DEBUG] coloredDepthCloud_ is empty, skipping publishCloud()" << std::endl;
			return;
		}

		sensor_msgs::PointCloud2 cloudMsg;
		pcl::toROSMsg(this->coloredDepthCloud_, cloudMsg);

		// **确保 frame_id 与 RViz 坐标系一致**
		cloudMsg.header.frame_id = "map"; // 确保这里的 frame_id 和 TF 里的一致
		cloudMsg.header.stamp = ros::Time::now();

		// **检查 cloudVisPub_ 是否正常发布**
		if (this->cloudVisPub_.getNumSubscribers() > 0) {
			this->cloudVisPub_.publish(cloudMsg);
			std::cout << "[DEBUG] published coloredDepthCloud_ with " 
					<< this->coloredDepthCloud_.size() << " points" << std::endl;
		} else {
			std::cout << "[DEBUG] No subscribers to /depth_cloud, skipping publish." << std::endl;
		}
	}


	void reconstruct::publishMap(){
        sensor_msgs::PointCloud2 cloudMsg;
        pcl::toROSMsg(this->prebuiltMap_, cloudMsg);
        cloudMsg.header.frame_id = "map";
        cloudMsg.header.stamp = ros::Time::now();
        this->mapVisPub_.publish(cloudMsg);
    }

	void reconstruct::publish3dBox(const std::vector<onboardDetector::box3D>& boxes,
                                   const ros::Publisher& publisher,
                                   double r, double g, double b){
        visualization_msgs::MarkerArray markers;

        for (size_t i = 0; i < boxes.size(); i++)
        {
            visualization_msgs::Marker line;
            line.header.frame_id = "map";
            line.ns = "box3D";
            line.id = i;
            line.type = visualization_msgs::Marker::LINE_LIST;
            line.action = visualization_msgs::Marker::ADD;
            line.scale.x = 0.06;
            line.color.r = r;
            line.color.g = g;
            line.color.b = b;
            line.color.a = 1.0;
            line.lifetime = ros::Duration(0.05);
            line.pose.orientation.x = 0.0;
            line.pose.orientation.y = 0.0;
            line.pose.orientation.z = 0.0;
            line.pose.orientation.w = 1.0;
            line.pose.position.x = boxes[i].x;
            line.pose.position.y = boxes[i].y;
            double x_width = boxes[i].x_width;
            double y_width = boxes[i].y_width;

            double top = boxes[i].z + boxes[i].z_width / 2.0;
            double z_width = top / 2.0;
            line.pose.position.z = z_width; 

            geometry_msgs::Point corner[8];
            corner[0].x = -x_width / 2.0; corner[0].y = -y_width / 2.0; corner[0].z = -z_width;
            corner[1].x = -x_width / 2.0; corner[1].y =  y_width / 2.0; corner[1].z = -z_width;
            corner[2].x =  x_width / 2.0; corner[2].y =  y_width / 2.0; corner[2].z = -z_width;
            corner[3].x =  x_width / 2.0; corner[3].y = -y_width / 2.0; corner[3].z = -z_width;

            corner[4].x = -x_width / 2.0; corner[4].y = -y_width / 2.0; corner[4].z =  z_width;
            corner[5].x = -x_width / 2.0; corner[5].y =  y_width / 2.0; corner[5].z =  z_width;
            corner[6].x =  x_width / 2.0; corner[6].y =  y_width / 2.0; corner[6].z =  z_width;
            corner[7].x =  x_width / 2.0; corner[7].y = -y_width / 2.0; corner[7].z =  z_width;

            int edgeIdx[12][2] = {
                {0,1}, {1,2}, {2,3}, {3,0},  
                {4,5}, {5,6}, {6,7}, {7,4},  
                {0,4}, {1,5}, {2,6}, {3,7}   
            };

            for (int e = 0; e < 12; e++)
            {
                line.points.push_back(corner[edgeIdx[e][0]]);
                line.points.push_back(corner[edgeIdx[e][1]]);
            }

            markers.markers.push_back(line);
        }

        publisher.publish(markers);
    }

	void reconstruct::projectDepthImage(){
		this->projPointsNum_ = 0;

		int cols = this->depthImage_.cols;
		int rows = this->depthImage_.rows;
		uint16_t* rowPtr;

		Eigen::Vector3d currPointCam, currPointMap;
		double depth;
		const double inv_factor = 1.0 / this->depthScale_;
		const double inv_fx = 1.0 / this->fx_;
		const double inv_fy = 1.0 / this->fy_;

        pcl::PointCloud<pcl::PointXYZ> depthCloud;
        pcl::PointCloud<pcl::PointXYZRGB> coloredDepthCloud;

		// iterate through each pixel in the depth image
		for (int v=this->depthFilterMargin_; v<rows-this->depthFilterMargin_; v=v+this->skipPixel_){ // row
			rowPtr = this->depthImage_.ptr<uint16_t>(v) + this->depthFilterMargin_;
			for (int u=this->depthFilterMargin_; u<cols-this->depthFilterMargin_; u=u+this->skipPixel_){ // column
				depth = (*rowPtr) * inv_factor;
				
				if (*rowPtr == 0) {
					depth = this->raycastMaxLength_ + 0.1;
					rowPtr =  rowPtr + this->skipPixel_;
					continue;
				} else if (depth < this->depthMinValue_) {
					continue;
				} else if (depth > this->depthMaxValue_ and depth < 1.5 * this->depthMaxValue_) {
					depth = this->raycastMaxLength_ + 0.1;
				}
				else if (depth >= 1.5 * this->depthMaxValue_){
					rowPtr =  rowPtr + this->skipPixel_;
					continue;
				}

				if (*rowPtr == 0) {
					depth = this->raycastMaxLength_ + 0.1;
				} else if (depth < this->depthMinValue_) {
					continue;
				} else if (depth > this->depthMaxValue_ ) {
					depth = this->raycastMaxLength_ + 0.1;
				}

				rowPtr =  rowPtr + this->skipPixel_;

				// get 3D point in camera frame
				currPointCam(0) = (u - this->cx_) * depth * inv_fx;
				currPointCam(1) = (v - this->cy_) * depth * inv_fy;
				currPointCam(2) = depth;
				currPointMap = this->orientation_ * currPointCam + this->position_; // transform to map coordinate

                pcl::PointXYZRGB pointC;
                pointC.x = currPointMap(0);
                pointC.y = currPointMap(1);
                pointC.z = currPointMap(2);
                cv::Vec3b color = this->colorImage_.at<cv::Vec3b>(v, u);
                pointC.r = int(color[0]);
                pointC.g = int(color[1]);
                pointC.b = int(color[2]);

                // pcl::PointXYZ point;
                // point.x = currPointMap(0);
                // point.y = currPointMap(1);
                // point.z = currPointMap(2);

				// if (this->useFreeRegions_){ // this region will not be updated and directly set to free
				// 	if (this->isInHistFreeRegions(currPointMap)){
				// 		continue;
				// 	}
				// }

				// store current point
				this->projPoints_[this->projPointsNum_] = currPointMap;
				this->projPointsNum_ = this->projPointsNum_ + 1;
                // depthCloud.points.push_back(point);
                coloredDepthCloud.points.push_back(pointC);

			}
            // this->depthCloud_ = depthCloud;
            this->coloredDepthCloud_ = coloredDepthCloud;
		} 
	}

	void reconstruct::cleanPointCloud(){
		cout<<"cleaning map"<<endl;
		// Create the filter object
		pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud(this->map_.makeShared());
		sor.setMeanK(200);  // Number of neighbors to analyze for each point
		sor.setStddevMulThresh(1.0);  // Standard deviation multiplier threshold
		sor.filter(cloud_filtered);

		std::cout << "PointCloud before filtering: " << this->map_.size() << " data points." << std::endl;
		std::cout << "PointCloud after filtering: " << cloud_filtered.size() << " data points." << std::endl;

		this->map_ = cloud_filtered;
		// // Replace original cloud with filtered one
		// *cloud = *cloud_filtered;
	}

    void reconstruct::getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseMatrix){
		Eigen::Quaterniond quat;
		quat = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
		Eigen::Matrix3d rot = quat.toRotationMatrix();

		// convert body pose to camera pose
		Eigen::Matrix4d map2body; map2body.setZero();
		map2body.block<3, 3>(0, 0) = rot;
		map2body(0, 3) = pose->pose.position.x; 
		map2body(1, 3) = pose->pose.position.y;
		map2body(2, 3) = pose->pose.position.z;
		map2body(3, 3) = 1.0;

		camPoseMatrix = map2body * this->body2Cam_;
	}

	void reconstruct::getCameraPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& camPoseMatrix){
		Eigen::Quaterniond quat;
		quat = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
		Eigen::Matrix3d rot = quat.toRotationMatrix();

		// convert body pose to camera pose
		Eigen::Matrix4d map2body; map2body.setZero();
		map2body.block<3, 3>(0, 0) = rot;
		map2body(0, 3) = odom->pose.pose.position.x; 
		map2body(1, 3) = odom->pose.pose.position.y;
		map2body(2, 3) = odom->pose.pose.position.z;
		map2body(3, 3) = 1.0;

		camPoseMatrix = map2body * this->body2Cam_;
	}

    void reconstruct::savePointCloud(){
        this->map_.width = this->map_.points.size();  // Set width to number of points
        this->map_.height = 1;                    // Set height to 1 for unorganized clouds
		this->map_.is_dense = false;

        // Save the point cloud to a PCD file
        if (pcl::io::savePCDFileBinary(this->saveMapDir_, this->map_) == 0) {
            cout << "Point cloud successfully saved to 'saved_cloud.pcd'." << endl;
        } else {
            cout << "Failed to save the point cloud." << endl;
        }

    }
}
