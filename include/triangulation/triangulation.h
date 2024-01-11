/*
	FILE: triangulation,h
	-------------------------------------
	occupancy voxel map header file
*/
#ifndef TRIANGULATION_TRIANGLATION
#define TRIANGULATION_TRIANGLATION
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <queue>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <thread>
#include <std_msgs/UInt16MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include<geometry_msgs/PoseStamped.h>
#include <algorithm>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include<triangulation/utils.h>

using std::cout; using std::endl;
namespace triangulation{
    class BoundingBox{ // bounding box class
    public:
        vertex v;
        object o;
        int label;
    };
    class triangulator{
        private:

	    protected:
        std::string ns_;
		std::string hint_;

		// ROS
		ros::NodeHandle nh_;
		std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depthSub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_alignedSub_;
		std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> poseSub_;
        std::shared_ptr<message_filters::Subscriber<std_msgs::UInt16MultiArray>> semanticMapSub_;
        //TODO: add semantic map subscriber
		ros::Timer triangulation_Timer_;
        ros::Timer visualization_Timer_;
        ros::Publisher depthCloudPub_;
        ros::Publisher depthImagePub_;
        ros::Publisher boundingBoxPub_;

		std::string depthTopicName_; // depth image topic
        std::string depth_alignedTopicName_; // depth aligned image topic
		std::string poseTopicName_;  // pose topic
        std::string semanticMapTopicName_; // semantic map topic

		// parameters
		// -----------------------------------------------------------------
		// ROBOT SIZE
		Eigen::Vector3d robotSize_;

		// CAMERA
		double fx_, fy_, cx_, cy_; // depth camera intrinsics
		double depthScale_; // value / depthScale
        double depthMinValue_, depthMaxValue_;
		int imgCols_, imgRows_;
		Eigen::Matrix4d body2Cam_; // from body frame to camera frame
        Eigen::Matrix4d camPoseMatrix_;

        // data
        // -----------------------------------------------------------------
        // SENSOR DATA //TODO
        cv::Mat depthImage_;
        cv::Mat depthAlignedImage_;
        pcl::PointCloud<pcl::PointXYZ> pointcloud_;
        Eigen::Vector3d position_; // current position
        Eigen::Matrix3d orientation_; // current orientation
        Eigen::Vector3i localBoundMin_, localBoundMax_; // sensor data range
        //TODO: semantic map data and finally depth image
        std_msgs::UInt16MultiArray semanticMap_;

        // MAP DATA
        int projPointsNum_ = 0;
        std::vector<Eigen::Vector3d> projPoints_; // projected points from depth image
        std::vector<int> countHitMiss_;
        std::vector<int> countHit_;
        std::queue<Eigen::Vector3i> updateVoxelCache_;
        std::vector<double> occupancy_; // occupancy log data
        std::vector<bool> occupancyInflated_; // inflated occupancy data
        int raycastNum_ = 0;
        std::vector<int> flagTraverse_, flagRayend_;
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions_;
        std::deque<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> histFreeRegions_;
        Eigen::Vector3d currMapRangeMin_ = Eigen::Vector3d (0, 0, 0);
        Eigen::Vector3d currMapRangeMax_ = Eigen::Vector3d (0, 0, 0);
        bool useFreeRegions_ = false;

        //Segmentation
        std::vector<std::vector<Eigen::Vector3d>> segments_;
        std::vector<cv::Mat> mask_;
        std::vector<int> labels_;
        std::vector<std::string> label2name_; // coco.txt
        std::vector<std::string> classNames_; // detected object class names
        std::string class_labels_path_;

        //Bounding box
        std::vector<vertex> boundingboxes;
        std::vector<object> objectposes;
        std::vector<BoundingBox> allBoundingBoxes; // all bounding boxes

		public:
		triangulator();
		triangulator(const ros::NodeHandle& nh);
		virtual ~triangulator() = default;
		void initTriangulator(const ros::NodeHandle& nh);
		void initParam();
		void registerCallback();
		void registerPub();
        void registerSub();

        void triangulationCB(const ros::TimerEvent& event);
        void visualizationCB(const ros::TimerEvent& event);

        void depthImageCB(const sensor_msgs::ImageConstPtr& depthImageMsg);
        void depthAlignedImageCB(const sensor_msgs::ImageConstPtr& depthAlignedImageMsg);
        void poseCB(const geometry_msgs::PoseStampedConstPtr& poseMsg);
        void semanticMapCB(const std_msgs::UInt16MultiArrayConstPtr& semanticMapMsg);

        void initLabelMap();
        void getMask(int height, int width, int channel);
        void getLabels();
        void label2name();
		void projectDepthImage(); // project depth image to point cloud

        void getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseMatrix);
        void publishDepthImage(); // publish depth image
        void publishProjPoints(); // publish depth cloud
        
        void projectObject();
        void getBoundingBox();
        void publishBoundingBox();
        void publishAllBoundingBoxes(); // publish all bounding boxes
        void allBoxFilter(); // filter all bounding boxes
        void Cam2Map(Eigen::Vector3d &position);
        object GetObjectPosition(const vertex &v);
        vertex GetObjectVertex(const object &o);
        bool IsDetected(const vertex &v);
    };
    inline void triangulator::getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseMatrix){
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
    // inline double getDistance(pose p1, pose p2){

	//     return sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2));	

    // }
}


#endif
