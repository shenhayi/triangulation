/*
	FILE: triangulation.cpp
	--------------------------------------
	function definition of triangulator class
*/
#include <triangulation/triangulation.h>

namespace triangulation{
    triangulator::triangulator(){
        // constructor
        this->ns_ = "triangulation";
        this->hint_ = "[TRIANGULATION] ";
    }

    triangulator::triangulator(const ros::NodeHandle& nh) : nh_(nh) {
        // constructor
        this->ns_ = "triangulation";
        this->hint_ = "[TRIANGULATION] ";
        this->initParam();
        this->registerPub();
        this->registerSub();
    }

    void triangulator::initTriangulator(const ros::NodeHandle& nh){
        // initialize triangulator
        this->nh_ = nh;
        this->initParam();
        this->registerPub();
        this->registerSub();
    }

    void triangulator::initParam(){
        // depth topic name
        if (not this->nh_.getParam(this->ns_ + "/depth_image_topic", this->depthTopicName_)){
            this->depthTopicName_ = "/camera/depth/image_raw";
            cout << this->hint_ << ": No depth image topic name. Use default: /camera/depth/image_raw" << endl;
        }
        else{
            cout << this->hint_ << ": Depth topic: " << this->depthTopicName_ << endl;
        }

        // pose topic name
        if (not this->nh_.getParam(this->ns_ + "/pose_topic", this->poseTopicName_)){
            this->poseTopicName_ = "/CERLAB/quadcopter/pose";
            cout << this->hint_ << ": No pose topic name. Use default: /CERLAB/quadcopter/pose" << endl;
        }
        else{
            cout << this->hint_ << ": Pose topic: " << this->poseTopicName_ << endl;
        }

        std::vector<double> robotSizeVec (3);
        if (not this->nh_.getParam(this->ns_ + "/robot_size", robotSizeVec)){
            robotSizeVec = std::vector<double>{0.5, 0.5, 0.3};
        }
        else{
            cout << this->hint_ << ": robot size: " << "[" << robotSizeVec[0]  << ", " << robotSizeVec[1] << ", "<< robotSizeVec[2] << "]" << endl;
        }
        this->robotSize_(0) = robotSizeVec[0]; this->robotSize_(1) = robotSizeVec[1]; this->robotSize_(2) = robotSizeVec[2];

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

        // transform matrix: body to camera
        std::vector<double> body2CamVec (16);
        if (not this->nh_.getParam(this->ns_ + "/body_to_camera", body2CamVec)){
            ROS_ERROR("[triangulation]: Please check body to camera matrix!");
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
    }

    void triangulator::registerCallback() {
        // depth pose callback
        this->depthSub_.reset(
                new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depthTopicName_, 50));

        this->poseSub_.reset(
                new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
        this->depthPoseSync_.reset(
                new message_filters::Synchronizer<depthPoseSync>(depthPoseSync(100), *this->depthSub_,
                                                                     *this->poseSub_));
        this->depthPoseSync_->registerCallback(boost::bind(&triangulator::depthPoseCB, this, _1, _2));

        this->triTimer_ = this->nh_.createTimer(ros::Duration(0.1), &triangulator::triangulationCB, this);
    }


    void triangulator::registerPub(){//TODO:
        this->depthCloudPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/depth_cloud", 10);
    }

    void triangulator::depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose) {
        // store current depth image
        cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
        if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            (imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
        }
        imgPtr->image.copyTo(this->depthImage_);

        // store current position and orientation (camera)
        Eigen::Matrix4d camPoseMatrix;
        this->getCameraPose(pose, camPoseMatrix);

        this->position_(0) = camPoseMatrix(0, 3);
        this->position_(1) = camPoseMatrix(1, 3);
        this->position_(2) = camPoseMatrix(2, 3);
        this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);
    }

    void triangulator::projectDepthImage(){//TODO: (image size: height: 480; width:640) convert the 1-D array back to the original depth image
        int height = semanticMap_.layout.dim[0].size;
        int width = semanticMap_.layout.dim[1].size;
        int channel = semanticMap_.layout.dim[2].size;

        for(int i = 0; i < height; i++){
            for(int j = 0; j < width; j++){
                for(int k = 0; k < channel; k++){
                    this->depthImage_.at<uint16_t>(i, j) = semanticMap_.data[i * width * channel + j * channel + k];//convert the 1-D array back to the original depth image
                }
            }
        }

        //project depth image to point cloud
        this->projPointsNum_ = 0;

        int cols = this->depthImage_.cols;
        int rows = this->depthImage_.rows;
        uint16_t* rowPtr;

        Eigen::Vector3d currPointCam, currPointMap;
        double depth;
        const double inv_factor = 1.0 / this->depthScale_;
        const double inv_fx = 1.0 / this->fx_;
        const double inv_fy = 1.0 / this->fy_;

        for(int i = 0; i < rows; i++){
            rowPtr = this->depthImage_.ptr<uint16_t>(i);
            for(int j = 0; j < cols; j++){
                depth = rowPtr[j] * inv_factor;
                if(depth > 0.0){
                    currPointCam(0) = (j - this->cx_) * depth * inv_fx;
                    currPointCam(1) = (i - this->cy_) * depth * inv_fy;
                    currPointCam(2) = depth;
                    currPointMap = this->body2Cam_.block<3, 3>(0, 0) * currPointCam + this->body2Cam_.block<3, 1>(0, 3);
                    this->projPoints_.push_back(currPointMap);
                    this->projPointsNum_++;
                }
            }
        }
    }

    void triangulator::publishProjPoints(){
        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> cloud;

        for (int i=0; i<this->projPointsNum_; ++i){
            pt.x = this->projPoints_[i](0);
            pt.y = this->projPoints_[i](1);
            pt.z = this->projPoints_[i](2);
            cloud.push_back(pt);
        }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = "map";

        sensor_msgs::PointCloud2 cloudMsg;
        pcl::toROSMsg(cloud, cloudMsg);
        this->depthCloudPub_.publish(cloudMsg);
    }


    void triangulator::triangulationCB(const ros::TimerEvent& event){
        // project depth image to point cloud
        this->projectDepthImage();
    }

    void triangulator::registerSub(){
        // register subscribers
        this->registerCallback();
    }
}
