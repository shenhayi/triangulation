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

        //depth aligned topic name
        if (not this->nh_.getParam(this->ns_ + "/depth_aligned_topic", this->depth_alignedTopicName_)){
            this->depth_alignedTopicName_ = "/camera/aligned_depth_to_color/image_raw_t";
            cout << this->hint_ << ": No depth aligned topic name. Use default: /camera/aligned_depth_to_color/image_raw_t" << endl;
        }
        else{
            cout << this->hint_ << ": Depth aligned topic: " << this->depth_alignedTopicName_ << endl;
        }

        // pose topic name
        if (not this->nh_.getParam(this->ns_ + "/pose_topic", this->poseTopicName_)){
            this->poseTopicName_ = "/CERLAB/quadcopter/pose";
            cout << this->hint_ << ": No pose topic name. Use default: /CERLAB/quadcopter/pose" << endl;
        }
        else{
            cout << this->hint_ << ": Pose topic: " << this->poseTopicName_ << endl;
        }

        //seg map topic name
        if (not this->nh_.getParam(this->ns_ + "/seg_topic", this->semanticMapTopicName_)){
            this->semanticMapTopicName_ = "/CERLAB/quadcopter/semantic_map";
            cout << this->hint_ << ": No semantic map topic name. Use default: /CERLAB/quadcopter/semantic_map" << endl;
        }
        else{
            cout << this->hint_ << ": Semantic map topic: " << this->semanticMapTopicName_ << endl;
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
        // depth callback
        this->depthSub_.reset(
                new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depthTopicName_, 50));
        //depth aligned callback
        this->depth_alignedSub_.reset(
                new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depth_alignedTopicName_, 50));
        //pose callback
        this->poseSub_.reset(
                new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
        //TODO: add semantic map subscriber
        this->semanticMapSub_.reset(
                new message_filters::Subscriber<std_msgs::UInt16MultiArray>(this->nh_, this->semanticMapTopicName_, 25));

        this->depthSub_->registerCallback(boost::bind(&triangulator::depthImageCB, this, _1));
        this->depth_alignedSub_->registerCallback(boost::bind(&triangulator::depthAlignedImageCB, this, _1));
        this->poseSub_->registerCallback(boost::bind(&triangulator::poseCB, this, _1));
        this->semanticMapSub_->registerCallback(boost::bind(&triangulator::semanticMapCB, this, _1));

        this->triangulation_Timer_ = this->nh_.createTimer(ros::Duration(0.1), &triangulator::triangulationCB, this);
    }

    void triangulator::registerPub(){//TODO:
        // depth image publisher
        this->depthImagePub_ = this->nh_.advertise<sensor_msgs::Image>(this->ns_ + "/depth_image", 10);
        this->depthCloudPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/depth_cloud", 10);
        this->boundingBoxPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/bounding_box", 1000);
    }

    void triangulator::getMask(int height, int width, int channel) {
        this->mask_.clear(); // first, clear the old data

        // create a mask for each channel
        for(int i=0;i<channel;i++){
            cv::Mat mask(height, width, CV_16UC1, cv::Scalar(0));
            this->mask_.push_back(mask);
        }

        // fill the mask with data
        if(channel !=0){
            int data_idx = 0;
            for(int i=0;i<height;i++){
                for(int j=0;j<width;j++){
                    for(int k=0;k<channel;k++){
                        this->mask_[k].at<ushort>(i,j) = this->semanticMap_.data[data_idx];
                        data_idx++;
                    }
                }
            }
        }

    }

    void triangulator::projectDepthImage(){//TODO: (image size: height: 480; width:640) convert the 1-D array back to the original depth image
        int height = 480;
        int width = 640;
        int channel = this->semanticMap_.data.size()/height/width;
        // get mask
        this->getMask(height, width, channel);
        // get depth image from the first channel of mask
        if(!mask_.empty()){
            this->depthImage_ = mask_[0].clone();
        }else{
            std::cout << "Mask is empty, can't get depth image" << std::endl;
            this->depthImage_ = cv::Mat(480, 640, CV_16UC1, cv::Scalar(0));
        }
        // project depth image to point cloud
        this->projPoints_.clear();
        this->projPointsNum_ = 0;

        Eigen::Vector3d currPointCam, currPointMap;
        double depth;
        const double inv_factor = 1.0 / this->depthScale_;
        const double inv_fx = 1.0 / this->fx_;
        const double inv_fy = 1.0 / this->fy_;

        for (int v=0; v<this->depthImage_.rows; ++v){
            for (int u=0; u<this->depthImage_.cols; ++u){
                depth = static_cast<double>(this->depthImage_.at<ushort>(v, u)) * inv_factor;
                if (depth > 0.0){
                    bool detect = false;
                    int label = 0;
                    for(int i=1;i<channel;i++){
                        if(this->mask_[i].at<ushort>(v,u) != 0){
                            detect = true;
                            label = this->mask_[i].at<ushort>(v,u);
                            break;
                        }
                    }
                    if(detect) {
                        currPointCam(0) = (u - this->cx_) * depth * inv_fx;
                        currPointCam(1) = (v - this->cy_) * depth * inv_fy;
                        currPointCam(2) = depth;

                        // currPointMap =
                        //         this->camPoseMatrix_.block<3, 3>(0, 0) * currPointCam + this->camPoseMatrix_.block<3, 1>(0, 3);

                        currPointMap =
                        this->body2Cam_.block<3, 3>(0, 0) * currPointCam + this->body2Cam_.block<3, 1>(0, 3);
                                

                        this->projPoints_.push_back(currPointMap);
                        this->projPointsNum_++;
                    }
                }
            }
        }

        // publish point cloud
        this->publishProjPoints();
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

    void triangulator::publishDepthImage(){
        cv_bridge::CvImage img;
        img.header.stamp = ros::Time::now();
        img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        img.image = this->depthImage_;
        this->depthImagePub_.publish(img.toImageMsg());
    }

    void triangulator::depthImageCB(const sensor_msgs::ImageConstPtr& depthImageMsg){
        // depth image callback
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(depthImageMsg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        this->depthImage_ = cv_ptr->image;
    }

    void triangulator::depthAlignedImageCB(const sensor_msgs::ImageConstPtr& depthAlignedImageMsg){
        // depth aligned image callback
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(depthAlignedImageMsg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        this->depthAlignedImage_ = cv_ptr->image;
    }

    void triangulator::poseCB(const geometry_msgs::PoseStampedConstPtr& poseMsg){
        // pose callback
        this->position_(0) = poseMsg->pose.position.x;
        this->position_(1) = poseMsg->pose.position.y;
        this->position_(2) = poseMsg->pose.position.z;

        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(poseMsg->pose.orientation.w, poseMsg->pose.orientation.x, poseMsg->pose.orientation.y, poseMsg->pose.orientation.z);
        this->orientation_ = quat.toRotationMatrix();

        getCameraPose(poseMsg, this->camPoseMatrix_);
    }

    void triangulator::semanticMapCB(const std_msgs::UInt16MultiArrayConstPtr& semanticMapMsg){
        // semantic map callback
        this->semanticMap_ = *semanticMapMsg;
        std::cout << "semantic map callback" << std::endl;
    }

    void triangulator::triangulationCB(const ros::TimerEvent& event){
        // project depth image to point cloud
        this->projectDepthImage();
        this->projectObject();
        // publish depth image
        this->publishDepthImage();
        this->publishBoundingBox();
    }

    void triangulator::registerSub(){
        // register subscribers
        this->registerCallback();
    }

    void triangulator::projectObject(){  
        this->boundingboxes.clear();
        Eigen::Vector3d currPointCam, currPointMap;
        std::vector<double> ObjectPointX;
        std::vector<double> ObjectPointY;
        std::vector<double> ObjectPointZ;

        double depth;
        double object_idx;
        const double inv_factor = 1.0 / this->depthScale_;
        const double inv_fx = 1.0 / this->fx_;
        const double inv_fy = 1.0 / this->fy_;

        int height = 480;
        int width = 640;
        int num_mask = this->semanticMap_.data.size()/height/width;

        for (int i=1;i<num_mask;++i){
            ObjectPointX.clear();
            ObjectPointY.clear();
            ObjectPointZ.clear();
            int object_idx;

            for (int v=0; v<this->depthImage_.rows; ++v){
                for (int u=0; u<this->depthImage_.cols; ++u){

                    depth = static_cast<double>(this->depthImage_.at<ushort>(v, u)) * inv_factor;
                    if (depth > 0.0){
                        if (this->mask_[i].at<ushort>(v,u) != 0){
                        object_idx = this->mask_[i].at<ushort>(v,u);
                        currPointCam(0) = (u - this->cx_) * depth * inv_fx;
                        currPointCam(1) = (v - this->cy_) * depth * inv_fy;
                        currPointCam(2) = depth;

                        // currPointMap = this->camPoseMatrix_.block<3, 3>(0, 0) * currPointCam + this->camPoseMatrix_.block<3, 1>(0, 3);
                        // ObjectPointX.push_back(currPointMap(0));
                        // ObjectPointY.push_back(currPointMap(1));
                        // ObjectPointZ.push_back(currPointMap(2));
                        
                        ObjectPointX.push_back(currPointCam(0));
                        ObjectPointY.push_back(currPointCam(1));
                        ObjectPointZ.push_back(currPointCam(2));

                        }
                    }
                }
            }

            if (ObjectPointX.size()!=0 && ObjectPointY.size()!=0 && ObjectPointZ.size()!=0){
                vertex v;
                v.xmax = *max_element(ObjectPointX.begin(), ObjectPointX.end());
                v.xmin = *min_element(ObjectPointX.begin(), ObjectPointX.end());
                v.ymax = *max_element(ObjectPointY.begin(), ObjectPointY.end());
                v.ymin = *min_element(ObjectPointY.begin(), ObjectPointY.end());
                v.zmax = *max_element(ObjectPointZ.begin(), ObjectPointZ.end());
                v.zmin = *min_element(ObjectPointZ.begin(), ObjectPointZ.end());
                v.idx = object_idx;
                // cout<<"-----------object detected--------------------"<<endl;
                boundingboxes.push_back(v);
            }
            
        }

    }


    void triangulator::publishBoundingBox(){

        if (this->boundingboxes.size()!= 0){
            visualization_msgs::Marker line;
            visualization_msgs::MarkerArray lines;
            line.header.frame_id = "map";
            line.type = visualization_msgs::Marker::LINE_LIST;
            line.action = visualization_msgs::Marker::ADD;
            line.ns = "box3D";  
            line.scale.x = 0.06;
            line.color.r = 1;
            line.color.g = 0;
            line.color.b = 0;
            line.color.a = 1.0;
            line.lifetime = ros::Duration(0.1);
            Eigen::Vector3d vertex_pose;


            for(size_t i = 0; i < boundingboxes.size(); i++){
                vertex v = this->boundingboxes[i];
                std::vector<geometry_msgs::Point> verts;
                verts.clear();
                geometry_msgs::Point p;
                
                vertex_pose(0) = v.xmax; vertex_pose(1) = v.ymax; vertex_pose(2) = v.zmax;
                Cam2Map(vertex_pose);
                p.x = vertex_pose(0); p.y = vertex_pose(1); p.z = vertex_pose(2);
                verts.push_back(p);

                vertex_pose(0) = v.xmin; vertex_pose(1) = v.ymax; vertex_pose(2) = v.zmax;
                Cam2Map(vertex_pose);
                p.x = vertex_pose(0); p.y = vertex_pose(1); p.z = vertex_pose(2);
                verts.push_back(p);

                vertex_pose(0) = v.xmin; vertex_pose(1) = v.ymin; vertex_pose(2) = v.zmax;
                Cam2Map(vertex_pose);
                p.x = vertex_pose(0); p.y = vertex_pose(1); p.z = vertex_pose(2);
                verts.push_back(p);

                vertex_pose(0) = v.xmax; vertex_pose(1) = v.ymin; vertex_pose(2) = v.zmax;
                Cam2Map(vertex_pose);
                p.x = vertex_pose(0); p.y = vertex_pose(1); p.z = vertex_pose(2);
                verts.push_back(p);

                vertex_pose(0) = v.xmax; vertex_pose(1) = v.ymax; vertex_pose(2) = v.zmin;
                Cam2Map(vertex_pose);
                p.x = vertex_pose(0); p.y = vertex_pose(1); p.z = vertex_pose(2);
                verts.push_back(p);

                vertex_pose(0) = v.xmin; vertex_pose(1) = v.ymax; vertex_pose(2) = v.zmin;
                Cam2Map(vertex_pose);
                p.x = vertex_pose(0); p.y = vertex_pose(1); p.z = vertex_pose(2);
                verts.push_back(p);

                vertex_pose(0) = v.xmin; vertex_pose(1) = v.ymin; vertex_pose(2) = v.zmin;
                Cam2Map(vertex_pose);
                p.x = vertex_pose(0); p.y = vertex_pose(1); p.z = vertex_pose(2);
                verts.push_back(p);

                vertex_pose(0) = v.xmax; vertex_pose(1) = v.ymin; vertex_pose(2) = v.zmin;
                Cam2Map(vertex_pose);
                p.x = vertex_pose(0); p.y = vertex_pose(1); p.z = vertex_pose(2);
                verts.push_back(p);

                // p.x = v.xmax; p.y = v.ymax; p.z = v.zmax;
                // verts.push_back(p);
                // p.x = v.xmin; p.y = v.ymax; p.z = v.zmax;
                // verts.push_back(p);
                // p.x = v.xmin; p.y = v.ymin; p.z = v.zmax;
                // verts.push_back(p);
                // p.x = v.xmax; p.y = v.ymin; p.z = v.zmax;
                // verts.push_back(p);
                // p.x = v.xmax; p.y = v.ymax; p.z = v.zmin;
                // verts.push_back(p);
                // p.x = v.xmin; p.y = v.ymax; p.z = v.zmin;
                // verts.push_back(p);
                // p.x = v.xmin; p.y = v.ymin; p.z = v.zmin;
                // verts.push_back(p);
                // p.x = v.xmax; p.y = v.ymin; p.z = v.zmin;
                // verts.push_back(p);

                int vert_idx[12][2] = {
                    {0,1},
                    {1,2},
                    {2,3},
                    {0,3},
                    {0,4},
                    {1,5},
                    {3,7},
                    {2,6},
                    {4,5},
                    {5,6},
                    {4,7},
                    {6,7}
                };

                for (size_t i=0;i<12;i++){
                        line.points.push_back(verts[vert_idx[i][0]]);
                        line.points.push_back(verts[vert_idx[i][1]]);
                    }
                    
                    lines.markers.push_back(line);
                    line.id++;
            }
            this->boundingBoxPub_.publish(lines);
        }
    }

    void triangulator::Cam2Map(Eigen::Vector3d &position){
        position = this->camPoseMatrix_.block<3, 3>(0, 0) * position + this->camPoseMatrix_.block<3, 1>(0, 3);
    }

}

