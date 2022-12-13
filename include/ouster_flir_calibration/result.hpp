#pragma once
#include "ouster_flir_calibration/my_global_header.hpp"


struct PointXYZR
{
    double x;
    double y;
    double z;
    int ring;
};


class RESULT
{
private:
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    // publisher
    ros::Publisher  lidar_pub;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub;

    // params.yaml
    std::string image_topic;
    std::string lidar_topic;
    std::vector<double> K_coeff;
    std::vector<double> dist;
    std::vector<double> r_cl_coeff;
    std::vector<double> t_cl_coeff;

    // filter limit
    double xlim_low;  double xlim_high;  
    double ylim_low;  double ylim_high;  
    double zlim_low;  double zlim_high;  


public:

    RESULT() : it_(nh)
    {
        nh.param<std::string>("/ouster_flir_calibration/pointCloudTopic", lidar_topic, "/os_cloud_node/points");
        nh.param<std::string>("/ouster_flir_calibration/imageTopic", image_topic, "/camera_array/cam0/image_raw"  );
        nh.param<std::vector<double>>("/ouster_flir_calibration/K", K_coeff, {0, 0, 0, 0, 0, 0, 0, 0, 0});
        nh.param<std::vector<double>>("/ouster_flir_calibration/D", dist, {0, 0, 0, 0, 0});
        nh.param<std::vector<double>>("/ouster_flir_calibration/r_cl", r_cl_coeff, {0, 0, 0});
        nh.param<std::vector<double>>("/ouster_flir_calibration/t_cl", t_cl_coeff, {0, 0, 0});

        // message filter
        image_sub.subscribe(nh, image_topic, 1);
        lidar_sub.subscribe(nh, lidar_topic, 1);
        sync_.reset(new Sync(MySyncPolicy(10), image_sub, lidar_sub));
        sync_->registerCallback(boost::bind(&RESULT::callback, this, _1, _2));

        // publisher
        image_pub = it_.advertise("/ouster_flir_calibration/CALIB_RESULT", 1);
        lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/ouster_flir_calibration/CALIB_LIDAR", 1);
    }



    void callback(const sensor_msgs::ImageConstPtr& image,  const sensor_msgs::PointCloud2ConstPtr& lidar)
    {

        ////////////////////////
        /* IMAGE UNDISTORTION */
        ////////////////////////

        // change format  ROS -> OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        cv::Mat image_cv = cv_ptr->image;

        // rectify image
        cv::Matx33d K(K_coeff[0], K_coeff[1], K_coeff[2], K_coeff[3], K_coeff[4], K_coeff[5], K_coeff[6], K_coeff[7], K_coeff[8]);
        cv::Mat image_rectify;
        cv::undistort(image_cv, image_rectify, K, dist);


        ///////////
        /* LIDAR */
        ///////////

        // change format ROS -> PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*lidar, *cloud_ptr);

        // get ring information and filter x, y, z
        std::vector<PointXYZR> cloud_filtered;
        
        // update filter limit from dynamic_reconfigure
        std::vector<double> filter_limit = {0.0, 100.0, -200.0, 200.0, -50.0, 50.0};  // xlim_low[m], xlim_high[m], ylim_low[m], ylim_high[m], zlim_low[m], zlim_high[m]
        nh.getParam("/dynamic_reconfigure/xlim_low",  xlim_low);
        nh.getParam("/dynamic_reconfigure/xlim_high", xlim_high);
        nh.getParam("/dynamic_reconfigure/ylim_low",  ylim_low);
        nh.getParam("/dynamic_reconfigure/ylim_high", ylim_high);
        nh.getParam("/dynamic_reconfigure/zlim_low",  zlim_low);
        nh.getParam("/dynamic_reconfigure/zlim_high", zlim_high);
        filter_limit[0] = xlim_low;    filter_limit[1] = xlim_high;
        filter_limit[2] = ylim_low;    filter_limit[3] = ylim_high;
        filter_limit[4] = zlim_low;    filter_limit[5] = zlim_high;

        getRing(cloud_ptr, cloud_filtered, filter_limit);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZI>);
        passthrough(cloud_ptr, cloud_pass, filter_limit);

        sensor_msgs::PointCloud2::Ptr cloud_pass_ros(new sensor_msgs::PointCloud2);
        PCL2ROS<pcl::PointCloud<pcl::PointXYZI>::Ptr>(cloud_pass, cloud_pass_ros, "os_sensor");
        lidar_pub.publish(*cloud_pass_ros);



        //////////////////////////////
        /* LIDAR-CAMERA-CALIBRATION */
        //////////////////////////////

        std::vector<cv::Point3d> object_points;
        std::vector<cv::Scalar>  color_array;

        for (int i = 0; i < cloud_filtered.size(); i++)
        {
            double x_temp = cloud_filtered[i].x;
            double y_temp = cloud_filtered[i].y;
            double z_temp = cloud_filtered[i].z;
            object_points.push_back(cv::Point3d(x_temp, y_temp, z_temp));

            int ring = cloud_filtered[i].ring;
            cv::Scalar COLOR = getColor(ring);
            color_array.push_back(COLOR);
        }


        cv::Mat r_cl(3, 1, cv::DataType<double>::type);
        r_cl.at<double>(0) = r_cl_coeff[0];
        r_cl.at<double>(1) = r_cl_coeff[1];
        r_cl.at<double>(2) = r_cl_coeff[2];

        cv::Mat t_cl(3, 1, cv::DataType<double>::type);
        t_cl.at<double>(0) = t_cl_coeff[0];
        t_cl.at<double>(1) = t_cl_coeff[1];
        t_cl.at<double>(2) = t_cl_coeff[2];

        cv::Mat project_points;
        cv::projectPoints(object_points, r_cl, t_cl, K, dist, project_points);


        cv::Mat RESULT;
        RESULT = image_rectify.clone();

        for (int idx = 0; idx < cloud_filtered.size(); idx++)
        {
            double u = project_points.at<double>(idx,0);
            double v = project_points.at<double>(idx,1);

            if ( (0 <= u) && (u < RESULT.size().width) && (0 <= v) && (v < RESULT.size().height))
                     cv::circle(RESULT, cv::Point(u,v), 2.0, color_array[idx], -1, -1, 0);
        }

        // change format  OpenCV -> ROS
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", RESULT).toImageMsg();
        image_msg->header.stamp = ros::Time::now();
        image_pub.publish(image_msg);
    }


    void passthrough(const  pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_src,  pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_dst, std::vector<double> filter_range)
    {
        double min_x = filter_range[0];
        double max_x = filter_range[1];
        double min_y = filter_range[2];
        double max_y = filter_range[3];
        double min_z = filter_range[4];
        double max_z = filter_range[5];

        pcl::PointCloud<pcl::PointXYZI>::Ptr temp;
        temp.reset(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PassThrough<pcl::PointXYZI> pass_filter_x;
        pcl::PassThrough<pcl::PointXYZI> pass_filter_y;
        pcl::PassThrough<pcl::PointXYZI> pass_filter_z;

        pass_filter_x.setInputCloud(cloud_src);
        pass_filter_x.setFilterFieldName("x");
        pass_filter_x.setFilterLimits(min_x, max_x);
        pass_filter_x.setFilterLimitsNegative(false);
        pass_filter_x.filter(*temp);

        pass_filter_y.setInputCloud(temp);
        pass_filter_y.setFilterFieldName("y");
        pass_filter_y.setFilterLimits(min_y, max_y);
        pass_filter_y.setFilterLimitsNegative(false);
        pass_filter_y.filter(*temp);

        pass_filter_z.setInputCloud(temp);
        pass_filter_z.setFilterFieldName("z");
        pass_filter_z.setFilterLimits(min_z, max_z);
        pass_filter_z.setFilterLimitsNegative(false);
        pass_filter_z.filter(*cloud_dst);
    }

    template<typename T>
    void PCL2ROS(const T &pclCloud, sensor_msgs::PointCloud2::Ptr &rosCloud, const std::string &frame_id)
    {
        pcl::toROSMsg(*pclCloud, *rosCloud);
        rosCloud->header.stamp    = ros::Time::now();
        rosCloud->header.frame_id = frame_id;
    }

    void getRing(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_XYZI, std::vector<PointXYZR> &cloud_XYZR, std::vector<double> filter_limit)
    {
        int width  = cloud_XYZI->width;     // width 1024
        int height = cloud_XYZI->height;    // height 64
        int idx_point = 0;

        for (int idx_ring = 0; idx_ring < height; idx_ring++)   // for each channel 
        {
            for (int idx_width = 0; idx_width < width; idx_width++)
            {
                double x = cloud_XYZI->points[idx_point].x;
                double y = cloud_XYZI->points[idx_point].y;
                double z = cloud_XYZI->points[idx_point].z;

                if ( (filter_limit[0] < x) && (x < filter_limit[1]) &&
                     (filter_limit[2] < y) && (y < filter_limit[3]) &&
                     (filter_limit[4] < z) && (z < filter_limit[5]) )
                {
                    PointXYZR point_new{x, y, z, idx_ring};
                    cloud_XYZR.push_back(point_new);
                }
                idx_point++;
            }
        }
    }

    cv::Scalar getColor(const int &ring)
    {
        cv::Scalar RED     = cv::Scalar(0,    0, 255);
        cv::Scalar YELLOW  = cv::Scalar(0,  255, 255); 
        cv::Scalar GREEN   = cv::Scalar(0,  255,   0);
        cv::Scalar BLUE    = cv::Scalar(255,  0,   0);
        cv::Scalar PURPLE  = cv::Scalar(255,  0, 100);

        cv::Scalar COLOR;
        int remain = ring % 5;
        if (remain == 0) {COLOR = RED;}
        else if(remain == 1) {COLOR = YELLOW;}
        else if(remain == 2) {COLOR = GREEN;} 
        else if(remain == 3) {COLOR = BLUE;}
        else if(remain == 4) {COLOR = PURPLE;}
        else{}

        return COLOR;
    }

};