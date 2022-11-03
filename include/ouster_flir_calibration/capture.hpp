#pragma once
#include "ouster_flir_calibration/my_global_header.hpp"


namespace pcl{

struct PointXYZIR
  {
    PCL_ADD_POINT4D                     // Macro quad-word XYZ
    float intensity;                    // Laser intensity
    uint8_t ring;                       // Laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // Ensure proper alignment
  } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint8_t, ring, ring)
)



class CAPTURE
{
private: 
    ros::NodeHandle nh;

    // topics from params.yaml
    std::string pointcloudTopic;
    std::string imageTopic;
    std::string saveFILEDirectory;

    // message filter - camera & lidar
    message_filters::Subscriber<sensor_msgs::Image> image_sub; 
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    // save related
    std::string filename_cloud;
    std::string filename_cloud_XYZIR;
    std::string filename_image;
    bool SAVE_FLAG_ROS = false;
    bool SAVE_FLAG_XYZIR = true;
    int count          = 1;
    int cloud_sec      = 0;
    int cloud_nsec     = 0;
    int image_sec      = 0;
    int image_nsec     = 0;

    std::vector<double> K_coeff;
    std::vector<double> dist_coeff;
    cv::Mat image_rectify;


public : 

    CAPTURE()
    {
        nh.param<std::string>("/ouster_flir_calibration/pointCloudTopic", pointcloudTopic, "/os_cloud_node/points");
        nh.param<std::string>("/ouster_flir_calibration/imageTopic", imageTopic, "/camera_array/cam0/image_raw");
        nh.param<std::string>("/ouster_flir_calibration/saveFILEDirectory", saveFILEDirectory, "/home/cmslab/catkin_ws/src/ouster_flir_calibration/DATA/");
        nh.param<bool>("/ouster_flir_calibration/saveFLAG", SAVE_FLAG_ROS, false );
        nh.param<bool>("/ouster_flir_calibration/saveFLAG_XYZIR", SAVE_FLAG_XYZIR, true);

        nh.param<std::vector<double>>("/ouster_flir_calibration/K", K_coeff, {0, 0, 0, 0, 0, 0, 0, 0, 0});
        nh.param<std::vector<double>>("/ouster_flir_calibration/D", dist_coeff, {0, 0, 0, 0, 0});


        image_sub.subscribe(nh, imageTopic, 1);
        lidar_sub.subscribe(nh, pointcloudTopic, 1);
        sync_.reset(new Sync(MySyncPolicy(10), image_sub, lidar_sub));
        sync_->registerCallback(boost::bind(&CAPTURE::callback, this, _1, _2));
    }


    void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
        // ROS_INFO("Synchronization successful");

        // Change format  ROS -> OpenCV & PCL
        cv::Mat temp;
        temp = cv_bridge::toCvShare(image,"bgr8")->image;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_save(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud, *cloud_save);
        pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud_save_XYZIR(new pcl::PointCloud<pcl::PointXYZIR>);
        pcl::fromROSMsg(*cloud, *cloud_save_XYZIR);

        // Load time stamp
        cloud_sec  = cloud->header.stamp.sec;
        cloud_nsec = cloud->header.stamp.nsec;
        image_sec  = image->header.stamp.sec;
        image_nsec = image->header.stamp.nsec; 

        // get ROS parameter from terminal
        nh.getParam("/ouster_flir_calibration/saveFLAG", SAVE_FLAG_ROS);

        if(SAVE_FLAG_ROS)
        {
            capture_lidar_camera(temp, image_rectify, *cloud_save, *cloud_save_XYZIR);
            nh.setParam("/ouster_flir_calibration/saveFLAG", false);
        }   
    }


    void capture_lidar_camera(cv::Mat image, cv::Mat image_rectify, pcl::PointCloud<pcl::PointXYZI> cloud, pcl::PointCloud<pcl::PointXYZIR> cloud_XYZIR)
    {

        int NUM_ZEROS = 3;
        std::string count_str = std::to_string(count);
        std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(),'0') + count_str;

        // Rectify image
        cv::Matx33d K( K_coeff[0], K_coeff[1], K_coeff[2], K_coeff[3], K_coeff[4], K_coeff[5], K_coeff[6], K_coeff[7], K_coeff[8]);
        cv::undistort(image, image_rectify, K, dist_coeff);

        // Save image
        filename_image = saveFILEDirectory + "CALIB_image" + count_str_padded + ".png";
        cv::imwrite(filename_image, image_rectify);
        ROS_INFO("Success to save IMG %04d file.", count);

        // Save cloud_XYZI
        filename_cloud = saveFILEDirectory + "CALIB_cloud_XYZI" + count_str_padded + ".pcd";
        pcl::io::savePCDFileASCII(filename_cloud, cloud);
        ROS_INFO("Success to save PCD_XYZI %04d file.", count);

        if(SAVE_FLAG_XYZIR)
        {
            // Save cloud_XYZIR
            filename_cloud_XYZIR = saveFILEDirectory + "CALIB_cloud_XYZIR" + count_str_padded + ".pcd";
            pcl::io::savePCDFileASCII(filename_cloud_XYZIR, cloud_XYZIR);
            ROS_INFO("Success to save PCD_XYZIR %04d file.", count);
        }
        
        ROS_INFO("IMG %04d captured time is [%09d,%09d]. ", count, image_sec, image_nsec);
        ROS_INFO("PCL %04d captured time is [%09d,%09d]. ", count, cloud_sec, cloud_nsec);

        count++;
    }

};
