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



class RESULT
{
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub;


    // topics from params.yaml
    std::string pointcloudTopic;
    std::string imageTopic;

    // message filter - camera & lidar
    message_filters::Subscriber<sensor_msgs::Image> image_sub; 
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;


    // pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr     inputCloud;       
    pcl::PointCloud<pcl::PointXYZI>::Ptr     filterCloud;
    pcl::PointCloud<pcl::PointXYZIR>::Ptr    inputCloud_XYZIR;

    // double filter_range[6] = {+0.00, 7.00, -10.00, +10.00, -5.00, +5.00};   // min_x, max_x, min_y, max_y, min_z, max_z
    double filter_range[6] = {+0.00, 18.00, -10.00, +10.00, -10.00, +10.00};   // min_x, max_x, min_y, max_y, min_z, max_z   //20
    std::string frame_id = "map";

    std::vector<double> K_coeff;
    std::vector<double> dist_coeff;
    std::vector<double> r_cl_coeff;
    std::vector<double> t_cl_coeff;



public:

    RESULT() : it_(nh)
    {
        nh.param<std::string>("/ouster_flir_calibration/pointCloudTopic", pointcloudTopic, "/os_cloud_node/points");
        nh.param<std::string>("/ouster_flir_calibration/imageTopic", imageTopic, "/camera_array/cam0/image_raw"  );
        nh.param<std::vector<double>>("/ouster_flir_calibration/K", K_coeff, {0, 0, 0, 0, 0, 0, 0, 0, 0});
        nh.param<std::vector<double>>("/ouster_flir_calibration/D", dist_coeff, {0, 0, 0, 0, 0});
        nh.param<std::vector<double>>("/ouster_flir_calibration/r_cl", r_cl_coeff, {0, 0, 0});
        nh.param<std::vector<double>>("/ouster_flir_calibration/t_cl", t_cl_coeff, {0, 0, 0});

        inputCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        filterCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        inputCloud_XYZIR.reset(new pcl::PointCloud<pcl::PointXYZIR>());
        
        // ros & message_filter
        image_sub.subscribe(nh, imageTopic, 1);
        lidar_sub.subscribe(nh, pointcloudTopic, 1);
        sync_.reset(new Sync(MySyncPolicy(10), image_sub, lidar_sub));
        sync_->registerCallback(boost::bind(&RESULT::callback, this, _1, _2));

        // publisher
        image_pub = it_.advertise("/RESULT/calibrated", 1);

        cv::namedWindow("Image window");
    }

    ~RESULT()
    {
        cv::destroyWindow("Image window");
    }



void callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    ROS_INFO("Synchronization successful");

    // Change format  ROS -> OpenCV & PCL
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img_input = cv_ptr->image;
    pcl::fromROSMsg(*cloud, *inputCloud);

    getRing(inputCloud, inputCloud_XYZIR);
    passthrough(inputCloud, filterCloud, filter_range);

    cv::Matx33d K( K_coeff[0], K_coeff[1], K_coeff[2], K_coeff[3], K_coeff[4], K_coeff[5], K_coeff[6], K_coeff[7], K_coeff[8]);
    cv::Mat r_cl, t_cl, R_cl, R_lc, t_lc;
    solvePnP_lidar_camera(r_cl, t_cl, R_cl, R_lc, t_lc, K, dist_coeff);

    cv::Mat img_result = projectlidar2image(inputCloud_XYZIR, img_input, r_cl, t_cl, K, dist_coeff, filter_range);

    sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_result).toImageMsg();
    

    image_pub.publish(pub_msg);
}

    template<typename T>
    void PCL2ROS(const T &pclCloud, sensor_msgs::PointCloud2::Ptr &rosCloud, const std::string &frame_id)
    {
        pcl::toROSMsg(*pclCloud, *rosCloud);
        rosCloud->header.stamp    = ros::Time::now();
        rosCloud->header.frame_id = frame_id;
    }

    template<typename T>
    void passthrough(const  pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_src,  pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_dst, const T filter_range[])
    {
        T min_x = filter_range[0];
        T max_x = filter_range[1];
        T min_y = filter_range[2];
        T max_y = filter_range[3];
        T min_z = filter_range[4];
        T max_z = filter_range[5];

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

    void getRing(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_XYZI, pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud_XYZIR)
    {
        int width  = cloud_XYZIR->width;     // width 1024
        int height = cloud_XYZIR->height;    // height 64
        int idx_point = 0;

        pcl::copyPointCloud(*cloud_XYZI, *cloud_XYZIR);
        for (int idx_ring = 0; idx_ring < height; idx_ring++)   // for each channel 
        {
            for (int idx_width = 0; idx_width < width; idx_width++)
            {
                cloud_XYZIR->points[idx_point].x = cloud_XYZI->points[idx_point].x;
                cloud_XYZIR->points[idx_point].y = cloud_XYZI->points[idx_point].y;
                cloud_XYZIR->points[idx_point].z = cloud_XYZI->points[idx_point].z;
                cloud_XYZIR->points[idx_point].ring = idx_ring;
                idx_point++;
            }
        }
    }


    void solvePnP_lidar_camera(cv::Mat &r_cl, cv::Mat &t_cl, cv::Mat &R_cl, cv::Mat &R_lc, cv::Mat &t_lc, cv::Matx33d &K, std::vector<double> &dist_coeff)
    {

    std::vector<cv::Point3d> lidar_points = 
    {
        // #001
        cv::Point3d(2.853236, 0.334955, -0.358741),
        cv::Point3d(3.153665, -0.038886, 0.300245),
        cv::Point3d(3.137304, -0.445902, 0.068099),
        cv::Point3d(2.853545, 0.086968, -0.603120),
        // #002
        cv::Point3d(2.782402, 0.326629, -0.348882),
        cv::Point3d(3.150378, -0.058181, 0.299994),
        cv::Point3d(3.166869, -0.430129, 0.027854),
        cv::Point3d(2.837943, -0.086486, -0.599605),
        // #003
        cv::Point3d(2.782402, 0.326629, -0.348882),
        cv::Point3d(3.150378, -0.058181, 0.299994),
        cv::Point3d(3.166869, -0.430129, 0.027854),
        cv::Point3d(2.837943, -0.086486, -0.599605),
        // #004
        cv::Point3d(3.019161, 0.334861, -0.420302),
        cv::Point3d(3.160819, -0.019577, 0.300832),
        cv::Point3d(2.977656, -0.441359, 0.140223),
        cv::Point3d(2.841843, -0.086606, -0.600483),
        // #005
        cv::Point3d(3.055087, 0.301782, -0.385989),
        cv::Point3d(3.132740, -0.038630, 0.298485),
        cv::Point3d(2.942066, -0.436089, 0.138973),
        cv::Point3d(2.856348, -0.051980, -0.603559),
        // #006
        cv::Point3d(2.803447, 0.276401, -0.281495),
        cv::Point3d(3.144876, -0.019480, 0.299491),
        cv::Point3d(3.241183, -0.440250, 0.027658),
        cv::Point3d(2.831593, -0.068902, -0.598067),
        // #007
        cv::Point3d(2.695799, 0.282862, -0.303779),
        cv::Point3d(3.124769, -0.038533, 0.297814),
        cv::Point3d(3.236881, -0.419452, 0.027676),
        cv::Point3d(2.817466, -0.085853, -0.594992),
        // #008
        cv::Point3d(2.742020, 0.304838, -0.342994),
        cv::Point3d(3.139715, -0.038715, 0.299072),
        cv::Point3d(3.242831, -0.420226, 0.027661),
        cv::Point3d(2.830618, -0.068878, -0.597847),
    };

    std::vector<cv::Point2d> image_points =
    {
        // #001
        cv::Point2d(404, 760),
        cv::Point2d(732, 196),
        cv::Point2d(1069, 387),
        cv::Point2d(776, 968),

        // #002
        cv::Point2d(413, 746),
        cv::Point2d(736, 194),
        cv::Point2d(1049, 406),
        cv::Point2d(778, 967),

        // #003
        cv::Point2d(411, 778),
        cv::Point2d(734, 198),
        cv::Point2d(1085, 357),
        cv::Point2d(774, 968),

        // #004
        cv::Point2d(432, 792),
        cv::Point2d(730, 197),
        cv::Point2d(1080, 332),
        cv::Point2d(773, 968),

       // #005
        cv::Point2d(443, 795),
        cv::Point2d(732, 198),
        cv::Point2d(1073, 319),
        cv::Point2d(775, 966),

        // #006
        cv::Point2d(420, 742),
        cv::Point2d(735, 198),
        cv::Point2d(1041, 412),
        cv::Point2d(778, 967),

       // #007
        cv::Point2d(439, 726),
        cv::Point2d(738, 195),
        cv::Point2d(1022, 426),
        cv::Point2d(781, 965),

        // #008
        cv::Point2d(428, 736),
        cv::Point2d(736, 194),
        cv::Point2d(1032, 418),
        cv::Point2d(780, 965),        
    };

    // SolvePnP - r_cl, R_cl, t_cl, R_lc, t_lc
    cv::solvePnPRansac(lidar_points, image_points, K, dist_coeff, r_cl, t_cl);
    cv::Rodrigues(r_cl, R_cl);   
    R_lc = R_cl.t();
    t_lc = -R_cl.t() * t_cl;

    // std::cout << "############# PnP_lidar  #############" << std::endl;
    // std::cout << "[PnP_lidar] r_cl    : " << r_cl.t() << std::endl;    // r_cl    : rotation 3x1 vector,    according to "camera" coord to "lidar" coord
    // std::cout << "[PnP_lidar] R_cl    : " << R_cl     << std::endl;    // R_cl    : rotation 3x3 matrix,    according to "camera" coord to "lidar" coord
    // std::cout << "[PnP_lidar] t_cl    : " << t_cl.t() << std::endl;    // t_cl    : translation 3x1 vector, according to "camera" coord to "lidar" coord
    // std::cout << "[PnP_lidar] R_lc    : " << R_lc     << std::endl;    // R_lc    : rotation 3x3 matrix,    according to "lidar"  coord to "camera" coord
    // std::cout << "[PnP_lidar] t_lc    : " << t_lc.t() << std::endl;    // t_lc    : translation 3x1 vector, according to "lidar"  coord to "camera" coord
    }


    cv::Mat projectlidar2image(const pcl::PointCloud<pcl::PointXYZIR>::Ptr &filterCloud, cv::Mat& image, 
                            cv::Mat& r_cl, cv::Mat& t_cl, cv::Matx33d &K, std::vector<double> & dist_coeff, double filter_range[])
    {

        cv::Scalar RED     = cv::Scalar(0,  0, 255);
        cv::Scalar YELLOW  = cv::Scalar(0, 255, 255); 
        cv::Scalar GREEN   = cv::Scalar(0, 255, 0);
        cv::Scalar BLUE    = cv::Scalar(255, 0 ,0);
        cv::Scalar PURPLE  = cv::Scalar(255, 0, 100);


        cv::Mat image_result = image.clone();
        
        std::vector<cv::Scalar>  color_array;
        std::vector<cv::Point3d> points;
        std::vector<bool>        use_or_not;


        for (int idx = 0; idx < filterCloud->size(); idx++)
        {
            points.push_back(cv::Point3d(filterCloud->points[idx].x, filterCloud->points[idx].y, filterCloud->points[idx].z ));

            int rmd = filterCloud->points[idx].ring % 16;
            int qt = filterCloud->points[idx].ring / 16;
            // int temp2 = filterCloud->points[idx].ring % 5;

            switch (qt)
            {
                case 0:
                    color_array.push_back(returnColor(RED,RED,rmd));
                    break;
                case 1:
                    color_array.push_back(returnColor(RED,YELLOW,rmd));
                    break;
                case 2:
                    if(rmd < 8){
                        color_array.push_back(returnColor(YELLOW,GREEN,rmd*2));
                    }
                    else{
                        color_array.push_back(returnColor(GREEN,BLUE,(rmd-8)*2));
                    }
                    break;
                case 3:
                    color_array.push_back(returnColor(BLUE,PURPLE,rmd));
                    break;
            }
            use_or_not.push_back(  check_range(filterCloud->points[idx].x, filterCloud->points[idx].y, filterCloud->points[idx].z, filter_range) );

        }
    
        cv::Mat project_points;
        cv::projectPoints(points , r_cl, t_cl, K, dist_coeff, project_points);        
        
        // std::cout << filterCloud->size() << std::endl;

        for (int idx = 0; idx < filterCloud->size(); idx++)
        { 
            if(use_or_not[idx])
                cv::circle(image_result, cv::Point(project_points.at<double>(idx,0), project_points.at<double>(idx,1)), 3.0, color_array[idx], -1, -1, 0 );
        }

        return image_result;
    }

    bool check_range( double x, double y, double z, double filter_range[])  
    {
        bool use_or_not = false; // min_x, max_x, min_y, max_y, min_z, max_z
        if ( ( filter_range[0] < x) && (x < filter_range[1]) &&  ( filter_range[2] < y) && (y < filter_range[3]) && ( filter_range[4] < z) && (z < filter_range[5]) )
            use_or_not = true;
        return use_or_not;
    }

    cv::Scalar returnColor(const cv::Scalar & Color1, const cv::Scalar & Color2, const int & rmd)
    {
        cv::Scalar dColor;
        return Color1 + (Color2 - Color1) * rmd / 16;
    }

};