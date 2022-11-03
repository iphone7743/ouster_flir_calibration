#pragma once
#include "ouster_flir_calibration/my_global_header.hpp"
#include <dynamic_reconfigure/server.h>
#include <ouster_flir_calibration/dyn_reconfigConfig.h>



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



class CALIBRATE
{
private:
    ros::NodeHandle nh;
    ros::Publisher  pub_read_cloud;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr     readCloud;       
    sensor_msgs::PointCloud2::Ptr            readCloud_ros;
    pcl::PointCloud<pcl::PointXYZI>::Ptr     filterCloud;
    sensor_msgs::PointCloud2::Ptr            filterCloud_ros;
    pcl::PointCloud<pcl::PointXYZIR>::Ptr    readCloud_XYZIR;
    sensor_msgs::PointCloud2::Ptr            readCloud_XYZIR_ros;
    // 빠르게 P3P를 하기위한 클라우드, 추후 수정
    pcl::PointCloud<pcl::PointXYZI>::Ptr     pnpCloud;
    sensor_msgs::PointCloud2::Ptr            pnpCloud_ros;

    std::string frame_id = "map";
    std::string saveFILEDirectory;

    int sleep_time = 100000000; // nanoseconds, 10Hz
    
    // double filter_range[6]     = {+2.50, +4.00, -0.70, +0.80, -1.00, +0.35};  // min_x, max_x, min_y, max_y, min_z, max_z
    double filter_range[6]     = {+0.00,  6.00, -10.00, +10.00, -5.00, +5.00};  // min_x, max_x, min_y, max_y, min_z, max_z
    double filter_range_old[6] = { 0.00,  0.00,  0.00,  0.00,  0.00,  0.00};
    double min_x = 0.0;     double max_x = 0.0;
    double min_y = 0.0;     double max_y = 0.0;
    double min_z = 0.0;     double max_z = 0.0;

    


public:

    CALIBRATE()
    {
        readCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        readCloud_ros.reset(new sensor_msgs::PointCloud2());
        filterCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        filterCloud_ros.reset(new sensor_msgs::PointCloud2());
        readCloud_XYZIR.reset(new pcl::PointCloud<pcl::PointXYZIR>());
        readCloud_XYZIR_ros.reset(new sensor_msgs::PointCloud2());
        
        // 빠르게 P3P를 하기위한 클라우드, 추후 수정
        pnpCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        pnpCloud_ros.reset(new sensor_msgs::PointCloud2());


        pub_read_cloud = nh.advertise<sensor_msgs::PointCloud2>("/CALIBRATE/read_cloud", 1);
        nh.param<std::string>("/ouster_flir_calibration/saveFILEDirectory", saveFILEDirectory, "/home/cmslab/catkin_ws/src/ouster_flir_calibration/DATA/");


        pcl::io::loadPCDFile<pcl::PointXYZIR> ("/home/cmslab/catkin_ws/src/ouster_flir_calibration/DATA/CALIB_cloud_XYZIR002.pcd", *readCloud_XYZIR);
        getRing(readCloud_XYZIR, 2047);

        pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/cmslab/catkin_ws/src/ouster_flir_calibration/DATA/CALIB_cloud_XYZI002.pcd", *readCloud);
        PCL2ROS<pcl::PointCloud<pcl::PointXYZIR>::Ptr>(readCloud_XYZIR, readCloud_XYZIR_ros, frame_id);



        // Camera Calibration
        cv::Matx33d K(2340.375977  ,0.0         ,745.556014
                    ,0.0          ,2351.579554  ,577.711325
                    ,0.0          ,0.0          ,1.0        );
        std::vector<double> dist_coeff = { -0.137154    ,0.628614     ,0.004937    ,0.001995     ,0.0 };

        // SolvePnP
        cv::Mat r_cl, t_cl, R_cl, R_lc, t_lc;
        solvePnP_lidar_camera(r_cl, t_cl, R_cl, R_lc, t_lc, K, dist_coeff);
        
        // opencv
        cv::Mat img_input = cv::imread("/home/cmslab/catkin_ws/src/ouster_flir_calibration/DATA/CALIB_image002.png");
        cv::Mat img_result;


        passthrough(readCloud, filterCloud, filter_range);
        img_result = projectlidar2image(readCloud_XYZIR, img_input, r_cl, t_cl, K, dist_coeff, filter_range);
        cv::imshow("Project_result", img_result);
        cv::waitKey(0);


        // while(ros::ok())
        // {
        //     nh.getParam("/dynamic_reconfigure/min_x", min_x);
        //     nh.getParam("/dynamic_reconfigure/max_x", max_x);
        //     nh.getParam("/dynamic_reconfigure/min_y", min_y);
        //     nh.getParam("/dynamic_reconfigure/max_y", max_y);
        //     nh.getParam("/dynamic_reconfigure/min_z", min_z);
        //     nh.getParam("/dynamic_reconfigure/max_z", max_z);
        //     filter_range[0] = min_x; filter_range[1] = max_x;
        //     filter_range[2] = min_y; filter_range[3] = max_y;
        //     filter_range[4] = min_z; filter_range[5] = max_z;


        //     passthrough(readCloud, filterCloud, filter_range);
            
        //     // 이 함수는 일단 캘리브레이션을 위해서 3점을 뽑아내는 함수.  (가장 왼쪽점, 가장 위의점, 가장 오른쪽점)
        //     // 빠르게 P3P를 하기위한 클라우드, 추후 수정
        //     // getPnPpoints(filterCloud, pnpCloud);
        //     // PCL2ROS<pcl::PointCloud<pcl::PointXYZI>::Ptr>(pnpCloud, pnpCloud_ros, frame_id);
        //     PCL2ROS<pcl::PointCloud<pcl::PointXYZI>::Ptr>(filterCloud, filterCloud_ros, frame_id);


        //     if( ! check_change(filter_range, filter_range_old) )
        //         ROS_INFO("min_x : %f, max_x : %f, min_y : %f, max_y : %f, min_z : %f, max_z : %f", min_x, max_x, min_y, max_y, min_z, max_z);


        //     // publish topic
        //     // pub_read_cloud.publish(*pnpCloud_ros);
        //     // pub_read_cloud.publish(*readCloud_XYZIR_ros);
        //     pub_read_cloud.publish(*filterCloud_ros);
            
        //     update_range(filter_range, filter_range_old);
        //     std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time));   
        // }
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

    void getRing(pcl::PointCloud<pcl::PointXYZIR>::Ptr &readCloud_XYZIR, const int & check_idx)
    {
        int ring_before = readCloud_XYZIR->points[check_idx].ring;

        int width  = readCloud_XYZIR->width;     // width 1024
        int height = readCloud_XYZIR->height;    // height 64
        int idx_point = 0;
        
        ROS_INFO("[Selected] height : %d, width : %d, check_idx : %d", height, width, check_idx);
        ROS_INFO("Start to get Ring on XYZIR file...");

        for (int idx_ring = 0; idx_ring < height; idx_ring++)   // for each channel 
        {
            for (int idx_width = 0; idx_width < width; idx_width++)
            {
                readCloud_XYZIR->points[idx_point].ring = idx_ring;
                idx_point++;
            }
        }

        ROS_INFO("...Finish to get Ring!");

        int ring_after = readCloud_XYZIR->points[check_idx].ring;
        ROS_INFO("[Before] points at %d ring number : %d", check_idx, ring_before);
        ROS_INFO("[After]  points at %d ring number : %d", check_idx, ring_after);
    }

    template<typename T>
    bool check_change( const T filter_range[], const T filter_range_old[])
    {
        bool check = true;
        for (int idx = 0; idx < 6; idx++)
        {check = check * (filter_range[idx] ==  filter_range_old[idx]);}
        return check;
    }

    template<typename T>
    void update_range( T &filter_range, T &filter_range_old)
    {
        for (int idx =0; idx < 6; idx++)
            filter_range_old[idx] = filter_range[idx];
    }

    // 이 함수는 일단 캘리브레이션을 위해서 3점을 뽑아내는 함수.  (가장 왼쪽점, 가장 위의점, 가장 오른쪽점)
    // 빠르게 P3P를 하기위한 클라우드, 추후 수정
    void getPnPpoints(const  pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_src,  pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_dst)
    {
        *cloud_dst = *cloud_src;  // 값을 복사하여 동일하게 만듬

        float min_y = cloud_dst->points[0].y;
        int   idx_min_y = 0;
        float max_y = cloud_dst->points[0].y;
        int   idx_max_y = 0;
        float min_z = cloud_dst->points[0].z;
        int   idx_min_z = 0;
        float max_z = cloud_dst->points[0].z;
        int   idx_max_z = 0;
        
        for (int idx = 0; idx < cloud_dst->size(); idx++)
        {
            // 우선 모든 포인트들의 intensity를 0으로 만든다.
            cloud_dst->points[idx].intensity = 0;

            // y min 값을 비교 후, idx 출력
            if (cloud_dst->points[idx].y < min_y)
            {
                min_y = cloud_dst->points[idx].y;
                idx_min_y = idx;
            }

            // y max 값을 비교 후, idx 출력
            if (cloud_dst->points[idx].y > max_y)
            {
                max_y = cloud_dst->points[idx].y;
                idx_max_y = idx;
            }

            // z max 값을 비교 후, idx 출력
            if (cloud_dst->points[idx].z > max_z)
            {
                max_z = cloud_dst->points[idx].z;
                idx_max_z = idx;
            }
            
            // z min 값을 비교 후, idx 출력
            if (cloud_dst->points[idx].z < min_z)
            {
                min_z = cloud_dst->points[idx].z;
                idx_min_z = idx;
            }
        }

        // 이제 모든 intensity가 0이 되었을텐데, 특별하게 뽑아낸 idx들에 대해서만 intensity를 1000, 2000, 3000으로 바꿔준다.

        cloud_dst->points[idx_max_y].intensity = 1000;
        cloud_dst->points[idx_max_z].intensity = 2000;
        cloud_dst->points[idx_min_y].intensity = 3000;
        cloud_dst->points[idx_min_z].intensity = 4000;
        
        ROS_INFO("1. \t %f \t %f \t %f", cloud_dst->points[idx_max_y].x, cloud_dst->points[idx_max_y].y, cloud_dst->points[idx_max_y].z );
        ROS_INFO("2. \t %f \t %f \t %f", cloud_dst->points[idx_max_z].x, cloud_dst->points[idx_max_z].y, cloud_dst->points[idx_max_z].z );
        ROS_INFO("3. \t %f \t %f \t %f", cloud_dst->points[idx_min_y].x, cloud_dst->points[idx_min_y].y, cloud_dst->points[idx_min_y].z );
        ROS_INFO("4. \t %f \t %f \t %f", cloud_dst->points[idx_min_z].x, cloud_dst->points[idx_min_z].y, cloud_dst->points[idx_min_z].z );
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

    std::cout << "############# PnP_lidar  #############" << std::endl;
    std::cout << "[PnP_lidar] r_cl    : " << r_cl.t() << std::endl;    // r_cl    : rotation 3x1 vector,    according to "camera" coord to "lidar" coord
    std::cout << "[PnP_lidar] R_cl    : " << R_cl     << std::endl;    // R_cl    : rotation 3x3 matrix,    according to "camera" coord to "lidar" coord
    std::cout << "[PnP_lidar] t_cl    : " << t_cl.t() << std::endl;    // t_cl    : translation 3x1 vector, according to "camera" coord to "lidar" coord
    std::cout << "[PnP_lidar] R_lc    : " << R_lc     << std::endl;    // R_lc    : rotation 3x3 matrix,    according to "lidar"  coord to "camera" coord
    std::cout << "[PnP_lidar] t_lc    : " << t_lc.t() << std::endl;    // t_lc    : translation 3x1 vector, according to "lidar"  coord to "camera" coord

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