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



class EXTRACT_LIDAR_POINTS
{
private: 
    ros::NodeHandle nh;
    ros::Publisher  pub_PCD;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_IMG;

    // params.yaml
    std::string saveFILEDirectory;

    // pcl
    pcl::PointCloud<pcl::PointXYZI>::Ptr readPCD;       
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterPCD;
    pcl::PointCloud<pcl::PointXYZI>::Ptr extractPCD;
    sensor_msgs::PointCloud2::Ptr        extractPCD_ros;


    double filter_range[6]     = {+0.00,  6.00, -10.00, +10.00, -5.00, +5.00};  // min_x, max_x, min_y, max_y, min_z, max_z
    double filter_range_old[6] = { 0.00,  0.00,  0.00,  0.00,  0.00,  0.00};
    double min_x = 0.0;     double max_x = 0.0;
    double min_y = 0.0;     double max_y = 0.0;
    double min_z = 0.0;     double max_z = 0.0;

    std::string frame_id = "os_sensor";
    int num_readPCD = 0;
    int num_readIMG = 0;
    int sleep_time = 100000000; // nanoseconds, 10Hz
    bool extract_FLAG = true;
    bool extract_NEXT = false;
    std::vector<double> extracted_points = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

public : 

    EXTRACT_LIDAR_POINTS() : it_(nh)
    {
        // params.yaml
        nh.param<std::string>("/ouster_flir_calibration/saveFILEDirectory", saveFILEDirectory, "/home/cmslab/catkin_ws/src/ouster_flir_calibration/DATA/");
        nh.param<bool>("/ouster_flir_calibration/extract_NEXT", extract_NEXT, false);

        // pcl
        readPCD.reset(new pcl::PointCloud<pcl::PointXYZI>());
        filterPCD.reset(new pcl::PointCloud<pcl::PointXYZI>());
        extractPCD.reset(new pcl::PointCloud<pcl::PointXYZI>());
        extractPCD_ros.reset(new sensor_msgs::PointCloud2());

        // ros publisher
        pub_PCD = nh.advertise<sensor_msgs::PointCloud2>("/ouster_flir_calibration/LIDAR", 1);
        pub_IMG = it_.advertise("/ouster_flir_calibration/IMAGE", 1);



        // (1) saved PCD path in "str_readPCD_list"
        std::string path_PCD  = saveFILEDirectory + "2_CAPTURE/XYZI/";
        cv::String path1(path_PCD);
        std::vector<cv::String> str_readPCD_list;
        cv::glob(path1, str_readPCD_list, false);
        num_readPCD = str_readPCD_list.size();
        std::cout << "The number of PCD files located in the path is : " << num_readPCD << std::endl;
        for (auto str_readPCD : str_readPCD_list) std::cout << str_readPCD << std::endl;


        // (2) saved IMG path in "str_readPCD_list"
        std::string path_IMG  = saveFILEDirectory + "2_CAPTURE/IMG/";
        cv::String path2(path_IMG);
        std::vector<cv::String> str_readIMG_list;
        cv::glob(path2, str_readIMG_list, false);
        num_readIMG = str_readIMG_list.size();
        std::cout << "The number of IMG files located in the path is : " << num_readIMG << std::endl;
        for (auto str_readIMG : str_readIMG_list) std::cout << str_readIMG << std::endl;


        // (3) extract points
        for (int i = 0; i < num_readPCD; i++)
        {
            extract_FLAG = true;
            nh.setParam("/ouster_flir_calibration/extractNEXT", false);
            int NUM_ZEROS = 3;
            std::string count_str = std::to_string(i+1);
            std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(),'0') + count_str;
            std::cout << "###################################################################################################"                        << std::endl;
            std::cout << "#################################      CALIB_cloud_XYZI " << count_str_padded << "       #################################" << std::endl;
            std::cout << "#################################      CALIB_image_PNG  " << count_str_padded << "       #################################" << std::endl;
            std::cout << "###################################################################################################"                        << std::endl;

            // get PCD & IMG
            pcl::io::loadPCDFile<pcl::PointXYZI> (str_readPCD_list[i], *readPCD); 
            cv::Mat readIMG = cv::imread(str_readIMG_list[i]);

            while(ros::ok())
            {
                nh.getParam("/dynamic_reconfigure/min_x", min_x);
                nh.getParam("/dynamic_reconfigure/max_x", max_x);
                nh.getParam("/dynamic_reconfigure/min_y", min_y);
                nh.getParam("/dynamic_reconfigure/max_y", max_y);
                nh.getParam("/dynamic_reconfigure/min_z", min_z);
                nh.getParam("/dynamic_reconfigure/max_z", max_z);
                filter_range[0] = min_x; filter_range[1] = max_x;
                filter_range[2] = min_y; filter_range[3] = max_y;
                filter_range[4] = min_z; filter_range[5] = max_z;

                if( !check_change(filter_range, filter_range_old) )
                {
                    std::cout << "##################################       Extracted Points       ###################################"  << std::endl;
                    extract_FLAG = true;
                }

                passthrough(readPCD, filterPCD, filter_range);
                extracted_points.clear();
                if(extract_FLAG)
                extract_points(filterPCD, extractPCD, extracted_points);

                PCL2ROS<pcl::PointCloud<pcl::PointXYZI>::Ptr>(extractPCD, extractPCD_ros, frame_id);
                sensor_msgs::ImagePtr IMG_ros = cv_bridge::CvImage(std_msgs::Header(), "bgr8", readIMG).toImageMsg();

                pub_PCD.publish(*extractPCD_ros);
                pub_IMG.publish(IMG_ros);

                update_range(filter_range, filter_range_old);
                extract_FLAG = false;
                std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time));   

                nh.getParam("/ouster_flir_calibration/extractNEXT", extract_NEXT);
                if(extract_NEXT)
                {
                    // save points
                    std::string filename_PCD = saveFILEDirectory + "3_POINTS/PCD/" + "PCD" + count_str_padded + ".xyz";
                    FILE* fout = fopen(filename_PCD.c_str(), "wt");
                        fprintf(fout, "%f %f %f\n", extracted_points[0], extracted_points[1], extracted_points[2]);
                        fprintf(fout, "%f %f %f\n", extracted_points[3], extracted_points[4], extracted_points[5]);
                        fprintf(fout, "%f %f %f\n", extracted_points[6], extracted_points[7], extracted_points[8]);
                        fprintf(fout, "%f %f %f\n", extracted_points[9], extracted_points[10], extracted_points[11]);
                    fclose(fout);
                }
                if(extract_NEXT)
                    break;
            }
        }

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

    void extract_points(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_src,  pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_dst, std::vector<double> &extracted_points)
    {
        *cloud_dst = *cloud_src; // copy points

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
            // make all points intensity = 0
            cloud_dst->points[idx].intensity = 0;

            // get y_min idx
            if (cloud_dst->points[idx].y < min_y)
            {
                min_y = cloud_dst->points[idx].y;
                idx_min_y = idx;
            }

            // get y_max idx
            if (cloud_dst->points[idx].y > max_y)
            {
                max_y = cloud_dst->points[idx].y;
                idx_max_y = idx;
            }
            
            // get z_min idx
            if (cloud_dst->points[idx].z < min_z)
            {
                min_z = cloud_dst->points[idx].z;
                idx_min_z = idx;
            }

            // get z_max idx
            if (cloud_dst->points[idx].z > max_z)
            {
                max_z = cloud_dst->points[idx].z;
                idx_max_z = idx;
            }
        }

        // change intensity of extract points 
        cloud_dst->points[idx_max_y].intensity = 1000;
        cloud_dst->points[idx_max_z].intensity = 2000;
        cloud_dst->points[idx_min_y].intensity = 3000;
        cloud_dst->points[idx_min_z].intensity = 4000;
        
        extracted_points.push_back(cloud_dst->points[idx_max_y].x);     extracted_points.push_back(cloud_dst->points[idx_max_y].y);     extracted_points.push_back(cloud_dst->points[idx_max_y].z);
        extracted_points.push_back(cloud_dst->points[idx_max_z].x);     extracted_points.push_back(cloud_dst->points[idx_max_z].y);     extracted_points.push_back(cloud_dst->points[idx_max_z].z);
        extracted_points.push_back(cloud_dst->points[idx_min_y].x);     extracted_points.push_back(cloud_dst->points[idx_min_y].y);     extracted_points.push_back(cloud_dst->points[idx_min_y].z);
        extracted_points.push_back(cloud_dst->points[idx_min_z].x);     extracted_points.push_back(cloud_dst->points[idx_min_z].y);     extracted_points.push_back(cloud_dst->points[idx_min_z].z);

        std::cout.precision(6);
        std::cout << "[max_y point] : (" << cloud_dst->points[idx_max_y].x << ", " << cloud_dst->points[idx_max_y].y << ", " << cloud_dst->points[idx_max_y].z << ")" <<  std::endl;
        std::cout << "[max_z point] : (" << cloud_dst->points[idx_max_z].x << ", " << cloud_dst->points[idx_max_z].y << ", " << cloud_dst->points[idx_max_z].z << ")" <<  std::endl;
        std::cout << "[min_y point] : (" << cloud_dst->points[idx_min_y].x << ", " << cloud_dst->points[idx_min_y].y << ", " << cloud_dst->points[idx_min_y].z << ")" <<  std::endl;
        std::cout << "[min_z point] : (" << cloud_dst->points[idx_min_z].x << ", " << cloud_dst->points[idx_min_z].y << ", " << cloud_dst->points[idx_min_z].z << ")" <<  std::endl;
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

};
