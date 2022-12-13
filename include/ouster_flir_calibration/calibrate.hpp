#pragma once
#include "ouster_flir_calibration/my_global_header.hpp"

struct PointXYZR
{
    double x;
    double y;
    double z;
    int ring;
};


class CALIBRATE
{
private:
    ros::NodeHandle nh;

    // params.yaml
    std::string saveFILEDirectory;

    int num_PCD = 0;
    int num_IMG = 0;
    int num_PCD_pts  = 0;
    int num_IMG_pts  = 0;
    
    std::vector<cv::Point3d> lidar_points;
    std::vector<cv::Point2d> image_points;
    
    std::vector<double> K_coeff;
    std::vector<double> dist_coeff;
    std::vector<double> filter_limit;

    // pcl
    pcl::PointCloud<pcl::PointXYZI>::Ptr readPCD;    

public:

    CALIBRATE()
    {
        // params.yaml
        nh.param<std::string>("/ouster_flir_calibration/saveFILEDirectory", saveFILEDirectory, "/home/cmslab/catkin_ws/src/ouster_flir_calibration/DATA/");
        nh.param<std::vector<double>>("/ouster_flir_calibration/K", K_coeff, {0, 0, 0, 0, 0, 0, 0, 0, 0});
        nh.param<std::vector<double>>("/ouster_flir_calibration/D", dist_coeff, {0, 0, 0, 0, 0});
        nh.param<std::vector<double>>("/ouster_flir_calibration/filter_limit", filter_limit, {0, 0, 0, 0, 0, 0});


        // pcl
        readPCD.reset(new pcl::PointCloud<pcl::PointXYZI>());

        std::cout << "###################################################################################################" << std::endl;
        // (1) read PCD
        std::string path_PCD  = saveFILEDirectory + "2_CAPTURE/XYZI/";
        cv::String path1(path_PCD);
        std::vector<cv::String> str_PCD_list;
        cv::glob(path1, str_PCD_list, false);
        num_PCD = str_PCD_list.size();
        std::cout << "The number of PCD files located in the path is : " << num_PCD << std::endl;
        for (auto str_PCD : str_PCD_list) std::cout << str_PCD << std::endl;

        // (2) read IMG
        std::string path_IMG  = saveFILEDirectory + "2_CAPTURE/IMG/";
        cv::String path2(path_IMG);
        std::vector<cv::String> str_IMG_list;
        cv::glob(path2, str_IMG_list, false);
        num_IMG = str_IMG_list.size();
        std::cout << "The number of IMG files located in the path is : " << num_IMG << std::endl;
        for (auto str_IMG : str_IMG_list) std::cout << str_IMG << std::endl;

        // (3) read PCD points 
        std::string path_PCD_pts  = saveFILEDirectory + "3_POINTS/PCD/";
        cv::String path3(path_PCD_pts);
        std::vector<cv::String> str_PCD_pts_list;
        cv::glob(path3, str_PCD_pts_list, false);
        num_PCD_pts = str_PCD_pts_list.size();
        std::cout << "The number of PCD points located in the path is : " << num_PCD_pts << std::endl;
        for (auto str_PCD_pts : str_PCD_pts_list) std::cout << str_PCD_pts << std::endl;

        for (int i = 0; i < num_PCD_pts; i++)
        {
            char buffer[100];
            strcpy(buffer, str_PCD_pts_list[i].c_str());
            FILE* fin0 = fopen(buffer, "rt");
            while (!feof(fin0))
            {
                double x, y, z;
                if (!feof(fin0) && fscanf(fin0, "%lf %lf %lf", &x, &y, &z) == 3)
                    lidar_points.push_back(cv::Point3d(x, y, z));
            }
            fclose(fin0);
        }
        
        // (4) read IMG points 
        std::string path_IMG_pts  = saveFILEDirectory + "3_POINTS/IMG/";
        cv::String path4(path_IMG_pts);
        std::vector<cv::String> str_IMG_pts_list;
        cv::glob(path4, str_IMG_pts_list, false);
        num_IMG_pts = str_IMG_pts_list.size();
        std::cout << "The number of IMG points located in the path is : " << num_IMG_pts << std::endl;
        for (auto str_IMG_pts : str_IMG_pts_list) std::cout << str_IMG_pts << std::endl;
        for (int i = 0; i < num_IMG_pts; i++)
        {
            char buffer[100];
            strcpy(buffer, str_IMG_pts_list[i].c_str());
            FILE* fin1 = fopen(buffer, "rt");
            while (!feof(fin1))
            {
                int u, v;
                if (!feof(fin1) && fscanf(fin1, "%d %d", &u, &v) == 2)
                    image_points.push_back(cv::Point2d(u, v));
            }
            fclose(fin1);
        }
        std::cout << "###################################################################################################" << std::endl;


        // (5) solve PnP
        cv::Matx33d K( K_coeff[0], K_coeff[1], K_coeff[2], K_coeff[3], K_coeff[4], K_coeff[5], K_coeff[6], K_coeff[7], K_coeff[8]);
        cv::Mat r_cl, t_cl;
        cv::solvePnPRansac(lidar_points, image_points, K, dist_coeff, r_cl, t_cl);    

        cv::Mat R_cl, R_lc, t_lc;
        cv::Rodrigues(r_cl, R_cl);   
        R_lc = R_cl.t();
        t_lc = -R_cl.t() * t_cl;
        std::cout << "###################################################################################################" << std::endl;
        std::cout << "####################################   Calibration Results   ######################################" << std::endl;
        std::cout << "###################################################################################################" << std::endl;
        std::cout << "[PnP_lidar] r_cl    : " << r_cl.t() << std::endl;    // r_cl    : rotation 3x1 vector,    according to "camera" coord to "lidar" coord
        // std::cout << "[PnP_lidar] R_cl    : " << R_cl     << std::endl;    // R_cl    : rotation 3x3 matrix,    according to "camera" coord to "lidar" coord
        std::cout << "[PnP_lidar] t_cl    : " << t_cl.t() << std::endl;    // t_cl    : translation 3x1 vector, according to "camera" coord to "lidar" coord
        // std::cout << "[PnP_lidar] R_lc    : " << R_lc     << std::endl;    // R_lc    : rotation 3x3 matrix,    according to "lidar"  coord to "camera" coord
        // std::cout << "[PnP_lidar] t_lc    : " << t_lc.t() << std::endl;    // t_lc    : translation 3x1 vector, according to "lidar"  coord to "camera" coord


        std::cout << "###################################################################################################" << std::endl;
        std::cout << "####################################      Save Results       ######################################" << std::endl;
        std::cout << "###################################################################################################" << std::endl;

        for (int i = 0; i < num_IMG; i++)
        {
            int NUM_ZEROS = 3;
            std::string count_str = std::to_string(i+1);
            std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(),'0') + count_str;
            
            // get PCD & IMG
            pcl::io::loadPCDFile<pcl::PointXYZI> (str_PCD_list[i], *readPCD); 
            cv::Mat readIMG = cv::imread(str_IMG_list[i]);
            
            std::vector<PointXYZR> filterPCD;
            getRing(readPCD, filterPCD, filter_limit);

            std::vector<cv::Point3d> object_points;
            std::vector<cv::Scalar>  color_array;

            for (int j = 0; j < filterPCD.size(); j++)
            {
                double x_temp = filterPCD[j].x;
                double y_temp = filterPCD[j].y;
                double z_temp = filterPCD[j].z;
                object_points.push_back(cv::Point3d(x_temp, y_temp, z_temp));

                int ring = filterPCD[j].ring;
                cv::Scalar COLOR = getColor(ring);
                color_array.push_back(COLOR);
            }

            cv::Mat project_points;
            cv::projectPoints(object_points, r_cl, t_cl, K, dist_coeff, project_points);

            cv::Mat RESULT;
            cv::undistort(readIMG, RESULT, K, dist_coeff);

            for (int idx = 0; idx < filterPCD.size(); idx++)
            {
                double u = project_points.at<double>(idx,0);
                double v = project_points.at<double>(idx,1);

                if ( (0 <= u) && (u < readIMG.size().width) && (0 <= v) && (v < readIMG.size().height))
                    cv::circle(RESULT, cv::Point(u,v), 3.0, color_array[idx], -1, -1, 0);
            }

            // Save image
            std::string filename_image = saveFILEDirectory + "4_RESULT/" + "RESULT" + count_str_padded + ".png";
            cv::imwrite(filename_image, RESULT);
            std::cout << "Success to save RESULT" << count_str_padded << ".png file." << std::endl;
        }


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
