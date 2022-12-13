#include "ouster_flir_calibration/extract_lidar_points.hpp"

int main (int argc, char** argv)
{
    ros::init (argc, argv, "extract_lidar_points");
    std::cout << "###################################################################################################" << std::endl;
    std::cout << "#############    Please type below command to extract lidar points (other terminal)   #############" << std::endl;
    std::cout << "#############     >>  rosparam set /ouster_flir_calibration/extractNEXT true  <<      #############" << std::endl;
    std::cout << "###################################################################################################" << std::endl;
    EXTRACT_LIDAR_POINTS elp;

    std::cout << "###################################################################################################" << std::endl;
    std::cout << "#########################    Finished, please shut down with 'CTRL + C'   #########################" << std::endl;
    std::cout << "###################################################################################################" << std::endl;
    return 0;
}