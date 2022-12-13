#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ouster_flir_calibration/dyn_reconfigConfig.h>

void callback(ouster_flir_calibration::dyn_reconfigConfig &config, uint32_t level)
{
    // std::cout <<       "min_x : " << config.min_x << "      max_x : " << config.max_x 
    //           << "      min_y : " << config.min_y << "      max_y : " << config.max_y
    //           << "      min_z : " << config.min_z << "      max_z : " << config.max_z << std::endl;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "dynamic_reconfigure");
    dynamic_reconfigure::Server<ouster_flir_calibration::dyn_reconfigConfig> server;
    dynamic_reconfigure::Server<ouster_flir_calibration::dyn_reconfigConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();
    return 0;
}