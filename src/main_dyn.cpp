#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ouster_flir_calibration/dyn_reconfigConfig.h>

// dynamic reconfigure call back
void callback(ouster_flir_calibration::dyn_reconfigConfig &config, uint32_t level)
{
    // ROS_INFO("Dynamic parameter: %d", config.test_parameter);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "dynamic_reconfigure");

    // dynamic reconfigure
    dynamic_reconfigure::Server<ouster_flir_calibration::dyn_reconfigConfig> server;
    dynamic_reconfigure::Server<ouster_flir_calibration::dyn_reconfigConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();
    return 0;
}