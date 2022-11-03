#include "ouster_flir_calibration/result.hpp"

int main (int argc, char** argv)
{
    ros::init (argc, argv, "result");
    RESULT result;
    ros::spin();
}