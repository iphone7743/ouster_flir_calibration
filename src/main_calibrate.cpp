#include "ouster_flir_calibration/calibrate.hpp"

int main (int argc, char** argv)
{
    ros::init (argc, argv, "calibrate");
    CALIBRATE calibrate;
    ros::spin();
}