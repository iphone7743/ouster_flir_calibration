#include <ros/ros.h>
#include "opencv2/opencv.hpp"

////////////////////////// How to Use ////////////////////////////
// 1. check calibration information
// 2. check path of /data/images
// 3. check path of /data/plate_points
/////////////////////////////////////////////////////////////////

std::vector<int> license_plate_x;
std::vector<int> license_plate_y;
int count_frame = 0;

void MouseEventHandler(int event, int x, int y, int flags, void* param)
{

    if (event == cv::EVENT_LBUTTONDOWN)
    {
        // Add the point to the given vector
        std::vector<cv::Point> *points_src = (std::vector<cv::Point> *)param;
        points_src->push_back(cv::Point(x, y));
        printf("A point at frame %04d (index: %zd) is selected at (%d, %d).\n", count_frame/4, points_src->size() -1, x, y);
        
        // Add the license_plate x & y in the buffer
        license_plate_x.push_back(x);
        license_plate_y.push_back(y);
        count_frame++;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "extract_plate_points");
    ros::NodeHandle nh;


    // Read all images from the path
    cv::String path("/home/cmslab/catkin_ws/src/ouster_flir_calibration/DATA/*.png");
    std::vector<cv::String> str;
    cv::glob(path, str, false);
    std::cout << "The number of images located in the path is : " << str.size() << std::endl;


    for (int img_num = 0; img_num < str.size(); img_num++ )
    {
        cv::Size plate_size(100, 200);

        // Prepare the rectified points
        std::vector<cv::Point> points_dst;
        points_dst.push_back(cv::Point(0, 0));
        points_dst.push_back(cv::Point(plate_size.width, 0));
        points_dst.push_back(cv::Point(plate_size.width, plate_size.height));
        points_dst.push_back(cv::Point(0, plate_size.height));

        // Load an image
        cv::Mat image_plate = cv::imread(str[img_num]);

        // Get the matched points from a user's mouse
        std::vector<cv::Point> points_src;
        cv::namedWindow("Calibration Board");
        cv::setMouseCallback("Calibration Board", MouseEventHandler, &points_src);
  
        while (points_src.size() < 4)
         {
           cv::Mat display = image_plate.clone();
           cv::rectangle(display, cv::Rect(cv::Point(10, 10), plate_size), cv::Vec3b(0, 0, 255), 2);
            size_t idx = cv::min(points_src.size(), points_dst.size() - 1);
             cv::circle(display, points_dst[idx] + cv::Point(10, 10), 5, cv::Vec3b(0, 255, 0), -1);
             cv::imshow("Calibration Board", display);
           if (cv::waitKey(1) == 27) cv::destroyAllWindows();  // 'ESC' key : Exit
           }
        if (points_src.size() < 4) return -1;


        // FILE* fout = fopen(cv::format("/home/cms/catkin_ws/src/radar_camera_fusion/data/plate_points/image%04d.xyz", img_num).c_str(), "wt");
           FILE* fout = fopen(cv::format("/home/cmslab/catkin_ws/src/ouster_flir_calibration/DATA/image%04d.xyz", img_num).c_str(), "wt");
        
        if (fout == NULL) return -1;
            for (int c = 0; c < license_plate_x.size(); c++)
             fprintf(fout, "%d %d\n", license_plate_x[c], license_plate_y[c]);
        fclose(fout);

        license_plate_x.clear();
        license_plate_y.clear();
    }

    return 0;
}