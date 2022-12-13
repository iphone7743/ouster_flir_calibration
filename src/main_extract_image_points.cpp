#include <ros/ros.h>
#include "opencv2/opencv.hpp"

std::vector<int> points_x;
std::vector<int> points_y;
std::string saveFILEDirectory;
int count_frame = 0;

void MouseEventHandler(int event, int x, int y, int flags, void* param)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        std::vector<cv::Point> *points_src = (std::vector<cv::Point> *)param;
        points_src->push_back(cv::Point(x, y));
        
        int temp = count_frame % 4;
        switch(temp)
        {
            case 0:
                printf("Selected 'max_y' points : (%d, %d).\n", x, y);
                break;
            case 1:
                printf("Selected 'max_z' points : (%d, %d).\n", x, y);
                break;
            case 2:
                printf("Selected 'min_y' points : (%d, %d).\n", x, y);
                break;
            case 3:
                printf("Selected 'min_z' points : (%d, %d).\n", x, y);
                break;
        }
        
        points_x.push_back(x);
        points_y.push_back(y);
        count_frame++;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "extract_image_points");
    ros::NodeHandle nh;

    // read all images from the path
    nh.param<std::string>("/ouster_flir_calibration/saveFILEDirectory", saveFILEDirectory, "/home/cmslab/catkin_ws/src/ouster_flir_calibration/DATA/");
    std::string path_IMG  = saveFILEDirectory + "2_CAPTURE/IMG_RECTIFY/*.png";
    cv::String path(path_IMG);
    std::vector<cv::String> str_readIMG_list;
    cv::glob(path, str_readIMG_list, false);
    int num_readIMG = str_readIMG_list.size();
    std::cout << "The number of IMG_RECTIFY files located in the path is : " << num_readIMG << std::endl;


    for (int img_num = 0; img_num < num_readIMG; img_num++ )
    {
        int NUM_ZEROS = 3;
        std::string count_str = std::to_string(img_num+1);
        std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(),'0') + count_str;

        std::cout << "###################################################################################################"                        << std::endl;
        std::cout << "#################################      CALIB_image_PNG  " << count_str_padded << "       #################################" << std::endl;
        std::cout << "###################################################################################################"                        << std::endl;
        
        cv::Size plate_size(100, 200);

        // prepare the rectified points
        std::vector<cv::Point> points_dst;
        points_dst.push_back(cv::Point(0, plate_size.height));
        points_dst.push_back(cv::Point(0, 0));
        points_dst.push_back(cv::Point(plate_size.width, 0));
        points_dst.push_back(cv::Point(plate_size.width, plate_size.height));

        // load an image
        cv::Mat image_plate = cv::imread(str_readIMG_list[img_num]);

        // get the matched points from a user's mouse
        std::vector<cv::Point> points_src;
        cv::namedWindow("Extract image points");
        cv::setMouseCallback("Extract image points", MouseEventHandler, &points_src);
  
        while (points_src.size() < 4)
         {
           cv::Mat display = image_plate.clone();
           cv::rectangle(display, cv::Rect(cv::Point(10, 10), plate_size), cv::Vec3b(0, 0, 255), 2);
            size_t idx = cv::min(points_src.size(), points_dst.size() - 1);
             cv::circle(display, points_dst[idx] + cv::Point(10, 10), 5, cv::Vec3b(0, 255, 0), -1);
             cv::imshow("Extract image points", display);
           if (cv::waitKey(1) == 27) cv::destroyAllWindows();  // 'ESC' key : Exit
           }
        if (points_src.size() < 4) return -1;

        // save points
        std::string filename_IMG = saveFILEDirectory + "3_POINTS/IMG/" + "IMG" + count_str_padded + ".xyz";
        FILE* fout = fopen(filename_IMG.c_str(), "wt");
        for (int c = 0; c < points_x.size(); c++)
             fprintf(fout, "%d %d\n", points_x[c], points_y[c]);
        fclose(fout);

        points_x.clear();
        points_y.clear();
    }

    return 0;
}