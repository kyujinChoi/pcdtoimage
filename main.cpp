#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry> 
#include <opencv2/opencv.hpp>

std::vector<std::string> splitString(std::string str, char Delimiter) 
{
    std::istringstream iss(str);             // istringstream에 str을 담는다.
    std::string buffer;                      // 구분자를 기준으로 절삭된 문자열이 담겨지는 버퍼
 
    std::vector<std::string> result;
 
    // istringstream은 istream을 상속받으므로 getline을 사용할 수 있다.
    while (getline(iss, buffer, Delimiter)) {
        buffer.erase(remove(buffer.begin(), buffer.end(), ' '), buffer.end());
        result.push_back(buffer);               // 절삭된 문자열을 vector에 저장
    }
 
    return result;
}

int main(int argc, char** argv)
{
    // if(argc != 3)
    // {
    //     std::cout << "usage : " << argv[0] << " <input pcd> <output dir>\n";
    //     return 1;
    // }
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/rp/Documents/local_data/b5_20240605_1_voxel0.01_wall_floor_reference_xyzrgb.pcd", *mapCloud) == -1) //* load the file
    // if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *mapCloud) == -1) //* load the file
    {
        //  /home/autoware/shared_dir/rosbog/20191026/20191008-100MB.pcd
        //  /home/autoware/shared_dir/pcd_output/separated_time.pcd
        PCL_ERROR("Couldn't read %s \n", argv[1]);
    }
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();

    int idx = 0;
    for (const auto& point : mapCloud->points) 
    {
        if (point.x < min_x) min_x = point.x;
        if (point.x > max_x) max_x = point.x;
        if (point.y < min_y) min_y = point.y;
        if (point.y > max_y) max_y = point.y;
    }
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (mapCloud);
    vg.setLeafSize (0.1f, 0.1f, 0.1f);
    vg.filter (*voxelCloud);
    float range_x = max_x - min_x;
    float range_y = max_y - min_y;

    int image_width = 1920;
    int image_height = (int)((image_width * range_y) / range_x);
    cv::Mat mapImage = cv::Mat::zeros(image_height, image_width, CV_8UC3);
    float scale_x = image_width / range_x;
    float scale_y = image_height / range_y;
    for(int i = 0; i < mapCloud->points.size(); i++)
    {
        int x = static_cast<int>((mapCloud->points[i].x - min_x) * scale_x);
        int y = image_height-static_cast<int>((mapCloud->points[i].y - min_y) * scale_y);

        // 이미지 범위 내에 좌표를 찍기
        if (x >= 0 && x < image_width && y >= 0 && y < image_height) {
            mapImage.at<cv::Vec3b>(y, x) = cv::Vec3b(mapCloud->points[i].b, mapCloud->points[i].g, mapCloud->points[i].r);  // 흰색 점
        }
    }
    std::string image_fn("/home/rp/Documents/local_data/");
    std::string txt_fn("/home/rp/Documents/local_data/");
    std::string pcd_fn("/home/rp/Documents/local_data/");
    image_fn += "/map_image.jpg";
    txt_fn += "/meta.txt";
    pcd_fn += "/map_data_voxelized.pcd";
    std::ofstream outFile(txt_fn);
    if (!outFile) {
        std::cerr << "Cannot open " << txt_fn << std::endl;
        return 1;
    }
    outFile << "image_width : " << std::to_string(image_width) << std::endl;
    outFile << "image_height : " << std::to_string(image_height) << std::endl;
    outFile << "range_x : " << std::to_string(range_x) << std::endl;
    outFile << "range_y : " << std::to_string(range_y) << std::endl;
    outFile << "min_x : " << std::to_string(min_x) << std::endl;
    outFile << "min_y : " << std::to_string(min_y) << std::endl;
    outFile << "int x = static_cast<int>((mapCloud->points[i].x - min_x) * (image_width / range_x));" << std::endl;
    outFile << "int y = image_height - static_cast<int>((mapCloud->points[i].y - min_y) * (image_height / range_y));" << std::endl;
    outFile.close();
    pcl::io::savePCDFileBinary<pcl::PointXYZRGB>(pcd_fn, *voxelCloud); //binary mod
    cv::imwrite(image_fn, mapImage);
    std::string window_name = "Bird's Eye View";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);  // 윈도우 크기 조정 가능하게 설정
    cv::resizeWindow(window_name, 1920,1280);  
    cv::imshow(window_name, mapImage);
    cv::waitKey(0);

    return 0;
}