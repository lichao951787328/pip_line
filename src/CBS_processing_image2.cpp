#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>
#include <vector>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <peac/PEAC_plane_detection.hpp>
using namespace std;

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ> org_pc;
    pcl::io::loadPCDFile("/home/lichao/TCDS/src/pip_line/data/CBS_needed/pc_world_safe1.pcd", org_pc);
    plane_detection pd;
    pd.initial("/home/lichao/TCDS/src/pip_line/config/plane_fitter_pcd.ini");
    pd.detect(org_pc);
    cout << "plane_num: " << pd.planes.size() << endl;
    for (auto & image : pd.planes)
    {
        cv::imshow("image", image); 
        cv::waitKey(0);
    }
    cv::imshow("result", pd.result);
    cv::waitKey(0);
    return 0;
}