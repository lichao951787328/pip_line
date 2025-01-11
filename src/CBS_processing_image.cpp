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

// #define FEASIBLE_IMAGE
// #define FEASIBLE_IMAGE_CONTOUR
// 第一个return之前的为获取可通行的平面，思路是先转成世界坐标系下的点云，并通过世界坐标系下的点云坐标确定在相机坐标系下的障碍点，再在相机坐标系下的点云中删除那些可能发生碰撞的障碍点。再进行平面检测

// 第二个return是对处理结果进行修饰得到的平面和轮廓

// 如何画出落脚点？根据自己试出的步态点确定对应的像素位置，这是由相机内参确定。步态点先根据落脚点和平面算出每个落脚脚的角点的位置


int main(int argc, char** argv)
{
#ifdef FEASIBLE_IMAGE
    ros::init(argc, argv, "CBS_processing_image");
    ros::NodeHandle nh;
    ros::Publisher pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcd_pub", 1);
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/image_pub", 1);
    
    unsigned int default_colors[10][3] =
    {
        {255, 0, 0},
        {255, 255, 0},
        {100, 20, 50},
        {0, 30, 255},
        // {10, 255, 60},
        {80, 10, 100},
        {0, 255, 200},
        {10, 60, 60},
        {255, 0, 128},
        {60, 128, 128}
    };
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::io::loadPCDFile("/home/lichao/TCDS/src/pip_line/data/CBS_needed/org_pc1.pcd", cloud);

    // load原始点，转到世界坐标系下，寻找超过某个高度的点，标记在图像上，对图像上该区域进行膨胀，再再原始点云下去除这些点，再进行平面检测
    Eigen::Matrix4d T_world_camera = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_install_depth = Eigen::Matrix4d::Identity();
    T_install_depth(1, 3) = -0.001;
    T_install_depth(2, 3) = 0.026 - 0.0045;
    Eigen::Matrix4d T_hole_install = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R;
    R<<- 0.7071, 0, 0.7071,
        0, 1, 0,
        - 0.7071, 0, -0.7071;
    T_hole_install.block<3,3>(0,0) = R;
    T_hole_install(0, 3) = 0.07025;
    T_hole_install(2, 3) = 0.00424;
    Eigen::Matrix4d T_base_hole = Eigen::Matrix4d::Identity();
    T_base_hole(0, 3) = 0.05675;
    T_base_hole(2, 3) = 0.49123;
    Eigen::Matrix4d T_world_base = Eigen::Matrix4d::Identity();
    // 一定要注意根据实际高度调节，控制端反馈
    T_world_base.block<3,1>(0,3) = Eigen::Vector3d(0, 0, 0.71);
    T_world_camera = T_world_base * T_base_hole * T_hole_install * T_install_depth;

    pcl::PointCloud<pcl::PointXYZ> cloud_world;
    cv::Mat obstacle_mask(cloud.height, cloud.width, CV_8UC1, cv::Scalar(0));
    cout<<"cloud size: "<<cloud.width<<" "<<cloud.height<<endl;
    // 确定障碍点，这些障碍点是在相机坐标系下
    pcl::PointCloud<pcl::PointXYZ> obstacle_cloud;
    for (int i = 0; i < cloud.width; i++)
    {
        for (int j = 0; j < cloud.height; j++)
        {
            auto point = cloud.at(i, j);
            if (point.z < 0.1)
            {
                // cout<<"nan"<<endl;
                cloud_world.emplace_back(pcl::PointXYZ(NAN, NAN, NAN));
                continue;
            }
            

            Eigen::Vector3d po_w = T_world_camera.block<3,3>(0,0)* Eigen::Vector3d(point.x, point.y, point.z) + T_world_camera.block<3,1>(0,3);
            cloud_world.emplace_back(pcl::PointXYZ(po_w.x(), po_w.y(), po_w.z()));
            if (po_w.z() > 0.4)
            {
                obstacle_mask.at<uchar>(j, i) = 255;
                // obstacle_cloud.emplace_back(pcl::PointXYZ(po_w.x(), po_w.y(), po_w.z()));
                obstacle_cloud.emplace_back(point);
            }
        }
    }
    cloud_world.height = cloud.height;
    cloud_world.width = cloud.width;
    pcl::io::savePCDFileASCII("/home/lichao/TCDS/src/pip_line/data/CBS_needed/obstacle_cloud1.pcd", obstacle_cloud);
    // 对障碍点进行滤波。由于相机深度值并不可靠，所以引入统计滤波，去除异常值
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(obstacle_cloud.makeShared());
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    sor.filter(cloud_filtered);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_stat;
    sor_stat.setInputCloud(cloud_filtered.makeShared());
    sor_stat.setMeanK(30);
    sor_stat.setStddevMulThresh(0.5);
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered_stat;
    sor_stat.filter(cloud_filtered_stat);
    pcl::io::savePCDFileASCII("/home/lichao/TCDS/src/pip_line/data/CBS_needed/obstacle_cloud_filter.pcd", cloud_filtered_stat);

    // 根据障碍点，想原始点和障碍点都转到世界坐标系下，在通过距离去除一部分离障碍点较近的点
    for (int i = 0; i < cloud.width; i++)
    {
        for (int j = 0; j < cloud.height; j++)
        {
            auto point = cloud.at(i, j);           
            if (point.z < 0.1)
            {
                continue;
            }
            Eigen::Vector3d po_w = T_world_camera.block<3,3>(0,0)* Eigen::Vector3d(point.x, point.y, point.z) + T_world_camera.block<3,1>(0,3);
            for (auto & obstcle_point : cloud_filtered_stat)
            {
                Eigen::Vector3d obsacle = T_world_camera.block<3,3>(0,0)* Eigen::Vector3d(obstcle_point.x, obstcle_point.y, obstcle_point.z) + T_world_camera.block<3,1>(0,3);
                if ((po_w - obsacle).head(2).norm() < 0.25)
                {
                    cloud.at(i, j).x = 0;
                    cloud.at(i, j).y = 0;
                    cloud.at(i, j).z = 0;
                    break;
                }
            }
        }
    }
    
    pcl::io::savePCDFileASCII("/home/lichao/TCDS/src/pip_line/data/CBS_needed/raw_safe.pcd", cloud);
    
    plane_detection pd;
    pd.initial("/home/lichao/TCDS/src/pip_line/config/plane_fitter_pcd.ini");
    pd.detect(cloud);
    cout << "plane_num: " << pd.planes.size() << endl;

    for (int i = 0; i < pd.planes.size(); i++)
    {
        cv::imwrite("/home/lichao/TCDS/src/pip_line/data/CBS_needed/feasible_planes/feasible_image" + std::to_string(i) + ".jpg", pd.planes.at(i));
    }
    
    cv::imwrite("/home/lichao/TCDS/src/pip_line/data/CBS_needed/feasible_planes/feasible_image.jpg", pd.result);
    cv::imshow("result", pd.result);
    cv::waitKey(0);
    return 0;

#else

#ifdef FEASIBLE_IMAGE_CONTOUR
    unsigned int default_colors[10][3] =
    {
        {255, 0, 0},
        {255, 255, 0},
        {100, 20, 50},
        // {0, 30, 255},
        {10, 255, 60},
        {80, 10, 100},
        {0, 255, 200},
        {10, 60, 60},
        {255, 0, 128},
        {60, 128, 128}
    };
    cv::Mat img1 = cv::imread("/home/lichao/TCDS/src/pip_line/data/CBS_paper_image/combined.jpg");
    cv::Mat img2 = cv::imread("/home/lichao/TCDS/src/pip_line/data/CBS_needed/feasible_planes/feasible_image1.jpg");
    cv::Mat img3 = cv::imread("/home/lichao/TCDS/src/pip_line/data/CBS_needed/feasible_planes/feasible_image2.jpg");
    cv::Mat img4 = cv::imread("/home/lichao/TCDS/src/pip_line/data/CBS_needed/feasible_planes/feasible_image3.jpg");
    vector<cv::Mat> images;
    images.push_back(img1);
    images.push_back(img2);
    images.push_back(img3);
    images.push_back(img4);

    cv::Mat color1 = cv::Mat(img1.rows, img1.cols, CV_8UC3, cv::Scalar(default_colors[0][0], default_colors[0][1], default_colors[0][2]));
    cv::Mat color2 = cv::Mat(img1.rows, img1.cols, CV_8UC3, cv::Scalar(default_colors[1][0], default_colors[1][1], default_colors[1][2]));
    cv::Mat color3 = cv::Mat(img1.rows, img1.cols, CV_8UC3, cv::Scalar(default_colors[2][0], default_colors[2][1], default_colors[2][2]));
    cv::Mat color4 = cv::Mat(img1.rows, img1.cols, CV_8UC3, cv::Scalar(default_colors[3][0], default_colors[3][1], default_colors[3][2]));

    vector<cv::Mat> colors;
    colors.push_back(color1);
    colors.push_back(color2);
    colors.push_back(color3);
    colors.push_back(color4);

    cv::Mat colored_image(img1.rows, img1.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < images.size(); i++)
    {
        cv::Mat mask = images.at(i).clone();
        cv::cvtColor(mask, mask, cv::COLOR_BGR2GRAY);
        cv::threshold(mask, mask, 200, 255, cv::THRESH_BINARY);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11)); // 结构元素
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        vector<vector<cv::Point>> approxContours(contours.size());
        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::approxPolyDP(contours[i], approxContours[i], 5, true);
        }
        
        colors.at(i).copyTo(colored_image, mask);
        cv::drawContours(colored_image, approxContours, -1, cv::Scalar(0, 0, 255), 2);
    }
    cv::imshow("colored_image", colored_image);
    cv::imwrite("/home/lichao/TCDS/src/pip_line/data/CBS_needed/colored_image_feasible.jpg", colored_image);
    cv::waitKey(0);
#else
    unsigned int default_colors[10][3] =
    {
        // {255, 0, 0},
        {255, 255, 0},
        {100, 20, 50},
        // {0, 30, 255},
        {10, 255, 60},
        {80, 10, 100},
        {0, 255, 200},
        {10, 60, 60},
        {255, 0, 128},
        {60, 128, 128}
    };

    cv::Mat img1 = cv::imread("/home/lichao/TCDS/src/pip_line/data/CBS_needed/combined.jpg");
    cv::Mat img2 = cv::imread("/home/lichao/TCDS/src/pip_line/data/CBS_needed/plane00.jpg");
    
    cv::Mat img3 = cv::imread("/home/lichao/TCDS/src/pip_line/data/CBS_needed/plane01.jpg");
    cv::Mat img4 = cv::imread("/home/lichao/TCDS/src/pip_line/data/CBS_needed/plane02.jpg");

    cv::Mat img5 = cv::imread("/home/lichao/TCDS/src/pip_line/data/CBS_needed/plane01_2.jpg");
    // cv::Mat img6 = cv::imread("/home/lichao/TCDS/src/pip_line/data/CBS_needed/plane23.jpg");

    // cout<<"img1 size: "<<img1.size()<<endl;
    // cout<<"img2 size: "<<img2.size()<<endl;
    // cout<<"img3 size: "<<img3.size()<<endl;
    // cout<<"img4 size: "<<img4.size()<<endl;
    // cout<<"img5 size: "<<img5.size()<<endl;
    vector<cv::Mat> images;
    images.push_back(img1);
    images.push_back(img2);
    images.push_back(img3);
    images.push_back(img4);
    images.push_back(img5);
    // images.push_back(img6);
    cv::Mat color1 = cv::Mat(img1.rows, img1.cols, CV_8UC3, cv::Scalar(default_colors[0][0], default_colors[0][1], default_colors[0][2]));
    cv::Mat color2 = cv::Mat(img1.rows, img1.cols, CV_8UC3, cv::Scalar(default_colors[1][0], default_colors[1][1], default_colors[1][2]));
    cv::Mat color3 = cv::Mat(img1.rows, img1.cols, CV_8UC3, cv::Scalar(default_colors[2][0], default_colors[2][1], default_colors[2][2]));
    cv::Mat color4 = cv::Mat(img1.rows, img1.cols, CV_8UC3, cv::Scalar(default_colors[3][0], default_colors[3][1], default_colors[3][2]));
    cv::Mat color5 = cv::Mat(img1.rows, img1.cols, CV_8UC3, cv::Scalar(default_colors[4][0], default_colors[4][1], default_colors[4][2]));
    vector<cv::Mat> colors;
    colors.emplace_back(color1);
    colors.emplace_back(color2);
    colors.emplace_back(color3);
    colors.emplace_back(color4);
    colors.emplace_back(color5);

    cv::Mat colored_image(img1.rows, img1.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < images.size(); i++)
    {
        cv::Mat mask = images.at(i).clone();
        cv::cvtColor(mask, mask, cv::COLOR_BGR2GRAY);
        cv::threshold(mask, mask, 200, 255, cv::THRESH_BINARY);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11)); // 结构元素
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        vector<vector<cv::Point>> approxContours(contours.size());
        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::approxPolyDP(contours[i], approxContours[i], 5, true);
        }
        
        // cv::imshow("mask", mask);
        // cv::waitKey(0);

        // cv::Mat color(images[i].size(), images[i].type(), cv::Scalar(default_colors[i % 10][0], default_colors[i % 10][1], default_colors[i % 10][2]));
        // color.copyTo(colored_image, mask);
        colors.at(i).copyTo(colored_image, mask);
        cv::drawContours(colored_image, approxContours, -1, cv::Scalar(0, 0, 255), 2);
    }
    cv::imshow("colored_image", colored_image);
    cv::imwrite("/home/lichao/TCDS/src/pip_line/data/CBS_needed/colored_image_seg.jpg", colored_image);
    cv::waitKey(0);

    std::ifstream infile("/home/lichao/TCDS/src/pip_line/data/CBS_needed/steps_data.txt");
    std::string line;
    std::vector<std::vector<Eigen::Vector3d>> steps;

    while (std::getline(infile, line))
    {
        if (line[0] == '#')
        {
            continue;
        }

        std::istringstream iss(line);
        double x, y, z;
        if (!(iss >> x >> y >> z))
        {
            break;
        }

        Eigen::Vector3d point(x, y, z);
        if (steps.empty() || steps.back().size() == 4)
        {
            steps.emplace_back();
        }
        steps.back().emplace_back(point);
    }

    // 获取相机深度相机的内参矩阵
    double fx = 457.484375;
    double cx = 311.671875;
    double fy = 457.5078125;
    double cy = 254.09375;

    Eigen::Matrix4d T_world_camera = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_install_depth = Eigen::Matrix4d::Identity();
    T_install_depth(1, 3) = -0.001;
    T_install_depth(2, 3) = 0.026 - 0.0045;
    Eigen::Matrix4d T_hole_install = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R;
    R<<- 0.7071, 0, 0.7071,
        0, 1, 0,
        - 0.7071, 0, -0.7071;
    T_hole_install.block<3,3>(0,0) = R;
    T_hole_install(0, 3) = 0.07025;
    T_hole_install(2, 3) = 0.00424;
    Eigen::Matrix4d T_base_hole = Eigen::Matrix4d::Identity();
    T_base_hole(0, 3) = 0.05675;
    T_base_hole(2, 3) = 0.49123;
    Eigen::Matrix4d T_world_base = Eigen::Matrix4d::Identity();
    // 一定要注意根据实际高度调节，控制端反馈
    T_world_base.block<3,1>(0,3) = Eigen::Vector3d(0, 0, 0.71);
    T_world_camera = T_world_base * T_base_hole * T_hole_install * T_install_depth;
    Eigen::Matrix4d T_camera_world = T_world_camera.inverse();
    // 使用上述的矩阵将点转到像素坐标系下
    std::vector<cv::Point> pixel_points;
    for (const auto& step : steps)
    {
        for (const auto& point : step)
        {
            Eigen::Vector3d point_camera = T_camera_world.block<3,3>(0,0)* Eigen::Vector3d(point.x(), point.y(), point.z()) + T_camera_world.block<3,1>(0,3);

            double u = fx * point_camera.x() / point_camera.z() + cx;
            double v = fy * point_camera.y() / point_camera.z() + cy;
            cv::Point pixel_point(u, v);
            cout << "u: " << u << ", v: " << v << endl;
            pixel_points.push_back(pixel_point);
            cv::circle(colored_image, pixel_point, 5, cv::Scalar(255, 0, 0), -1);
            if (pixel_points.size() == 4)
            {
                for (int i = 0; i < 4; i++)
                {
                    cv::line(colored_image, pixel_points[i], pixel_points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
                }
                pixel_points.clear();
            }
        }
    }
    cv::imshow("colored_image", colored_image);
    cv::waitKey(0); 
#endif
    return 0;
#endif
}