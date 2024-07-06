/*
 * @Author: lichao951787328 951787328@qq.com
 * @Date: 2024-06-07 11:01:20
 * @LastEditors: lichao951787328 951787328@qq.com
 * @LastEditTime: 2024-06-24 23:32:56
 * @FilePath: /pip_line/src/pip_line.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: lichao951787328 951787328@qq.com
 * @Date: 2024-06-07 11:01:20
 * @LastEditors: lichao951787328 951787328@qq.com
 * @LastEditTime: 2024-06-07 11:31:16
 * @FilePath: /pip_line/src/pip_line.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <pip_line/pip_line.h>
#include <grid_map_core/GridMap.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <PEAC_AHFP_pl/plane_fitter_pcl_AHFP.hpp>
#include <opencv2/opencv.hpp>

#include <tf2/buffer_core.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <glog/logging.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <grid_map_filters/MedianFillFilter.hpp>
#include <diy_msgs/footSteps.h>
#include <diy_msgs/avoidPointsMsg.h>
#include "plane_detection/load_parameter.h"
#include <plane_detection/plane_segmentation.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <grid_map_cv/GridMapCvConverter.hpp>
// #include <plane_detection/plane_new.h>

#include <ros/package.h>
#include <ctime>
void initial_package_path(string package_name, string & package_path)
{
  package_path = ros::package::getPath(package_name);
  // 检查是否成功找到包路径
  if (package_path.empty()) {
      std::cerr << "Error: Could not find package " << package_name << std::endl;
  }
  cout<<"package path: "<<package_path<<endl;
}

pip_line::pip_line(ros::NodeHandle & n):nh(n)
{
    pointcloud_sub = nh.subscribe("/camera/depth/color/points", 1, &pip_line::pointcloud_callback, this);
    raw_heightmap_pub = nh.advertise<grid_map_msgs::GridMap>("raw_heightmap", 1);
    // seg_result_image_pub = nh.advertise<sensor_msgs::Image>("seg_result", 1);
    // footsteps_pub = nh.advertise<diy_msgs::footSteps>("footsteps", 1);

    // timer = nh.createTimer(ros::Duration(1), &pip_line::timerCallback, this);

    grid_map::Length length(4, 2);
    grid_map::Position position(2, 0);
    map.setGeometry(length, 0.02, position);
    map.add("elevation", NAN);

    
    // is_finish = false;
    // helios 使用的变换矩阵
    // Eigen::Matrix4d T_velodyne_camera = Eigen::Matrix4d::Identity();
    // Eigen::Matrix4d T_base_velodyne = Eigen::Matrix4d::Identity();
    // Eigen::Matrix4d T_world_base = Eigen::Matrix4d::Identity();
    // T_velodyne_camera.block<3,3>(0,0) = Eigen::Quaterniond(0.229184, 0, 0.973383, 0).toRotationMatrix();
    // T_velodyne_camera.block<3,1>(0,3) = Eigen::Vector3d(0.15, 0, -0.4024);
    // T_base_velodyne.block<3,1>(0,3) = Eigen::Vector3d(-0.01, 0, 0.7217);
    // T_world_base.block<3,1>(0,3) = Eigen::Vector3d(0, 0, 0.69);
    // T_world_camera = T_world_base * T_base_velodyne * T_velodyne_camera;

    // 45度L515相机使用的变换矩阵
    Eigen::Matrix4d T_install_depth = Eigen::Matrix4d::Identity();
    T_install_depth(1, 3) = -0.001;
    T_install_depth(2, 3) = 0.026 - 0.0045;

    Eigen::Matrix4d T_hole_install = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R;
    R<<0, -0.5, 0.866,
        1, 0, 0,
        0, -0.866, -0.5;
    T_hole_install.block<3,3>(0,0) = R;
    T_hole_install(0, 3) = 0.06739;
    T_hole_install(2, 3) = 0.01115;

    Eigen::Matrix4d T_base_hole = Eigen::Matrix4d::Identity();
    T_base_hole(0, 3) = 0.05675;
    T_base_hole(2, 3) = 0.49123;
    Eigen::Matrix4d T_world_base = Eigen::Matrix4d::Identity();
    T_world_base.block<3,1>(0,3) = Eigen::Vector3d(0, 0, 0.75);
    T_world_camera = T_world_base * T_base_hole * T_hole_install * T_install_depth;
}

void preprocessing_bk(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
    // clock_t start1 = clock();
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // *cloud = *cloud_in;
    
    // // 去除这个过程
    // // GaussPointCloud(cloud_in, cloud);
    // // pcl::io::savePCDFile("/home/bhr/catkin_beijing4th/src/elevation_mapping/triggerHelios/debug_data/" + std::to_string(cloud_index) + ".pcd", *cloud);
    // // cloud_index++;
    // std::cout<<cloud->width<<" "<<cloud->height<<std::endl;
    // clock_t end1 = clock();
    // // cout<<"preprocess1 cost: "<<(double)(end1 - start1)/CLOCKS_PER_SEC<<std::endl;

    // // 去掉顶上一部分
    // // pcl::io::savePCDFile("/home/bhr/TCDS/src/pip_line/data/raw_points.pcd", *cloud);
    // pcl::PointCloud<pcl::PointXYZ> stable_points;
    // // 注意相机方向
    // clock_t start2 = clock();
    // for (int i = 100; i < cloud->width - 230; i++)
    // {
    //     for (int j = 30; j < cloud->height - 30; j++)
    //     {
    //         if (!std::isnan(cloud->at(j * cloud->width + i).z))
    //         {
    //             stable_points.emplace_back(cloud->at(j * cloud->width + i));
    //         }
    //     }
    // }
    // clock_t end2 = clock();
    // cout<<"preprocess2 cost: "<<(double)(end2 - start2)/CLOCKS_PER_SEC<<std::endl;
    // pcl::io::savePCDFile("/home/bhr/TCDS/src/pip_line/data/stable_points.pcd", stable_points);

    clock_t start3 = clock();
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;

    // 设置体素滤波器的输入点云
    // 这个地方可以选择使用平面点云还是所有点云
    voxel_grid_filter.setInputCloud(cloud_in);

    // 设置体素大小（体素的边长）这个值不能设成0.01，否则太耗时
    voxel_grid_filter.setLeafSize(0.018f, 0.018f, 0.018f);  // 设置为0.01米

    // 执行体素滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid_filter.filter(*filtered_cloud);

    clock_t end3 = clock();
    // cout<<"preprocess3 cost: "<<(double)(end3 - start3)/CLOCKS_PER_SEC<<std::endl;
    // cout<<"after filter: "<<filtered_cloud->size()<<endl;

    clock_t start4 = clock();
    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud(filtered_cloud);

    // // 设置过滤轴和范围（这里以Z轴为例，过滤掉Z轴范围在[0.0, 1.0]之外的点）
    pass.setFilterFieldName("x");// 相机转动之前是y，转动之后是x
    // // 这个值需要根据数据调节，后面上楼梯肯定会改
    pass.setFilterLimits(-4, 2);

    // // 执行直通滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr pass_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*pass_filtered_cloud);
    clock_t end4 = clock();
    // cout<<"preprocess4 cost: "<<(double)(end4 - start4)/CLOCKS_PER_SEC<<std::endl;
    // 飞点去除
    // cout<<"after pass filter: "<<pass_filtered_cloud->size()<<endl;
    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    // ne.setInputCloud(pass_filtered_cloud);// 注意衔接的输入输出

    // // 创建KdTree用于法向量估计
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // ne.setSearchMethod(tree);

    // // 设置搜索半径，用于确定每个点的邻域
    // ne.setRadiusSearch(0.08);  // 设置为0.03米

    // // 计算法向量
    // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // ne.compute(*normals);
    // pcl::PointCloud<pcl::PointXYZ> result;
    // for (size_t i = 0; i < pass_filtered_cloud->size(); i++)
    // {
    //     Eigen::Vector3d p(pass_filtered_cloud->at(i).x, pass_filtered_cloud->at(i).y, pass_filtered_cloud->at(i).z);
    //     Eigen::Vector3d n(normals->at(i).normal_x, normals->at(i).normal_y, normals->at(i).normal_z);
    //     if (std::abs(p.normalized().dot(n)) > 0.3)// 虽然这是一个阈值，但应该能满足要求
    //     {
    //         result.emplace_back(pass_filtered_cloud->at(i));
    //     }
    // }
    // cout<<"after Normal filter: "<<result.size()<<endl;
    clock_t start5 = clock();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud(result.makeShared());
    sor.setInputCloud(pass_filtered_cloud);
    
    // 设置统计滤波器参数
    sor.setMeanK(50);  // 设置邻域中点的数量
    sor.setStddevMulThresh(1.0);  // 设置标准差的倍数阈值

    // 应用统计滤波器
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(cloud_out);
    clock_t end5 = clock();
    cout<<"preprocess5 cost: "<<(double)(end5 - start5)/CLOCKS_PER_SEC<<std::endl;
    // LOG(INFO)<<"after StatisticalOutlierRemoval filter: "<<cloud_out.size();

}


pcl::PointCloud<pcl::PointXYZ> pip_line::gridMap2Pointcloud(grid_map::GridMap & map)
{
    pcl::PointCloud<pcl::PointXYZ> pc;
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            grid_map::Index index(i, j);
            grid_map::Position3 p3;
            if (map.getPosition3("elevation", index, p3))
            {
                if (!std::isnan(p3.z()))
                {
                    pc.emplace_back(pcl::PointXYZ(p3.x(), p3.y(), p3.z()));
                }
                else
                {
                    pc.emplace_back(pcl::PointXYZ(NAN, NAN, NAN));
                }
            }
            else
            {
                pc.emplace_back(pcl::PointXYZ(NAN, NAN, NAN));
            }
        }
    }
    pc.width = map.getSize().y();
    pc.height = map.getSize().x();
    return pc;
}


void pip_line::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr msg)
{
    // 构建高程图
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *pc);
    LOG(INFO)<<pc->size();
    clock_t start = clock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmppc(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto & point : *pc)
    {
        if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z))
        {
            tmppc->emplace_back(pcl::PointXYZ(point.x, point.y, point.z));
        }
    }
    
    // auto start_o = clock();
    pcl::PointCloud<pcl::PointXYZ> pub_cloud;

    
    preprocessing_bk(tmppc, pub_cloud);
    pcl::io::savePCDFileASCII("/home/lichao/TCDS/src/pip_line/data/processed.pcd", pub_cloud);

    pcl::PointCloud<pcl::PointXYZ> pc_world;
    LOG(INFO)<<pub_cloud.size();
    for (auto & point : pub_cloud)
    {
        Eigen::Vector3d po_w = T_world_camera.block<3,3>(0,0)* Eigen::Vector3d(point.x, point.y, point.z) + T_world_camera.block<3,1>(0,3);
        pc_world.emplace_back(pcl::PointXYZ(po_w.x(), po_w.y(), po_w.z()));
        grid_map::Index index;
        if (map.getIndex(po_w.head(2), index))
        {
            map["elevation"](index.x(), index.y()) = po_w.z();
        }
    }
    pcl::io::savePCDFileASCII("/home/lichao/TCDS/src/pip_line/data/pc_world.pcd", pc_world);
    grid_map_msgs::GridMap map_msg;
    grid_map::GridMapRosConverter::toMessage(map, map_msg);
    map_msg.info.header.frame_id = "map";
    raw_heightmap_pub.publish(map_msg);

    const float minValue = map.get("elevation").minCoeffOfFinites();
    const float maxValue = map.get("elevation").maxCoeffOfFinites();
    cv::Mat originalImage;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "elevation", CV_8UC1, minValue, maxValue, originalImage);
    // cv::imshow("originalImage", originalImage);
    // cv::waitKey(0);
    // 对空洞进行补齐
    // 创建一个标记小区域的掩码
    cv::Mat smallRegionsMask = cv::Mat::zeros(originalImage.size(), CV_8UC1);

    // // 标记连接组件
    cv::Mat labels, stats, centroids;
    int nLabels = cv::connectedComponentsWithStats(originalImage == 0, labels, stats, centroids, 8, CV_32S);

    // 遍历每个连接组件
    for (int label = 1; label < nLabels; ++label)
    {
        int area = stats.at<int>(label, cv::CC_STAT_AREA);

        if (area < 40)
        {
            // 对小区域进行标记
            cv::Mat mask = (labels == label);
            smallRegionsMask.setTo(255, mask);
        }
    }
    // cv::imshow("smallRegionsMask", smallRegionsMask);
    // cv::waitKey(0);

    // // 使用掩码进行图像修复
    cv::Mat inpainted;
    cv::inpaint(originalImage, smallRegionsMask, inpainted, 5, cv::INPAINT_TELEA);
    // cv::imshow("inpainted", inpainted);
    // cv::waitKey(0);

    map.erase("elevation");
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(inpainted, "elevation", map, minValue, maxValue);

    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            if (originalImage.at<uchar>(i, j) == 0 && smallRegionsMask.at<uchar>(i, j) == 0)
            {
                map["elevation"](i, j) = NAN;
            }
        }
    }
    // is_finish = false;
    // // 使用地图转成有序点云
    pcl::PointCloud<pcl::PointXYZ> org_pc = gridMap2Pointcloud(map);
    pcl::io::savePCDFileASCII("/home/lichao/TCDS/src/pip_line/data/submap.pcd", org_pc);
    
    orginazed_points raw_points;
    raw_points.initialByPCL(org_pc);
    size_t width = org_pc.width;
    size_t height = org_pc.height;
    LOG(INFO)<<width<<" "<<height;
    LOG(INFO)<<raw_points.width<<" "<<raw_points.height;
    string package_path = "/home/lichao/TCDS/src/pip_line";
    // string package_path = "/home/lichao/TCDS/src/pip_line";
    // // initial_package_path("pip_line", package_path);
    parameter param;
    load_parameter(param, package_path + "/config/parameter.xml");
    std::cout<<"load parameter finish"<<std::endl;
    param.initial(std::max(raw_points.height, raw_points.width));
    LOG(INFO)<<"param.quatree_width: "<<param.quatree_width;
    Eigen::Matrix4f T_I = Eigen::Matrix4f::Identity();
    quatree::node::setStaticMember(width, height, param.quatree_width, T_I, raw_points, param);
    quatree::quatree qq(raw_points, param);
    plane_segmentation ps(height, width, &qq, raw_points, param);
    vector<plane_info> planes = ps.getPlaneResult();
    cv::Mat result = ps.getSegResult();
    // cv::imshow("result", result);
    // cv::waitKey(0);
    cv::imwrite("/home/lichao/TCDS/src/pip_line/data/result.png", result);
    seg_result_image = result;
    vector<cv::Mat> single_results = ps.getSegResultSingle();
    LOG(INFO)<<single_results.size();

    // 去除地面
    vector<plane_info> slope_plane, step_plane;
    for (auto & plane : planes)
    {
        if (plane.center.z() < 0.02 && plane.normal.z() > 0.98)
        {
            LOG(INFO)<<"it is ground";
            continue;
        }
        if (plane.normal.z() > 0.98 && plane.center.z() > 0.02)
        {
            LOG(INFO)<<"it is step";
            step_plane.emplace_back(plane);
        }
        if (plane.normal.z() < 0.98)
        {
            LOG(INFO)<<"it is slope";
            slope_plane.emplace_back(plane);
        }
    }
    
    if (step_plane.size() > 1)
    {
        double distance = 0;
        int dis_num = 0;
        for (int i = 0; i < step_plane.size() - 1; i++)
        {
            distance += abs((step_plane.at(i).center - step_plane.at(i+1).center).dot(step_plane.at(i).normal));
            dis_num++;
        }
        LOG(INFO)<<"step height: "<<distance/dis_num;
    }
    else if (step_plane.size() == 1)
    {
        LOG(INFO)<<"just have 1 plane";
    }
    else
    {
        LOG(INFO)<<"not have plane";
    }
    
    if(!step_plane.empty())
    {
        Eigen::Vector3f plane_normal = step_plane.begin()->normal;

        for(auto & slope_plane : slope_plane)
        {
            float dot = plane_normal.dot(slope_plane.normal);
            LOG(INFO)<<std::acos(dot);
        }   
    }
}


// void pip_line::timerCallback(const ros::TimerEvent & event)
// {
//     if (is_finish)
//     {
//         grid_map_msgs::GridMap map_msg;
//         grid_map::GridMapRosConverter::toMessage(map, map_msg);
//         map_msg.info.header.frame_id = "map";
//         raw_heightmap_pub.publish(map_msg);

//         grid_map_msgs::GridMap feasible_map_msg;
//         grid_map::GridMapRosConverter::toMessage(feasible_map, feasible_map_msg);
//         feasible_map_msg.info.header.frame_id = "map";
//         feasible_map_pub.publish(feasible_map_msg);

//         // sensor_msgs::PointCloud2 raw_heightmap_pc;
//         // pcl::toROSMsg(org_pc, raw_heightmap_pc);
//         // raw_heightmap_pc.header.frame_id = "map";
//         // raw_heightmap_pc_pub.publish(raw_heightmap_pc);

//         std::vector<cv::Point> white_points;
//         cv::findNonZero(upper_body_image, white_points);
//         for (auto & cv_p : white_points)
//         {
//             height_map_upper["elevation"](cv_p.y, cv_p.x) = map["elevation"](cv_p.y, cv_p.x);
//         }
        
//         white_points.clear();
//         cv::findNonZero(knee_image, white_points);
//         for (auto & cv_p : white_points)
//         {
//             height_map_lower["elevation"](cv_p.y, cv_p.x) = map["elevation"](cv_p.y, cv_p.x);
//         }

//         grid_map_msgs::GridMap upper_msg;
//         grid_map::GridMapRosConverter::toMessage(height_map_upper, upper_msg);
//         upper_msg.info.header.frame_id = "map";
//         height_map_upper_pub.publish(upper_msg);

//         grid_map_msgs::GridMap lower_msg;
//         grid_map::GridMapRosConverter::toMessage(height_map_lower, lower_msg);
//         lower_msg.info.header.frame_id = "map";
//         height_map_lower_pub.publish(lower_msg);

//         sensor_msgs::ImagePtr result = cv_bridge::CvImage(std_msgs::Header(), "bgr8", seg_result_image).toImageMsg();

//         result->header.frame_id = "map";
//         seg_result_image_pub.publish(result);
//         // grid_map_msgs::GridMap foot_msg;
//         // grid_map::GridMapRosConverter::toMessage(height_map_foot, foot_msg);
//         // foot_msg.info.header.frame_id = "map";
//         // height_map_foot_pub.publish(foot_msg);

//         // grid_map_msgs::GridMap height_msg;
//         // grid_map::GridMapRosConverter::toMessage(plane_map, height_msg);
//         // height_msg.info.header.frame_id = "map";
//         // planes_all.publish(height_msg);

//         // grid_map_msgs::GridMap cutted_msg;
//         // grid_map::GridMapRosConverter::toMessage(plane_cutted, cutted_msg);
//         // cutted_msg.info.header.frame_id = "map";
//         // planes_cutted.publish(cutted_msg);

//         // planes_polygon_pub.publish(planes_msg);
//         // planes_polygon_cutted_pub.publish(planes_cutted_msg);
//     }
    
// }


// void getPlaneInfo(grid_map::GridMap & map, cv::Mat & image, Eigen::Vector3d & normal, double & d)
// {
//     Eigen::Vector3d center = Eigen::Vector3d::Zero();
//     vector<Eigen::Vector3d> points;
//     for (int i = 0; i < image.rows; i++)
//     {
//         for (int j = 0; j < image.cols; j++)
//         {
//             if (image.at<uchar>(i, j) == 255)
//             {
//                 grid_map::Position3 p3;
//                 if (map.getPosition3("elevation", grid_map::Index(i, j), p3))
//                 {
//                     points.emplace_back(p3);
//                     center += p3;
//                 }
//             }
//         }
//     }
//     center = center/points.size();
    
//     Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
//     for (auto & point : points)
//     {
//         M += (point - center) * (point - center).transpose();
//     }
//     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(M);
//     auto eigenvectors = eigensolver.eigenvectors();
//     normal = eigenvectors.col(0);
//     d = -(normal.dot(center));
// }

// 使用平面地形高程图检测平面，获取轮廓，在根据像素轮廓，获取平面点，再根据三维点画出多边形
// void pip_line::draw_planes(grid_map::GridMap map, visualization_msgs::MarkerArray & msg, double r, double g, double b)
// {
//     pcl::PointCloud<pcl::PointXYZ> org_pc = gridMap2Pointcloud(map);
//     AHFP_pl::PlanarContourExtraction pce(org_pc);
//     pce.run();
//     vector<cv::Mat> planes = pce.getSegPlanes();
//     visualization_msgs::MarkerArray polygons;
//     int index = 0;
//     for (auto & image : planes)
//     {
//         visualization_msgs::Marker polygon;
//         polygon.header.frame_id = "map";
//         polygon.header.stamp = ros::Time::now();
//         polygon.ns = "polygons";
//         polygon.id = index;
//         index++;
//         polygon.type = visualization_msgs::Marker::LINE_LIST;
//         polygon.action = visualization_msgs::Marker::ADD;
//         polygon.pose.orientation.w = 1.0;
//         polygon.scale.x = 0.03;  // 线条的宽度
//         polygon.color.r = r;
//         polygon.color.g = g;
//         polygon.color.b = b;
//         polygon.color.a = 1.0;
//         Eigen::Vector3d normal; 
//         double d;
//         getPlaneInfo(map, image, normal, d);
//         // cv::Mat blurred;
//         // cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
//         std::vector<std::vector<cv::Point>> contours;
//         std::vector<cv::Vec4i> hierarchy;
//         cv::findContours(image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
//         auto itr = contours.begin();    //使用迭代器去除噪声轮廓
//         while (itr != contours.end())
//         {
//             auto area = cv::contourArea(*itr);  //获得轮廓面积
//             if (area < 8)    //删除较小面积的轮廓
//             {
//                 itr = contours.erase(itr); //itr一旦erase，需要重新赋值
//             }
//             else
//             {
//                 itr++;
//             }
//         }
//         std::vector<std::vector<cv::Point>> approxContours(contours.size());
//         for (size_t i = 0; i < contours.size(); i++) {
//             double epsilon = 0.01 * cv::arcLength(contours[i], true);
//             cv::approxPolyDP(contours[i], approxContours[i], epsilon, true);
//         }
//         // 根据像素坐标系求轮廓点的xy值，在根据平面信息求z值  
//         // vector<Eigen::Vector3d> contour;
//         for (auto & contour : approxContours)
//         {
//             for (int i = 0; i < contour.size(); i++)
//             {
//                 grid_map::Position position;
//                 if (map.getPosition(grid_map::Index(contour.at(i%contour.size()).y, contour.at(i%contour.size()).x), position))
//                 {
//                     double z = (d - normal(0) * position.x() - normal(1)*position.y())/normal(2);
//                     geometry_msgs::Point p;
//                     p.x = position.x();
//                     p.y = position.y();
//                     p.z = z;
//                     polygon.points.emplace_back(p);
//                 }
//                 if (map.getPosition(grid_map::Index(contour.at((i+1)%contour.size()).y, contour.at((i+1)%contour.size()).x), position))
//                 {
//                     double z = (d - normal(0) * position.x() - normal(1)*position.y())/normal(2);
//                     geometry_msgs::Point p;
//                     p.x = position.x();
//                     p.y = position.y();
//                     p.z = z;
//                     polygon.points.emplace_back(p);
//                 }          
//             }
//         }   
//         // for (auto & Po_cv : approxContours.at(0))
//         // {
//         //     grid_map::Position position;
//         //     if (map.getPosition(grid_map::Index(Po_cv.y, Po_cv.x), position))
//         //     {
//         //         double z = (d - normal(0) * position.x() - normal(1)*position.y())/normal(2);
//         //         contour.emplace_back(Eigen::Vector3d(position.x(), position.y(), z));
//         //         geometry_msgs::Point p;
//         //         p.x = position.x();
//         //         p.y = position.y();
//         //         p.z = z;
//         //         polygon.points.emplace_back(p);
//         //     }
//         //     polygon.points.emplace_back(*polygon.points.begin());
//         // }
//         msg.markers.emplace_back(polygon);
//     }
// }


pip_line::~pip_line()
{
}