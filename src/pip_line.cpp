/*
 * @Author: lichao951787328 951787328@qq.com
 * @Date: 2024-06-07 11:01:20
 * @LastEditors: lichao951787328 951787328@qq.com
 * @LastEditTime: 2024-06-18 23:41:52
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
#include <PEAC_AHFP_pl/plane_fitter_pcl_AHFP.hpp>
#include <opencv2/opencv.hpp>
#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlanner.h>
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
pip_line::pip_line(ros::NodeHandle & n):nh(n)
{
    pointcloud_sub = nh.subscribe("/trigger_points", 1, &pip_line::pointcloud_callback, this);
    string goal_point_topic = "/move_base_simple/goal";
    goal_point_sub = nh.subscribe(goal_point_topic, 1, &pip_line::goal_point_callback, this);
    raw_heightmap_pub = nh.advertise<grid_map_msgs::GridMap>("raw_heightmap", 1);
    height_map_upper_pub = nh.advertise<grid_map_msgs::GridMap>("height_map_upper", 1);
    height_map_lower_pub = nh.advertise<grid_map_msgs::GridMap>("height_map_lower", 1);
    height_map_foot_pub = nh.advertise<grid_map_msgs::GridMap>("height_map_foot", 1);
    planes_all = nh.advertise<grid_map_msgs::GridMap>("planes_all", 1);
    planes_cutted = nh.advertise<grid_map_msgs::GridMap>("planes_cutted", 1);
    planes_polygon_pub = nh.advertise<visualization_msgs::MarkerArray>("planes_polygon", 1);
    planes_polygon_cutted_pub = nh.advertise<visualization_msgs::MarkerArray>("planes_polygon_cutted", 1);
    footsteps_pub = nh.advertise<diy_msgs::footSteps>("footsteps", 1);
    avoid_points_pub = nh.advertise<diy_msgs::avoidPointsMsg>("avoid_points", 1);

    timer = nh.createTimer(ros::Duration(1), &pip_line::timerCallback, this);

    grid_map::Length length(4, 2);
    grid_map::Position position(1, 0);
    map.setGeometry(length, 0.02, position);
    map.add("elevation", NAN);

    height_map_upper = map;
    height_map_lower = map;
    height_map_foot = map;
    height_map_upper.clearAll();
    height_map_lower.clearAll();
    height_map_foot.clearAll();
    height_map_upper.add("elevation", NAN);
    height_map_lower.add("elevation", NAN);
    height_map_foot.add("elevation", NAN);
    plane_map = map;
    plane_map.clearAll();
    plane_map.add("elevation", NAN);
    is_finish = false;
    Eigen::Matrix4d T_velodyne_camera = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_base_velodyne = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_world_base = Eigen::Matrix4d::Identity();
    T_velodyne_camera.block<3,3>(0,0) = Eigen::Quaterniond(0.229184, 0, 0.973383, 0).toRotationMatrix();
    T_velodyne_camera.block<3,1>(0,3) = Eigen::Vector3d(0.15, 0, -0.4024);
    T_base_velodyne.block<3,1>(0,3) = Eigen::Vector3d(-0.01, 0, 0.7217);
    T_world_base.block<3,1>(0,3) = Eigen::Vector3d(0, 0, 0.665);
    T_world_camera = T_world_base * T_base_velodyne * T_velodyne_camera;
}


double gaussWight(const int x,const int y, const double sigma)
{
    return (1.0 / (2.0 * CV_PI * sigma * sigma)) * exp(-(x * x + y * y) / (2 * sigma * sigma));
}

void convertToImg(const pcl::PointCloud<pcl::PointXYZ>::Ptr &ptCloud, cv::Mat & image)
{
    // omp_set_num_threads(8);
    // #pragma omp parallel for 
    int imgHeight = image.rows;
    int imgWidth = image.cols;
    for(int row = 0; row < imgHeight; ++row)
    {
        for(int col = 0; col < imgWidth; ++col)
        {
            int index = row * imgWidth + col;
            float depth = ptCloud->points[index].z;
            if(std::isnan(depth))
            {
                image.ptr<uint16_t>(row)[col] = (uint16_t)0;
            }
            else
            {
                image.ptr<uint16_t>(row)[col] = (uint16_t)(depth * 1000);
            }
        }
    }
}

void GaussPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &ptCloudRaw, const pcl::PointCloud<pcl::PointXYZ>::Ptr &ptCloudRefine)
{
    vector<double> gaussWightValues;
    const double sigmaGauss = 1.0;
    const int kSizeGauss = 5;
    const int radiusGauss = kSizeGauss/2;
    //事先生成高斯核的值
    for(int i = -radiusGauss; i <= radiusGauss; ++i)
    {
        for(int j = -radiusGauss; j <= radiusGauss; ++j)
        {
            gaussWightValues.push_back(gaussWight(i, j, sigmaGauss));
        }
    }
    // cout<<"get gauss weight"<<endl;
    cv::Mat imgDepth(ptCloudRaw->height, ptCloudRaw->width, CV_16UC1);
    convertToImg(ptCloudRaw, imgDepth);
    // cout<<"convert to image"<<endl;
    cv::Mat imgDepthTmp = imgDepth.clone();
    cv::Scalar meanImg, stddevImg;
    cv::meanStdDev(imgDepth, meanImg, stddevImg);
    const uint16_t thredHigh = (uint16_t)(meanImg[0] + 3 * stddevImg[0]);
    int kk = meanImg[0] - 3 * stddevImg[0] > 0 ? meanImg[0] - 3 * stddevImg[0] : 1;
    const uint16_t thredLow  = (uint16_t)kk;
    // cout<<"get paramter"<<endl;
    int imgHeight = ptCloudRaw->height;
    int imgWidth = ptCloudRaw->width;
    // ptCloudRefine->resize(imgHeight * imgWidth);
    for(int row = radiusGauss; row < imgHeight - radiusGauss; ++row)
    {
        for(int col = radiusGauss; col < imgWidth - radiusGauss; ++col)
        {
            uint16_t depthOld = imgDepthTmp.ptr<uint16_t>(row)[col];
            int index = row * imgWidth + col;

            double sum = 0.0;
            double weightSum = 0.0;
            int indexTmp = 0;
            for(int i = -radiusGauss; i <= radiusGauss; ++i)
            {
                for(int j = -radiusGauss; j <= radiusGauss; ++j)
                {
                    uint16_t depthNerbor = imgDepthTmp.ptr<uint16_t>(row + i)[col + j];
                    //这里也可以填补空洞
                    if(depthNerbor <=  thredHigh && depthNerbor >= thredLow)
                    {

                        double weight = gaussWightValues[indexTmp];
                        indexTmp ++;
                        sum += depthNerbor * weight;
                        weightSum += weight;
                    }
                }
            }
            uint16_t depthNew = (uint16_t)(sum / weightSum);
            imgDepth.ptr<uint16_t>(row)[col] = depthNew;
            double scale = (double)depthNew / (double)depthOld;       
            ptCloudRefine->points[index].x *= scale;
            ptCloudRefine->points[index].y *= scale;
            ptCloudRefine->points[index].z *= scale;
            
        }
    }
}

// int cloud_index = 0;
void preprocessing(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = *cloud_in;
    
    GaussPointCloud(cloud_in, cloud);
    // pcl::io::savePCDFile("/home/bhr/catkin_beijing4th/src/elevation_mapping/triggerHelios/debug_data/" + std::to_string(cloud_index) + ".pcd", *cloud);
    // cloud_index++;
    std::cout<<cloud->width<<" "<<cloud->height<<std::endl;

    // 去掉顶上一部分
    pcl::io::savePCDFile("/home/bhr/TCDS/src/pip_line/data/raw_points.pcd", *cloud);
    pcl::PointCloud<pcl::PointXYZ> stable_points;
    // 注意相机方向
    for (int i = 120; i < cloud->width - 50; i++)
    {
        for (int j = 30; j < cloud->height - 30; j++)
        {
            if (!std::isnan(cloud->at(j * cloud->width + i).z))
            {
                stable_points.emplace_back(cloud->at(j * cloud->width + i));
            }
        }
    }
    pcl::io::savePCDFile("/home/bhr/TCDS/src/pip_line/data/stable_points.pcd", stable_points);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;

    // 设置体素滤波器的输入点云
    // 这个地方可以选择使用平面点云还是所有点云
    voxel_grid_filter.setInputCloud(stable_points.makeShared());

    // 设置体素大小（体素的边长）这个值不能设成0.01，否则太耗时
    voxel_grid_filter.setLeafSize(0.01f, 0.01f, 0.01f);  // 设置为0.01米

    // 执行体素滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid_filter.filter(*filtered_cloud);
    // cout<<"after filter: "<<filtered_cloud->size()<<endl;
    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud(filtered_cloud);

    // // 设置过滤轴和范围（这里以Z轴为例，过滤掉Z轴范围在[0.0, 1.0]之外的点）
    pass.setFilterFieldName("x");// 相机转动之前是y，转动之后是x
    // // 这个值需要根据数据调节，后面上楼梯肯定会改
    pass.setFilterLimits(-4, 0.5);

    // // 执行直通滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr pass_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*pass_filtered_cloud);

    // cout<<"after pass filter: "<<pass_filtered_cloud->size()<<endl;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(pass_filtered_cloud);// 注意衔接的输入输出

    // 创建KdTree用于法向量估计
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);

    // 设置搜索半径，用于确定每个点的邻域
    ne.setRadiusSearch(0.08);  // 设置为0.03米

    // 计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);
    pcl::PointCloud<pcl::PointXYZ> result;
    for (size_t i = 0; i < pass_filtered_cloud->size(); i++)
    {
        Eigen::Vector3d p(pass_filtered_cloud->at(i).x, pass_filtered_cloud->at(i).y, pass_filtered_cloud->at(i).z);
        Eigen::Vector3d n(normals->at(i).normal_x, normals->at(i).normal_y, normals->at(i).normal_z);
        if (std::abs(p.normalized().dot(n)) > 0.3)// 虽然这是一个阈值，但应该能满足要求
        {
            result.emplace_back(pass_filtered_cloud->at(i));
        }
    }
    // cout<<"after Normal filter: "<<result.size()<<endl;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(result.makeShared());
    
    // 设置统计滤波器参数
    sor.setMeanK(50);  // 设置邻域中点的数量
    sor.setStddevMulThresh(1.0);  // 设置标准差的倍数阈值

    // 应用统计滤波器
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(cloud_out);
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc);
    LOG(INFO)<<pc->size();

    pcl::PointCloud<pcl::PointXYZ> pub_cloud;
    preprocessing(pc, pub_cloud);

    LOG(INFO)<<pub_cloud.size();
    for (auto & point : pub_cloud)
    {
        Eigen::Vector3d po_w = T_world_camera.block<3,3>(0,0)* Eigen::Vector3d(point.x, point.y, point.z) + T_world_camera.block<3,1>(0,3);
        grid_map::Index index;
        if (map.getIndex(po_w.head(2), index))
        {
            map["elevation"](index.x(), index.y()) = po_w.z();
        }
    }

    // cv::Mat raw_image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
    // for (int i = 0; i < map.getSize().x(); i++)
    // {
    //     for (int j = 0; j < map.getSize().y(); j++)
    //     {
    //         grid_map::Position3 p3;
    //         if (map.getPosition3("elevation", grid_map::Index(i, j), p3))
    //         {
    //             if (!std::isnan(p3.z()))
    //             {
    //                 raw_image.at<uchar>(i, j) = 255;
    //             }
    //         }
    //     }
    // }
    // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/raw_image.png", raw_image);

    
    // 使用地图转成有序点云
    pcl::PointCloud<pcl::PointXYZ> org_pc = gridMap2Pointcloud(map);

    // 替换

    orginazed_points raw_points;
    raw_points.initialByPCL(*pc);
    size_t width = pc->width;
    size_t height = pc->height;

    string package_path;
    initial_package_path("plane_detection", package_path);
    parameter param;
    load_parameter(param, package_path + "/config/parameter.xml");
    std::cout<<"load parameter finish"<<std::endl;
    param.initial(raw_points.width);
    // // param.showParameter();
    quatree::node::setStaticMember(width, height, param.quatree_width, ROBOTWORLD_T_CAMERA, raw_points, param);

    quatree::quatree qq(raw_points, param);
    
    plane_segmentation ps(height, width, &qq, raw_points, param);
    vector<plane_info> planes = ps.getPlaneResult();
    cv::Mat result = ps.getSegResult();
    vector<cv::Mat> single_results = ps.getSegResultSingle();

    vector<cv::Mat> collision_free_images;
    // 地图膨胀层
    double resolution = map.getResolution();
    double inflation_radius = 0.5;
    int inflation_pixel = 0.5/resolution;
    for (int i = 0; i < single_results.size(); i++)
    {
        cv::Mat image = single_results.at(i);
        int kernel_size = inflation_pixel;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));

        // 对图像进行膨胀操作
        cv::Mat dilated_image;
        cv::dilate(image, dilated_image, kernel);

        // 计算膨胀后的边缘
        cv::Mat collision_layer = dilated_image - image;
        cv::Mat upper_body = cv::Mat::zeros(collision_layer.size(), CV_8UC1);
        cv::Mat knee = cv::Mat::zeros(collision_layer.size(), CV_8UC1);
        for (int y = 0; y < collision_layer.rows; ++y) 
        {
            for (int x = 0; x < collision_layer.cols; ++x) 
            {
                if (collision_layer.at<uchar>(y, x) == 255) 
                {
                    // 获取膝盖层和上半身层
                    grid_map::Position3 p3;
                    if (map.getPosition3("elevation", grid_map::Index(y, x), p3))
                    {
                        if (!std::isnan(p3.z()))
                        {
                            double dis = (p3 - planes.at(i).center).dot(planes.at(i).normal);
                            if (dis > 0.5)// 上半身
                            {
                                upper_body.at<uchar>(y,x) = 255;
                            }
                            else if (dis > 0.3) // 膝盖
                            {
                                knee.at<uchar>(y,x) = 255;
                            }
                        }
                    }
                }
            }
        }
        // 进行不同半径的膨胀
        double upper_inflation = 0.5;
        double knee_inflation = 0.1;
        int inflation_radius_upper = upper_inflation/resolution;
        int inflation_radius_knee = knee_inflation/resolution;

        cv::Mat kernel_upper = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(inflation_radius_upper, inflation_radius_upper));
        cv::Mat kernel_knee = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(inflation_radius_knee, inflation_radius_knee));

        // 对图像进行膨胀操作
        cv::Mat dilated_image_upper;
        cv::dilate(upper_body, dilated_image_upper, kernel_upper);

        cv::Mat dilated_image_knee;
        cv::dilate(knee, dilated_image_knee, kernel_knee);
    
        // 计算膨胀后的边缘
        cv::Mat collision_layer1 = image - dilated_image_upper;
        cv::Mat collision_layer2 = collision_layer1 - dilated_image_knee;
        collision_free_images.emplace_back(collision_layer2);
    }
    // 落脚点规划

    // // 平面检测
    // AHFP_pl::PlanarContourExtraction pce(org_pc);
    // pce.run();
    // cv::Mat plane_image = pce.getSegImage();
    // // cv::imshow("seg 1", plane_image);
    // // cv::waitKey(0);
    // // LOG(INFO)<<"...";
    // vector<cv::Mat> planes = pce.getSegPlanes();
    // vector<Eigen::Vector3d> obstacle, plane_points;
    // pcl::PointCloud<pcl::PointXYZ> pcl_obstacle, pcl_plane_points;
    // for (int i = 0; i < plane_image.rows; i++)
    // {
    //     for (int j = 0; j < plane_image.cols; j++)
    //     {
    //         if (plane_image.at<cv::Vec3b>(i, j) ==  cv::Vec3b(0, 0, 0))
    //         {
    //             grid_map::Position3 p3;
    //             if (map.getPosition3("elevation", grid_map::Index(i, j), p3))
    //             {
    //                 if (!std::isnan(p3.z()))
    //                 {
    //                     obstacle.emplace_back(p3);
    //                     pcl_obstacle.emplace_back(pcl::PointXYZ(p3.x(), p3.y(), p3.z()));
    //                 }
    //             }
    //         }
    //         else
    //         {
    //             grid_map::Position3 p3;
    //             if (map.getPosition3("elevation", grid_map::Index(i, j), p3))
    //             {
    //                 if (!std::isnan(p3.z()))
    //                 {
    //                     plane_points.emplace_back(p3);
    //                     pcl_plane_points.emplace_back(pcl::PointXYZ(p3.x(), p3.y(), p3.z()));
    //                 }
    //             }
    //         }
    //     }
    // }
    // pcl::io::savePCDFileASCII("/home/bhr/TCDS/src/pip_line/data/obstacle.pcd", pcl_obstacle);
    // pcl::io::savePCDFileASCII("/home/bhr/TCDS/src/pip_line/data/plane.pcd", pcl_plane_points);
    // for (auto & image : planes)
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
    //     if (center.z() > 0.5)
    //     {
    //         LOG(INFO)<<"INSERT";
    //         obstacle.insert(obstacle.end(), points.begin(), points.end());
    //         continue;
    //     }
    //     Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    //     for (auto & point : points)
    //     {
    //         M += (point - center) * (point - center).transpose();
    //     }
    //     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(M);
    //     auto eigenvectors = eigensolver.eigenvectors();
    //     Eigen::Vector3d minEigenvector = eigenvectors.col(0);
    //     if (abs(minEigenvector.z()) < 0.9)
    //     {
    //         LOG(INFO)<<"INSERT";
    //         obstacle.insert(obstacle.end(), points.begin(), points.end());
    //     }
    // }
    // LOG(INFO)<<height_map_upper.getSize().transpose();
    // cv::Mat height_map_upper_image = cv::Mat::zeros(height_map_upper.getSize().x(), height_map_upper.getSize().y(), CV_8UC1);
    // // LOG(INFO)<<height_map_upper_image.rows<<" "<<height_map_upper_image.cols;
    // cv::Mat height_map_lower_image = cv::Mat::zeros(height_map_lower.getSize().x(), height_map_lower.getSize().y(), CV_8UC1);
    // // LOG(INFO)<<height_map_lower_image.rows<<" "<<height_map_lower_image.cols;
    // cv::Mat height_map_foot_image = cv::Mat::zeros(height_map_foot.getSize().x(), height_map_foot.getSize().y(), CV_8UC1);
    
    // // 构建不可落脚区域的高程图
    // for (auto & point : obstacle)
    // {
    //     if (point.z() > 0.6)
    //     {
    //         grid_map::Index index;
    //         if (height_map_upper.getIndex(point.head(2), index))
    //         {
    //             height_map_upper["elevation"](index.x(), index.y()) = point.z();
    //             height_map_upper_image.at<uchar>(index.x(), index.y()) = 255;
    //         }
    //     }
    //     else if (point.z() < 0.15)
    //     {
    //         grid_map::Index index;
    //         if (height_map_lower.getIndex(point.head(2), index))
    //         {
    //             height_map_foot["elevation"](index.x(), index.y()) = point.z();
    //             height_map_foot_image.at<uchar>(index.x(), index.y()) = 255;
    //         }
    //     }
    //     else
    //     {
    //         grid_map::Index index;
    //         if (height_map_lower.getIndex(point.head(2), index))
    //         {
    //             height_map_lower["elevation"](index.x(), index.y()) = point.z();
    //             height_map_lower_image.at<uchar>(index.x(), index.y()) = 255;
    //         }
    //     }
    // }
    // // LOG(INFO)<<"...";
    // // cv::imshow("seg 2", height_map_upper_image);
    // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/height_map_upper_image.png", height_map_upper_image);
    // // cv::waitKey(0);
    // // LOG(INFO)<<"...";
    // // cv::imshow("seg 3", height_map_lower_image);
    // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/height_map_lower_image.png", height_map_lower_image);
    // // cv::waitKey(0);
    // // LOG(INFO)<<"...";
    // // cv::imshow("seg 4", height_map_foot_image);
    // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/height_map_foot_image.png", height_map_foot_image);
    // // cv::waitKey(0);
    // // LOG(INFO)<<"...";
    // for (auto & point : plane_points)
    // {
    //     grid_map::Index index;
    //     if (plane_map.getIndex(point.head(2), index))
    //     {
    //         plane_map["elevation"](index.x(), index.y()) = point.z();
    //     }
    // }
    // // 下半身图像扩展
    // int lower_pixel = 0.1/height_map_lower.getResolution();
    // int upper_pixel = 0.3/height_map_upper.getResolution();
    // // Define the structuring element (3x3 square kernel)
    // cv::Mat kernel_lower = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(lower_pixel, lower_pixel));
    // // Perform erosion
    // cv::Mat eroded_image_lower;
    // // cv::dilate(image, dilatedImage, kernel);
    // // LOG(INFO)<<"...";
    // cv::dilate(height_map_lower_image, eroded_image_lower, kernel_lower);
    // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/eroded_image_lower.png", eroded_image_lower);
    // // cv::imshow("seg 4", eroded_image_lower);
    // // cv::waitKey(0);
    // // LOG(INFO)<<"...";
    // // Define the structuring element (3x3 square kernel)
    // cv::Mat kernel_upper = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(upper_pixel, upper_pixel));
    // // Perform erosion
    // cv::Mat eroded_image_upper;
    // cv::dilate(height_map_upper_image, eroded_image_upper, kernel_upper);
    // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/eroded_image_upper.png", eroded_image_upper);
    // // cv::imshow("seg 5", eroded_image_lower);
    // // cv::waitKey(0);
    // // LOG(INFO)<<"...";
    // plane_cutted = plane_map;
    // // 根据高程图对平面高程图进行切割
    // for (int i = 0; i < height_map_upper_image.rows; i++)
    // {
    //     for (int j = 0; j < height_map_upper_image.cols; j++)
    //     {
    //         if (eroded_image_lower.at<uchar>(i, j) == 255 || eroded_image_upper.at<uchar>(i, j) == 255)
    //         {
    //             plane_cutted["elevation"](i, j) = NAN;
    //         }
    //     }
    // }
    // // 创建中值滤波器
    // // grid_map::filters::MedianFillFilter median_fill_filter("eleation", "eleation", 0.1);
    // // // 应用滤波器
    // // if (!median_fill_filter.configure()) {
    // //     ROS_ERROR("无法配置中值滤波器！");
    // //     return -1;
    // // }
    // // grid_map::GridMap output_map, output_map2;
    // // if (!median_fill_filter.update(plane_map, output_map)) {
    // //     ROS_ERROR("无法应用中值滤波器！");
    // //     return -1;
    // // }
    // // if (!median_fill_filter.update(plane_cutted, output_map2)) {
    // //     ROS_ERROR("无法应用中值滤波器！");
    // //     return -1;
    // // }
    // draw_planes(plane_map, planes_msg, 1, 0, 0);
    // draw_planes(plane_cutted, planes_cutted_msg, 0, 1, 0);
    // is_finish = true;
    // // return;
    // // 再在切割后的高程图下进行规划
    // if (get_goal)
    // {
    //     // 获得终点
    //     LOG(INFO)<<"start planning -------------------------";
    //     grid_map::Index left_top_index;
    //     if (plane_cutted.getIndex(grid_map::Position(0.5, 1), left_top_index))
    //     {
    //         const int lengthInXSubmapI = static_cast<int>(0.8/plane_cutted.getResolution());
    //         const int lengthInYSubmapI = static_cast<int>(2/plane_cutted.getResolution());
    //         for (int i = 0; i < lengthInXSubmapI; i++)
    //         {
    //             for (int j = 0; j < lengthInYSubmapI; j++)
    //             {
    //                 if (std::isnan(plane_cutted["elevation"](left_top_index.x() + i, left_top_index.y() + j)))
    //                 {
    //                    plane_cutted["elevation"](left_top_index.x() + i, left_top_index.y() + j) = 0;
    //                 }
    //             }
    //         }
    //     }
    //     FootParam footparam(0.13, 0.11, 0.065, 0.065);
    //     AstarHierarchicalFootstepPlanner planner(plane_cutted, footparam, 0.2);
    //     Eigen::Vector3d left_foot(0, 0.1, 0);
    //     Eigen::Vector3d right_foot(0, -0.1, 0);
    //     Eigen::Vector3d goal_p;
    //     goal_p.x() = goal.position.x;
    //     goal_p.y() = goal.position.y;
    //     Eigen::Quaterniond qd(goal.orientation.w, goal.orientation.x, goal.orientation.y, goal.orientation.z);
    //     Eigen::Vector3d v_t = qd.toRotationMatrix() * Eigen::Vector3d::UnitX();
    //     double yaw = atan(v_t.y()/v_t.x());
    //     goal_p.z() = yaw;
    //     vector<Footstep> steps;
    //     vector<vector<Eigen::Vector3d>> avoid_points;
    //     if (planner.initial(left_foot, right_foot, 0, goal_p))// 先迈右脚
    //     {
    //         LOG(INFO)<<"set start and goal";
    //         if (planner.plan())
    //         {
    //             steps = planner.getResultSteps();
    //             avoid_points = planner.computeAvoidPoints();
    //             for (auto & step : steps)
    //             {
    //                 cout<<setw(8)<<step.x<<" "<<step.y<<" "<<step.z<<" "<<step.roll<<" "<<step.pitch<<" "<<step.yaw*57.3<<" "<<step.robot_side<<endl;
    //             }
    //         }
    //     }
    //     else
    //     {
    //         LOG(INFO)<<"planning error";
    //     }
    //     diy_msgs::footSteps steps_pub;
    //     steps_pub.header.frame_id = "map";
    //     for (auto & s : steps)
    //     {
    //         diy_msgs::footStep tmp_step;
    //         tmp_step.is_left = s.robot_side == 0 ? true : false;
    //         tmp_step.x = s.x;
    //         tmp_step.y = s.y;
    //         tmp_step.z = s.z;
    //         tmp_step.roll = s.roll;
    //         tmp_step.pitch = s.pitch;
    //         tmp_step.yaw = s.yaw;
    //         steps_pub.footsteps.emplace_back(tmp_step);
    //     }
    //     diy_msgs::avoidPointsMsg points_pub;
    //     points_pub.header.frame_id = "map";
    //     for (auto & points : avoid_points)
    //     {
    //         diy_msgs::avoidPoints ps;
    //         // ps.terrainType
    //         for (auto & point : points)
    //         {
    //             geometry_msgs::Point p;
    //             p.x = point.x();
    //             p.y = point.y();
    //             p.z = point.z();
    //             ps.avoidPoints.emplace_back(p);
    //         }
    //         points_pub.avoidPointsMsg.emplace_back(ps);
    //     }
    //     footsteps_pub.publish(steps_pub);
    //     avoid_points_pub.publish(points_pub);
    // }

}

void pip_line::timerCallback(const ros::TimerEvent & event)
{
    if (is_finish)
    {
        grid_map_msgs::GridMap map_msg;
        grid_map::GridMapRosConverter::toMessage(map, map_msg);
        map_msg.info.header.frame_id = "map";
        raw_heightmap_pub.publish(map_msg);

        grid_map_msgs::GridMap upper_msg;
        grid_map::GridMapRosConverter::toMessage(height_map_upper, upper_msg);
        upper_msg.info.header.frame_id = "map";
        height_map_upper_pub.publish(upper_msg);

        grid_map_msgs::GridMap lower_msg;
        grid_map::GridMapRosConverter::toMessage(height_map_lower, lower_msg);
        lower_msg.info.header.frame_id = "map";
        height_map_lower_pub.publish(lower_msg);

        grid_map_msgs::GridMap foot_msg;
        grid_map::GridMapRosConverter::toMessage(height_map_foot, foot_msg);
        foot_msg.info.header.frame_id = "map";
        height_map_foot_pub.publish(foot_msg);

        grid_map_msgs::GridMap height_msg;
        grid_map::GridMapRosConverter::toMessage(plane_map, height_msg);
        height_msg.info.header.frame_id = "map";
        planes_all.publish(height_msg);

        grid_map_msgs::GridMap cutted_msg;
        grid_map::GridMapRosConverter::toMessage(plane_cutted, cutted_msg);
        cutted_msg.info.header.frame_id = "map";
        planes_cutted.publish(cutted_msg);

        planes_polygon_pub.publish(planes_msg);
        planes_polygon_cutted_pub.publish(planes_cutted_msg);
    }
    
}

void pip_line::goal_point_callback(geometry_msgs::PoseStamped::ConstPtr pose_p)
{
    ROS_INFO("get goal********************************************************8");
    std::lock_guard<std::mutex> lock(m_goal);
    goal.orientation = pose_p->pose.orientation;
    goal.position    = pose_p->pose.position;
    get_goal = true;
}

void getPlaneInfo(grid_map::GridMap & map, cv::Mat & image, Eigen::Vector3d & normal, double & d)
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    vector<Eigen::Vector3d> points;
    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            if (image.at<uchar>(i, j) == 255)
            {
                grid_map::Position3 p3;
                if (map.getPosition3("elevation", grid_map::Index(i, j), p3))
                {
                    points.emplace_back(p3);
                    center += p3;
                }
            }
        }
    }
    center = center/points.size();
    
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    for (auto & point : points)
    {
        M += (point - center) * (point - center).transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(M);
    auto eigenvectors = eigensolver.eigenvectors();
    normal = eigenvectors.col(0);
    d = -(normal.dot(center));
}

// 使用平面地形高程图检测平面，获取轮廓，在根据像素轮廓，获取平面点，再根据三维点画出多边形
void pip_line::draw_planes(grid_map::GridMap map, visualization_msgs::MarkerArray & msg, double r, double g, double b)
{
    pcl::PointCloud<pcl::PointXYZ> org_pc = gridMap2Pointcloud(map);
    AHFP_pl::PlanarContourExtraction pce(org_pc);
    pce.run();
    vector<cv::Mat> planes = pce.getSegPlanes();
    visualization_msgs::MarkerArray polygons;
    int index = 0;
    for (auto & image : planes)
    {
        visualization_msgs::Marker polygon;
        polygon.header.frame_id = "map";
        polygon.header.stamp = ros::Time::now();
        polygon.ns = "polygons";
        polygon.id = index;
        index++;
        polygon.type = visualization_msgs::Marker::LINE_LIST;
        polygon.action = visualization_msgs::Marker::ADD;
        polygon.pose.orientation.w = 1.0;
        polygon.scale.x = 0.03;  // 线条的宽度
        polygon.color.r = r;
        polygon.color.g = g;
        polygon.color.b = b;
        polygon.color.a = 1.0;

        Eigen::Vector3d normal; 
        double d;
        getPlaneInfo(map, image, normal, d);

        // cv::Mat blurred;
        // cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        auto itr = contours.begin();    //使用迭代器去除噪声轮廓
        while (itr != contours.end())
        {
            auto area = cv::contourArea(*itr);  //获得轮廓面积
            if (area < 8)    //删除较小面积的轮廓
            {
                itr = contours.erase(itr); //itr一旦erase，需要重新赋值
            }
            else
            {
                itr++;
            }
        }

        std::vector<std::vector<cv::Point>> approxContours(contours.size());
        for (size_t i = 0; i < contours.size(); i++) {
            double epsilon = 0.01 * cv::arcLength(contours[i], true);
            cv::approxPolyDP(contours[i], approxContours[i], epsilon, true);
        }
        // 根据像素坐标系求轮廓点的xy值，在根据平面信息求z值
        
        // vector<Eigen::Vector3d> contour;
        for (auto & contour : approxContours)
        {
            for (int i = 0; i < contour.size(); i++)
            {
                grid_map::Position position;
                if (map.getPosition(grid_map::Index(contour.at(i%contour.size()).y, contour.at(i%contour.size()).x), position))
                {
                    double z = (d - normal(0) * position.x() - normal(1)*position.y())/normal(2);
                    geometry_msgs::Point p;
                    p.x = position.x();
                    p.y = position.y();
                    p.z = z;
                    polygon.points.emplace_back(p);
                }
                if (map.getPosition(grid_map::Index(contour.at((i+1)%contour.size()).y, contour.at((i+1)%contour.size()).x), position))
                {
                    double z = (d - normal(0) * position.x() - normal(1)*position.y())/normal(2);
                    geometry_msgs::Point p;
                    p.x = position.x();
                    p.y = position.y();
                    p.z = z;
                    polygon.points.emplace_back(p);
                }
                
            }
        }
        
        
        // for (auto & Po_cv : approxContours.at(0))
        // {
        //     grid_map::Position position;
        //     if (map.getPosition(grid_map::Index(Po_cv.y, Po_cv.x), position))
        //     {
        //         double z = (d - normal(0) * position.x() - normal(1)*position.y())/normal(2);
        //         contour.emplace_back(Eigen::Vector3d(position.x(), position.y(), z));
        //         geometry_msgs::Point p;
        //         p.x = position.x();
        //         p.y = position.y();
        //         p.z = z;
        //         polygon.points.emplace_back(p);
        //     }
        //     polygon.points.emplace_back(*polygon.points.begin());
        // }
        msg.markers.emplace_back(polygon);
    }
}


pip_line::~pip_line()
{
}