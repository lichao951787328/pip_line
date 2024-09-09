/*
 * @Author: lichao951787328 951787328@qq.com
 * @Date: 2024-06-07 11:01:20
 * @LastEditors: lichao951787328 951787328@qq.com
 * @LastEditTime: 2024-09-07 16:33:07
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
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <pcl/features/principal_curvatures.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <plane_detection/plane_new.h>

// #ifdef OURMETHOD
// #include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlanner.h>
// #else
// #include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerTra.h>
// #endif



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
    // pointcloud_sub = nh.subscribe("/camera/depth/color/points", 1, &pip_line::pointcloud_callback, this);
    // pointcloud_sub = nh.subscribe("/trigger_points", 1, &pip_line::pointcloud_callback, this);
    string goal_point_topic = "/move_base_simple/goal";
    goal_point_sub = nh.subscribe(goal_point_topic, 1, &pip_line::goal_point_callback, this);
    raw_heightmap_pub = nh.advertise<grid_map_msgs::GridMap>("raw_heightmap", 1);
    // sub_map_pub = nh.advertise<grid_map_msgs::GridMap>("sub_map", 1);
    raw_heightmap_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("raw_heightmap_pc", 1);
    height_map_upper_pub = nh.advertise<grid_map_msgs::GridMap>("height_map_upper", 1);
    height_map_lower_pub = nh.advertise<grid_map_msgs::GridMap>("height_map_lower", 1);
    feasible_map_pub = nh.advertise<grid_map_msgs::GridMap>("feasible_map", 1);
    // height_map_foot_pub = nh.advertise<grid_map_msgs::GridMap>("height_map_foot", 1);
    seg_result_image_pub = nh.advertise<sensor_msgs::Image>("seg_result", 1);
    // planes_all = nh.advertise<grid_map_msgs::GridMap>("planes_all", 1);
    // planes_cutted = nh.advertise<grid_map_msgs::GridMap>("planes_cutted", 1);
    // planes_polygon_pub = nh.advertise<visualization_msgs::MarkerArray>("planes_polygon", 1);
    // planes_polygon_cutted_pub = nh.advertise<visualization_msgs::MarkerArray>("planes_polygon_cutted", 1);


    footsteps_pub = nh.advertise<diy_msgs::footSteps>("footsteps", 1);
    footsteps_visual_pub = nh.advertise<visualization_msgs::MarkerArray>("footsteps_visual", 1);
    footsteps_arrow_pub = nh.advertise<visualization_msgs::MarkerArray>("footsteps_arrow", 1);

    avoid_points_pub = nh.advertise<diy_msgs::avoidPointsMsg>("avoid_points", 1);
    avoid_points_visual_pub = nh.advertise<visualization_msgs::MarkerArray>("avoid_points_visual", 1);

    timer = nh.createTimer(ros::Duration(1), &pip_line::timerCallback, this);

    grid_map::Length length(3, 2);
    grid_map::Position position(0.5, 0);
    map.setGeometry(length, 0.01, position);
    map.add("elevation", NAN);

    height_map_upper = map;
    height_map_lower = map;
    // height_map_foot = map;
    height_map_upper.clearAll();
    height_map_lower.clearAll();
    // height_map_foot.clearAll();
    height_map_upper.add("elevation", NAN);
    height_map_lower.add("elevation", NAN);
    // height_map_foot.add("elevation", NAN);
    // plane_map = map;
    // plane_map.clearAll();
    // plane_map.add("elevation", NAN);
    is_finish = false;

    initial_package_path("pip_line", package_path);
    LOG(INFO)<<"package path is: "<<package_path;
    pd.initial(package_path + "/config/plane_fitter_pcd.ini");
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

    // sim_terrain
    // T_world_camera.setIdentity();
    // T_world_camera(0, 0) = -1;
    // T_world_camera(2, 2) = -1;
    // T_world_camera(0, 3) = 2.5;
    // T_world_camera(2, 3) = 6;

    // Eigen::Matrix4d T_install_depth = Eigen::Matrix4d::Identity();
    // T_install_depth(1, 3) = -0.001;
    // T_install_depth(2, 3) = 0.026 - 0.0045;
    // Eigen::Matrix4d T_hole_install = Eigen::Matrix4d::Identity();
    // Eigen::Matrix3d R;
    // R<<- 0.7071, 0, 0.7071,
    //     0, 1, 0,
    //     - 0.7071, 0, -0.7071;
    // T_hole_install.block<3,3>(0,0) = R;
    // T_hole_install(0, 3) = 0.07025;
    // T_hole_install(2, 3) = 0.00424;
    // Eigen::Matrix4d T_base_hole = Eigen::Matrix4d::Identity();
    // T_base_hole(0, 3) = 0.05675;
    // T_base_hole(2, 3) = 0.49123;
    // Eigen::Matrix4d T_world_base = Eigen::Matrix4d::Identity();
    // // 一定要注意根据实际高度调节，控制端反馈
    // T_world_base.block<3,1>(0,3) = Eigen::Vector3d(0, 0, 0.71);
    // T_world_camera = T_world_base * T_base_hole * T_hole_install * T_install_depth;


    // 通过彩色图标定的位置
    //   frame_id: "depth_to_color_extrinsics"
// rotation: [0.9999532699584961, -0.009290458634495735, 0.002664804458618164, 0.009367496706545353, 0.9994901418685913, -0.03052295371890068, -0.0023798735346645117, 0.030546491965651512, 0.9995304942131042]
// translation: [-0.0004955815966241062, 0.013567420653998852, -0.007013721391558647]

    // Eigen::Matrix4d T_color_depth = Eigen::Matrix4d::Identity();
    // Eigen::Matrix3d R_color_depth;
    // R_color_depth<< 0.9999532699584961, -0.009290458634495735, 0.002664804458618164,
    //                 0.009367496706545353, 0.9994901418685913, -0.03052295371890068,
    //                 -0.0023798735346645117, 0.030546491965651512, 0.9995304942131042;
    // T_color_depth(0, 3) = -0.0004955815966241062;
    // T_color_depth(1, 3) = 0.013567420653998852;
    // T_color_depth(2, 3) = -0.007013721391558647;

    // Eigen::Matrix4d T_base1_color = Eigen::Matrix4d::Identity();
    // T_base1_color<<-0.728249, 0.0600791, 0.682674, 0.173513,
    //                 0.0538417, 0.998087, -0.0304012, -0.0128659,
    //                 -0.68319, 0.0146158, -0.730088, 0.389446,
    //                 0,0,0,1;
    // Eigen::Matrix4d T_base_base1 = Eigen::Matrix4d::Identity();
    // T_base_base1(2, 3) = 0.15;
    // T_world_camera = T_world_base * T_base_base1 * T_base1_color * T_color_depth;
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
    clock_t start1 = clock();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = *cloud_in;
    
    // 去除这个过程
    // GaussPointCloud(cloud_in, cloud);
    // pcl::io::savePCDFile("/home/bhr/catkin_beijing4th/src/elevation_mapping/triggerHelios/debug_data/" + std::to_string(cloud_index) + ".pcd", *cloud);
    // cloud_index++;
    std::cout<<cloud->width<<" "<<cloud->height<<std::endl;
    clock_t end1 = clock();
    // cout<<"preprocess1 cost: "<<(double)(end1 - start1)/CLOCKS_PER_SEC<<std::endl;

    // 去掉顶上一部分
    // pcl::io::savePCDFile("/home/bhr/TCDS/src/pip_line/data/raw_points.pcd", *cloud);
    pcl::PointCloud<pcl::PointXYZ> stable_points;
    // 注意相机方向
    clock_t start2 = clock();
    for (int i = 100; i < cloud->width - 230; i++)
    {
        for (int j = 30; j < cloud->height - 30; j++)
        {
            if (!std::isnan(cloud->at(j * cloud->width + i).z))
            {
                stable_points.emplace_back(cloud->at(j * cloud->width + i));
            }
        }
    }
    clock_t end2 = clock();
    // cout<<"preprocess2 cost: "<<(double)(end2 - start2)/CLOCKS_PER_SEC<<std::endl;
    // pcl::io::savePCDFile("/home/bhr/TCDS/src/pip_line/data/stable_points.pcd", stable_points);

    clock_t start3 = clock();
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;

    // 设置体素滤波器的输入点云
    // 这个地方可以选择使用平面点云还是所有点云
    voxel_grid_filter.setInputCloud(stable_points.makeShared());

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
    pass.setFilterLimits(-4, 0.5);

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
    voxel_grid_filter.setLeafSize(0.06f, 0.06f, 0.06f);  // 设置为0.01米

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
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(pass_filtered_cloud);// 注意衔接的输入输出

    // // 创建KdTree用于法向量估计
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);

    // // 设置搜索半径，用于确定每个点的邻域
    ne.setRadiusSearch(0.08);  // 设置为0.03米

    // // 计算法向量
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
    clock_t start5 = clock();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(result.makeShared());
    // sor.setInputCloud(pass_filtered_cloud);
    
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

void preprocessingColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
    // 将有颜色的点云转成无色点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud_in, *cloud_xyz);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>());
    *cloud_filter = *cloud_xyz;
    GaussPointCloud(cloud_xyz, cloud_filter);
    LOG(INFO)<<cloud_filter->size();
    clock_t start3 = clock();
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;

    // 设置体素滤波器的输入点云
    // 这个地方可以选择使用平面点云还是所有点云
    voxel_grid_filter.setInputCloud(cloud_filter);

    // 设置体素大小（体素的边长）这个值不能设成0.01，否则太耗时
    voxel_grid_filter.setLeafSize(0.008f, 0.008f, 0.008f);  // 设置为0.01米

    // 执行体素滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid_filter.filter(*filtered_cloud);

    clock_t end3 = clock();
    // cout<<"preprocess3 cost: "<<(double)(end3 - start3)/CLOCKS_PER_SEC<<std::endl;
    // cout<<"after filter: "<<filtered_cloud->size()<<endl;

    // clock_t start4 = clock();
    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud(filtered_cloud);

    // // 设置过滤轴和范围（这里以Z轴为例，过滤掉Z轴范围在[0.0, 1.0]之外的点）
    pass.setFilterFieldName("x");// 相机转动之前是y，转动之后是x
    // // 这个值需要根据数据调节，后面上楼梯肯定会改
    pass.setFilterLimits(-8, 8);

    // // 执行直通滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr pass_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*pass_filtered_cloud);
    // clock_t end4 = clock();
    // cout<<"preprocess4 cost: "<<(double)(end4 - start4)/CLOCKS_PER_SEC<<std::endl;
    // 飞点去除
    // cout<<"after pass filter: "<<pass_filtered_cloud->size()<<endl;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(pass_filtered_cloud);// 注意衔接的输入输出

    // // // 创建KdTree用于法向量估计
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);

    // // // 设置搜索半径，用于确定每个点的邻域
    ne.setRadiusSearch(0.04);  // 设置为0.03米

    // // // 计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);
    pcl::PointCloud<pcl::PointXYZ> result;
    for (size_t i = 0; i < pass_filtered_cloud->size(); i++)
    {
        Eigen::Vector3d p(pass_filtered_cloud->at(i).x, pass_filtered_cloud->at(i).y, pass_filtered_cloud->at(i).z);
        Eigen::Vector3d n(normals->at(i).normal_x, normals->at(i).normal_y, normals->at(i).normal_z);
        if (std::abs(p.normalized().dot(n)) > 0.2)// 虽然这是一个阈值，但应该能满足要求
        {
            result.emplace_back(pass_filtered_cloud->at(i));
        }
    }
    cloud_out = result;
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
    pcl::io::savePCDFileASCII(package_path + "/data/pc.pcd", *pc);
    LOG(INFO)<<pc->size();
    
    // auto start_o = clock();
    pcl::PointCloud<pcl::PointXYZ> pub_cloud;
    auto time_clock1 = std::chrono::high_resolution_clock::now();
    preprocessingColor(pc, pub_cloud);
    auto time_clock2 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(time_clock2 - time_clock1).count();
    std::cout << "Time 1 taken by function: " << duration1 << " milliseconds" << std::endl;

    // 点云转成高程图，并对高程图内的空点云处理
    pcl::PointCloud<pcl::PointXYZ> pc_world;
    LOG(INFO)<<pub_cloud.size();
    for (auto & point : pub_cloud)
    {
        Eigen::Vector3d po_w = T_world_camera.block<3,3>(0,0)* Eigen::Vector3d(point.x, point.y, point.z) + T_world_camera.block<3,1>(0,3);
        pc_world.emplace_back(pcl::PointXYZ(po_w.x(), po_w.y(), po_w.z()));
        grid_map::Index index;
        if (map.getIndex(po_w.head(2), index))
        {
            if (std::isnan(map["elevation"](index.x(), index.y())))
            {
                map["elevation"](index.x(), index.y()) = po_w.z();
            }
            else
            {
                if(map["elevation"](index.x(), index.y()) < po_w.z())
                {
                    map["elevation"](index.x(), index.y()) = po_w.z();
                }
            }
        }
    }
    pcl::io::savePCDFileASCII(package_path + "/data/pc_world.pcd", pc_world);

    // 这是补齐实际采集点云的空洞，仿真数据时线不要这部分
    const float minValue = map.get("elevation").minCoeffOfFinites();
    const float maxValue = map.get("elevation").maxCoeffOfFinites();
    cv::Mat originalImage;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "elevation", CV_8UC1, minValue, maxValue, originalImage);
    // cv::imshow("originalImage", originalImage);
    // cv::waitKey(0);
    // // 对空洞进行补齐
    // // 创建一个标记小区域的掩码
    cv::Mat smallRegionsMask = cv::Mat::zeros(originalImage.size(), CV_8UC1);

    // // // 标记连接组件
    cv::Mat labels, stats, centroids;
    int nLabels = cv::connectedComponentsWithStats(originalImage == 0, labels, stats, centroids, 8, CV_32S);

    // // 遍历每个连接组件
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

    // // // // 使用掩码进行图像修复
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

    // 对小区域内补齐 (0.4, -0.4) (0, 0.4)
    grid_map::Index fill_start, fill_end;
    if (map.getIndex(grid_map::Position(0.55, 0.5), fill_start))
    {
        if (map.getIndex(grid_map::Position(-0.45, -0.5), fill_end))
        {
            LOG(INFO)<<"fill_start: "<<fill_start.transpose();
            LOG(INFO)<<"fill_end: "<<fill_end.transpose();
            for (int i = fill_start.x(); i < fill_end.x(); i++)
            {
                for (int j = fill_start.y(); j < fill_end.y(); j++)
                {
                    grid_map::Position3 p3;
                    if (map.getPosition3("elevation", grid_map::Index(i, j), p3))
                    {
                        if (std::isnan(p3.z()))
                        {
                            map["elevation"](i, j) = 0;
                        }
                    }
                    else
                    {
                        map["elevation"](i, j) = 0;
                    }
                }
            }
        }
        else
        {
            cout<<"can not get end"<<endl;
        }
    }
    else
    {
        cout<<"can get start"<<endl;
    }

    // // 使用地图转成有序点云
    pcl::PointCloud<pcl::PointXYZ> org_pc = gridMap2Pointcloud(map);
    pcl::io::savePCDFileASCII(package_path + "/data/submap.pcd", org_pc);
    LOG(INFO)<<"-------------------------";
    // is_finish = true;

    pd.detect(org_pc);
    for (int i = 0; i < pd.planes_info.size(); i++)
    {
        if (pd.planes_info.at(i).normal.z() < 0)
        {
            pd.planes_info.at(i).normal = - pd.planes_info.at(i).normal;
            LOG(INFO)<<pd.planes_info.at(i).normal.transpose();
        }
    }
    LOG(INFO)<<pd.planes_info.size();
    vector<planeInfo>  planes_info = pd.planes_info;
    // LOG(INFO)<<planes.size();
    // // 平面分割结果
    cv::Mat result = pd.result;
    vector<cv::Mat> single_results = pd.planes;
    for (auto & tmp_image : single_results)
    {
        cv::imshow("tmp_image", tmp_image);
        cv::waitKey(0);
    }
    
    
    class Graph {
    private:
        unordered_map<int, vector<int>> adjList;

        void DFS(int node, unordered_set<int>& visited, vector<int>& component) {
            visited.insert(node);
            component.push_back(node);
            for (int neighbor : adjList[node]) {
                if (visited.find(neighbor) == visited.end()) {
                    DFS(neighbor, visited, component);
                }
            }
        }

    public:
        void addEdge(int src, int dest) {
            adjList[src].push_back(dest);
            adjList[dest].push_back(src); // 因为是无向图
        }

        vector<vector<int>> findConnectedComponents() {
            unordered_set<int> visited;
            vector<vector<int>> components;

            for (const auto& pair : adjList) {
                int node = pair.first;
                if (visited.find(node) == visited.end()) {
                    vector<int> component;
                    DFS(node, visited, component);
                    components.push_back(component);
                }
            }

            return components;
        }
    };

    Graph g;
    LOG(INFO)<<planes_info.size();
    for (int i = 0; i < planes_info.size(); i++)
    {
        for (int j = 0; j < planes_info.size(); j++)
        {
            // LOG(INFO)<<abs(planes_info.at(i).first.dot(planes_info.at(j).first));
            if (abs(planes_info.at(i).normal.dot(planes_info.at(j).normal)) > 0.95)
            {
                // LOG(INFO)<<abs((planes_info.at(i).second - planes_info.at(j).second).dot(planes_info.at(i).first));
                if (abs((planes_info.at(i).center - planes_info.at(j).center).dot(planes_info.at(i).normal)) < 0.02)
                {
                    // cout<<"add: "<<i<<"-"<<j<<endl;
                    g.addEdge(i, j);
                }
            }
        }
    }
    vector<vector<int>> components = g.findConnectedComponents();
    for (int i = 0; i < components.size(); i++)
    {
        for (int j = 0; j < components.at(i).size(); j++)
        {
            cout<<components.at(i).at(j)<<" ";
        }
        cout<<endl;
    }
    vector<cv::Mat> merge_results;
    vector<planeInfo> merge_planes;
    for (int i = 0; i < components.size(); i++)
    {
        cv::Mat image = cv::Mat::zeros(seg_result_image.size(), seg_result_image.type());
        for (int j = 0; j < components.at(i).size(); j++)
        {
            cv::Mat mask;
            cv::inRange(single_results.at(components.at(i).at(j)), cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), mask);
            single_results.at(components.at(i).at(j)).copyTo(image, mask);
        }
        merge_results.emplace_back(image);
        merge_planes.emplace_back(planes_info.at(components.at(i).front()));
    }

    for (int i = 0; i < merge_planes.size(); i++)
    {
        LOG(INFO)<<i<<" "<<merge_planes.at(i).normal.transpose()<<", "<<merge_planes.at(i).center.transpose();
        cv::imwrite(package_path + "/data/result_xx" + std::to_string(i) + ".png", merge_results.at(i));
    }
    
    vector<cv::Mat> collision_free_images;
    // // // 地图膨胀层
    double resolution = map.getResolution();
    double inflation_radius = 0.5;
    int inflation_pixel = 0.5/resolution;
    for (int i = 0; i < merge_results.size(); i++)
    {
        cv::Mat image = merge_results.at(i);
        // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/image" + std::to_string(i) + ".png", image);
        // cv::imshow("image", image);
        // cv::waitKey(0);
        int kernel_size = inflation_pixel;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
        // 对图像进行膨胀操作
        cv::Mat dilated_image;
        cv::dilate(image, dilated_image, kernel);
        // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/dilated_image" + std::to_string(i) + ".png", dilated_image);
        // cv::imshow("dilated_image", dilated_image);
        // cv::waitKey(0);
        // 计算膨胀后的边缘
        cv::Mat collision_layer = dilated_image - image;
        // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/collision_layer" + std::to_string(i) + ".png", collision_layer);
        // cv::imshow("collision_layer", collision_layer);
        // cv::waitKey(0);
        cv::Mat upper_body = cv::Mat::zeros(collision_layer.size(), CV_8UC1);
        cv::Mat knee = cv::Mat::zeros(collision_layer.size(), CV_8UC1);

        std::vector<cv::Point> white_points;
        cv::findNonZero(collision_layer, white_points);
        for (auto & cv_p : white_points)
        {
            grid_map::Position3 p3;
            if (map.getPosition3("elevation", grid_map::Index(cv_p.y, cv_p.x), p3))
            {
                // cout<<"  mm: "<cv_p<<endl;
                if (!std::isnan(p3.z()))
                {

                    // Eigen::Vector3f p3f(p3.x(), p3.y(), p3.z());
                    double dis = (p3 - merge_planes.at(i).center).dot(merge_planes.at(i).normal);

                    if (dis > 0.45)// 上半身
                    {
                        // cout<<"dis = "<<dis<<endl;
                        upper_body.at<uchar>(cv_p.y, cv_p.x) = 255;
                    }
                    else if (dis > 0.25) // 膝盖
                    {
                        // cout<<"dis = "<<dis<<endl;
                        knee.at<uchar>(cv_p.y, cv_p.x) = 255;
                    }
                }
            }
        }
        // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/knee" + std::to_string(i) + ".png", knee);
        // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/upper_body" + std::to_string(i) + ".png", upper_body);
        // cv::imshow("knee", knee);
        // cv::waitKey(0);
        // cv::imshow("upper_body", upper_body);
        // cv::waitKey(0);
        // // 进行不同半径的膨胀
        double upper_inflation = 0.4;
        double knee_inflation = 0.2;
        int inflation_radius_upper = upper_inflation/resolution;
        int inflation_radius_knee = knee_inflation/resolution;
        cv::Mat kernel_upper = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(inflation_radius_upper, inflation_radius_upper));
        cv::Mat kernel_knee = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(inflation_radius_knee, inflation_radius_knee));
        // // 对图像进行膨胀操作
        cv::Mat dilated_image_upper;
        cv::dilate(upper_body, dilated_image_upper, kernel_upper);
        upper_body_image = upper_body;
        upper_body_dilate = dilated_image_upper;
        cv::Mat dilated_image_knee;
        cv::dilate(knee, dilated_image_knee, kernel_knee);
        knee_image = knee;
        knee_image_dilate = dilated_image_knee;
        // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/dilated_image_upper" + std::to_string(i) + ".png", dilated_image_upper);
        // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/dilated_image_knee" + std::to_string(i) + ".png", dilated_image_knee);
        // cv::imshow("dilated_image_upper", dilated_image_upper);
        // cv::waitKey(0);
        // cv::imshow("dilated_image_knee", dilated_image_knee);
        // cv::waitKey(0);
        // 计算膨胀后的边缘
        cv::Mat collision_layer1 = image - dilated_image_upper;
        cv::Mat free_collision = collision_layer1 - dilated_image_knee;
        // cv::imwrite("/home/lichao/TCDS/src/pip_line/data/free_collision" + std::to_string(i) + ".png", free_collision);
        // cv::imshow("free_collision", free_collision);
        // cv::waitKey(0);
        collision_free_images.emplace_back(free_collision);
    }
    // // 构造适合落脚点规划的地图
    // 显示可通行地图
    
    feasible_map = map;

    map.add("label");
    cv::Mat plane_image = cv::Mat::zeros(result.size(), CV_8UC3);
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            bool flag = false;
            for (int label = 0; label < collision_free_images.size(); label++)
            {
                if (collision_free_images.at(label).at<uchar>(i, j) == 255)
                {
                    flag = true;
                    map["label"](i ,j) = label;
                    // 后续计算
                    plane_image.at<cv::Vec3b>(i, j) = cv::Vec3b(default_colors[(static_cast<int>(label))%12][0], default_colors[(static_cast<int>(label))%10][1], default_colors[(static_cast<int>(label))%12][2]);
                    break;
                }
            }
            if (!flag)
            {
                map["label"](i,j) = NAN;
                feasible_map["elevation"](i, j) = NAN;
            }
        }
    }
    
    is_finish = true;
    cv::imwrite(package_path + "/data/plane_image.png", plane_image);
    
    while (!get_goal)
    {
        sleep(1);
    }
    cout<<"get goal"<<endl;
    
    // 落脚点规划的输入，带"label"的高程图，
    // 台阶用0.175更合适，斜坡0.16就可以
// #ifdef OURMETHOD
//     FootParam footparam(0.16, 0.11, 0.065, 0.065, 0.1, 0);
// #else
//     FootParam footparam(0.16, 0.11, 0.065, 0.065);
// #endif
FootParam footparam(0.16, 0.11, 0.065, 0.065);
    AstarHierarchicalFootstepPlanner planner(map, plane_image, collision_free_images, merge_planes, footparam, 0.2);
    Eigen::Vector3d left_foot(0.0, 0.1, 0);
    Eigen::Vector3d right_foot(0.0, -0.1, 0);
    Eigen::Vector3d goal_p;
    goal_p.x() = goal.position.x;
    goal_p.y() = goal.position.y;
    Eigen::Quaterniond qd(goal.orientation.w, goal.orientation.x, goal.orientation.y, goal.orientation.z);
    Eigen::Vector3d v_t = qd.toRotationMatrix() * Eigen::Vector3d::UnitX();
    double yaw = atan(v_t.y()/v_t.x());
    goal_p.z() = yaw;

    clock_t start_plane = clock();
    if (planner.initial(right_foot, left_foot, 1, goal_p))// 先迈右脚
    {
        LOG(INFO)<<"set start and goal";
        if (planner.plan())
        {
             // 记录结束时间
            clock_t end_plan = clock();
    
            // 计算消耗的时间（毫秒）
            double duration_plan = double(end_plan - start_plane) / CLOCKS_PER_SEC * 1000;

            LOG(INFO)<<"TOTAL TIME: "<<duration_plan;
            steps = planner.getResultSteps();
            LOG(INFO)<<"ERROR";
            avoid_points = planner.computeAvoidPoints();
            LOG(INFO)<<"ERROR";

            // for (auto & step : steps)
            // {
            //     cout<<setw(8)<<"step: "<<step.x<<" "<<step.y<<" "<<step.z<<" "<<step.roll<<" "<<step.pitch<<" "<<step.yaw*57.3<<" "<<step.robot_side<<endl;
            //     cout<<setw(8)<<"points: "<<
            // }
            LOG(INFO)<<steps.size();
            for (int i = 0; i < steps.size(); i++)
            {
                cout<<setw(8)<<"step "<<i<<": "<<steps.at(i).x<<" "<<steps.at(i).y<<" "<<steps.at(i).z<<" "<<steps.at(i).roll*57.3<<" "<<steps.at(i).pitch*57.3<<" "<<steps.at(i).yaw*57.3<<" "<<steps.at(i).robot_side<<endl;
                cout<<"points: "<<endl;

                
                for (auto & point : avoid_points.at(i))
                {
                    cout<<setw(8)<<point.transpose()<<endl;
                }
            }
            publishSteps();
            // LOG(INFO)<<"ERROR";
            publishAvoidpoints();
            // LOG(INFO)<<"ERROR";
            publishVisualSteps();
            // LOG(INFO)<<"ERROR";
            publishVisualAvoidpoints();
            // LOG(INFO)<<"ERROR";
        }
        else
        {
            LOG(INFO)<<"planning error";
        }
    }
}


void pip_line::publishSteps()
{
    // LOG(INFO)<<"steps: "<<steps.size();
    diy_msgs::footSteps steps_pub;
    steps_pub.header.frame_id = "map";
    for (auto & s : steps)
    {
        diy_msgs::footStep tmp_step;
        tmp_step.is_left = s.robot_side == 0 ? true : false;
        tmp_step.x = s.x;
        tmp_step.y = s.y;
        tmp_step.z = s.z;
        tmp_step.roll = s.roll;
        tmp_step.pitch = s.pitch;
        tmp_step.yaw = s.yaw;
        steps_pub.footsteps.emplace_back(tmp_step);
    }
    footsteps_pub.publish(steps_pub);
}

void pip_line::publishAvoidpoints()
{
    diy_msgs::avoidPointsMsg points_pub;
    points_pub.header.frame_id = "map";
    for (auto & points : avoid_points)
    {
        diy_msgs::avoidPoints ps;
        // ps.terrainType
        for (auto & point : points)
        {
            geometry_msgs::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            ps.avoidPoints.emplace_back(p);
        }
        points_pub.avoidPointsMsg.emplace_back(ps);
    }
    avoid_points_pub.publish(points_pub);
}

void pip_line::publishVisualSteps()
{
    visualization_msgs::MarkerArray visual_steps;
    visualization_msgs::MarkerArray steps_arrow;
    int index_visual_step = 0;
    for (auto & step : steps)
    {
        visualization_msgs::Marker visual_step;
        visual_step.action = visualization_msgs::Marker::ADD;
        visual_step.header.frame_id = "map";  // 设置坐标系
        visual_step.id = index_visual_step;
        visual_step.type = visualization_msgs::Marker::MESH_RESOURCE;  // 表示网格模型
        Eigen::AngleAxisd ad_roll(step.roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd ad_pitch(step.pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd ad_yaw(step.yaw, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d r = ad_roll.toRotationMatrix() * ad_pitch.toRotationMatrix() * ad_yaw.toRotationMatrix();
        // Eigen::Matrix3d r_stl;
        // r_stl<<1, 0, 0, 0, -1, 0, 0, 0, -1;
        // Eigen::Quaterniond qd(r_stl * r);
        Eigen::Quaterniond qd(r);
        visual_step.pose.orientation.w = qd.w();
        visual_step.pose.orientation.x = qd.x();
        visual_step.pose.orientation.y = qd.y();
        visual_step.pose.orientation.z = qd.z();
        visual_step.pose.position.x = step.x - 0.09;
        visual_step.pose.position.y = step.y + 0.065;
        // visual_step.pose.position.y = step.y - 0.065;
        visual_step.pose.position.z = step.z;
        visual_step.scale.x = 1.0;  // 调整模型大小
        visual_step.scale.y = 1.0;
        visual_step.scale.z = 1.0;
        visual_step.color.a = 1.0;
        // 获取包的路径
        std::string package_path = ros::package::getPath("pip_line");
        // 设置模型的相对路径
        // mesh_resource_marker.mesh_resource = "file://" + package_path + "/path/to/your/model.stl";
        if (step.robot_side == 0)
        {
            visual_step.mesh_resource = "file://" + package_path + "/foot_visual/leftfoot4.STL";  // 设置STL文件路径
            // model_marker.mesh_resource = "file://" + package_path + "/data/left_foot.STL";  // 设置STL文件路径
        }
        else
        {
            visual_step.mesh_resource = "file://" + package_path + "/foot_visual/rightfoot4.STL";  // 设置STL文件路径
            // model_marker.mesh_resource = "file://" + package_path + "/data/right_foot.STL";  // 设置STL文件路径
        }
        // 220,223,227
        visual_step.color.r = 220.0/255.0;
        visual_step.color.g = 223.0/255.0;
        visual_step.color.b = 227.0/255.0;
        visual_steps.markers.emplace_back(visual_step);


        visualization_msgs::Marker step_arrow;
        step_arrow.action = visualization_msgs::Marker::ADD;
        step_arrow.header.frame_id = "map";  // 设置坐标系
        step_arrow.id = index_visual_step;
        step_arrow.ns = "arrows";
        step_arrow.type = visualization_msgs::Marker::ARROW;
        step_arrow.pose.position.x = step.x; // Offset each arrow by 2 units
        step_arrow.pose.position.y = step.y;
        step_arrow.pose.position.z = step.z;
        step_arrow.pose.orientation.x = qd.x();
        step_arrow.pose.orientation.y = qd.y();
        step_arrow.pose.orientation.z = qd.z();
        step_arrow.pose.orientation.w = qd.w();
        // Set the scale of the arrow
        step_arrow.scale.x = 0.4; // Shaft diameter
        step_arrow.scale.y = 0.05; // Head diameter
        step_arrow.scale.z = 0.05; // Head length

        if (step.robot_side == 0)
        {
            step_arrow.color.r = 1.0f;
            step_arrow.color.g = 0.0f;
            step_arrow.color.b = 0.0f;
            step_arrow.color.a = 1.0;
        }
        else
        {
            step_arrow.color.r = 0.0f;
            step_arrow.color.g = 1.0f;
            step_arrow.color.b = 0.0f;
            step_arrow.color.a = 1.0;
        }
        steps_arrow.markers.emplace_back(step_arrow);

        index_visual_step++;
    }
    footsteps_visual_pub.publish(visual_steps);
    footsteps_arrow_pub.publish(steps_arrow);
}

void pip_line::publishVisualAvoidpoints()
{
    visualization_msgs::MarkerArray visual_avoid_points;
    int index_avoid_points = 0;
    for (auto & points : avoid_points)
    {
        visualization_msgs::Marker visual_points;
        visual_points.action = visualization_msgs::Marker::ADD;
        visual_points.header.frame_id = "map";  // 设置坐标系
        visual_points.id = index_avoid_points;
        index_avoid_points ++;
        visual_points.type = visualization_msgs::Marker::POINTS;
        visual_points.color.r = 1.0f;
        visual_points.color.g = 0.0f;
        visual_points.color.b = 0.0f;
        visual_points.color.a = 1.0f;
        visual_points.scale.x = 0.05; // 点的宽度
        visual_points.scale.y = 0.05; // 点的高度
        visual_points.scale.z = 0.05; // 点的高度
        // visual_points.scale.x = 1.0f;
        // visual_points.scale.y = 1.0f;
        // visual_points.scale.z = 1.0f;
        for (auto & point : points)
        {
            geometry_msgs::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            visual_points.points.emplace_back(p);
        }
        // 这个地方并不考虑障碍点与步态点不对应，只是为了显示，不考虑对应关系
        if (!visual_points.points.empty())
        {
            visual_avoid_points.markers.emplace_back(visual_points);
        }
        
        
    }
    avoid_points_visual_pub.publish(visual_avoid_points);
}

void pip_line::timerCallback(const ros::TimerEvent & event)
{
    if (is_finish)
    {
        grid_map_msgs::GridMap map_msg;
        grid_map::GridMapRosConverter::toMessage(map, map_msg);
        map_msg.info.header.frame_id = "map";
        raw_heightmap_pub.publish(map_msg);

        grid_map_msgs::GridMap feasible_map_msg;
        grid_map::GridMapRosConverter::toMessage(feasible_map, feasible_map_msg);
        feasible_map_msg.info.header.frame_id = "map";
        feasible_map_pub.publish(feasible_map_msg);

        // sensor_msgs::PointCloud2 raw_heightmap_pc;
        // pcl::toROSMsg(org_pc, raw_heightmap_pc);
        // raw_heightmap_pc.header.frame_id = "map";
        // raw_heightmap_pc_pub.publish(raw_heightmap_pc);

        // std::vector<cv::Point> white_points;
        // cv::findNonZero(upper_body_image, white_points);
        // for (auto & cv_p : white_points)
        // {
        //     height_map_upper["elevation"](cv_p.y, cv_p.x) = map["elevation"](cv_p.y, cv_p.x);
        // }
        
        // white_points.clear();
        // cv::findNonZero(knee_image, white_points);
        // for (auto & cv_p : white_points)
        // {
        //     height_map_lower["elevation"](cv_p.y, cv_p.x) = map["elevation"](cv_p.y, cv_p.x);
        // }

        // grid_map_msgs::GridMap upper_msg;
        // grid_map::GridMapRosConverter::toMessage(height_map_upper, upper_msg);
        // upper_msg.info.header.frame_id = "map";
        // height_map_upper_pub.publish(upper_msg);

        // grid_map_msgs::GridMap lower_msg;
        // grid_map::GridMapRosConverter::toMessage(height_map_lower, lower_msg);
        // lower_msg.info.header.frame_id = "map";
        // height_map_lower_pub.publish(lower_msg);

        // sensor_msgs::ImagePtr result = cv_bridge::CvImage(std_msgs::Header(), "bgr8", seg_result_image).toImageMsg();

        // result->header.frame_id = "map";
        // seg_result_image_pub.publish(result);
        // grid_map_msgs::GridMap foot_msg;
        // grid_map::GridMapRosConverter::toMessage(height_map_foot, foot_msg);
        // foot_msg.info.header.frame_id = "map";
        // height_map_foot_pub.publish(foot_msg);

        // grid_map_msgs::GridMap height_msg;
        // grid_map::GridMapRosConverter::toMessage(plane_map, height_msg);
        // height_msg.info.header.frame_id = "map";
        // planes_all.publish(height_msg);

        // grid_map_msgs::GridMap cutted_msg;
        // grid_map::GridMapRosConverter::toMessage(plane_cutted, cutted_msg);
        // cutted_msg.info.header.frame_id = "map";
        // planes_cutted.publish(cutted_msg);

        // planes_polygon_pub.publish(planes_msg);
        // planes_polygon_cutted_pub.publish(planes_cutted_msg);
    }
    
}

void pip_line::goal_point_callback(geometry_msgs::PoseStamped::ConstPtr pose_p)
{
    ROS_INFO("get goal********************************************************8");
    std::lock_guard<std::mutex> lock(m_goal);
    goal.orientation = pose_p->pose.orientation;
    goal.position    = pose_p->pose.position;
    LOG(INFO)<<goal.orientation.w<<" "<<goal.orientation.x<<" "<<goal.orientation.y<<" "<<goal.orientation.z;
    LOG(INFO)<<goal.position.x<<" "<<goal.position.y<<" "<<goal.position.z;
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