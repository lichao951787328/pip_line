/*
 * @Author: lichao951787328 951787328@qq.com
 * @Date: 2024-06-18 22:27:39
 * @LastEditors: lichao951787328 951787328@qq.com
 * @LastEditTime: 2024-06-18 23:02:22
 * @FilePath: /pip_line/src/plane_detection/plane_detection_node.cpp
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <plane_detection/plane_segmentation.h>
#include <plane_detection/load_parameter.h>
#include <ros/package.h>
void initial_package_path(string package_name, string & package_path)
{
  package_path = ros::package::getPath(package_name);
  // 检查是否成功找到包路径
  if (package_path.empty()) {
      std::cerr << "Error: Could not find package " << package_name << std::endl;
  }
  cout<<"package path: "<<package_path<<endl;
}

void initialMatrix(Eigen::Matrix4f & ROBOTWORLD_T_CAMERA)
{
  float BaseVisionZ = +1.1258e+00-+7.1365e-01+0.18938;
  BaseVisionZ -= (77.51 - 12.81)/1000;
  float BaseVisionX = 0.15995;
  BaseVisionX -= (82.4 - 65.17)/1000;
  float BaseVisionY = 0.0;
  float BaseVisionPitchDeg = 27.5;
  Eigen::Matrix4f Base_T_Vision, Vision_T_Tar, World_T_Base, World_T_Tar;
  Base_T_Vision = getT(BaseVisionX, BaseVisionY, BaseVisionZ, 0, 0, 0);
  Eigen::Matrix3f Base_R_VisionTemp;
  Base_R_VisionTemp << 0,-1,0, -1,0,0, 0,0,-1;
  Base_T_Vision.block<3,3>(0,0) = Base_R_VisionTemp*(Eigen::AngleAxisf(_deg2rad(BaseVisionPitchDeg),Eigen::Vector3f::UnitX())).matrix();
  World_T_Base = getT(-0.015, 0, 0.65, 0.0, 0.0, 0.0);
  Vision_T_Tar.setIdentity();
  World_T_Tar = World_T_Base*Base_T_Vision*Vision_T_Tar;
  ROBOTWORLD_T_CAMERA = World_T_Tar;
}

class plane_detection
{
private:
    ros::NodeHandle nh;
    ros::Subscriber pointcloud_sub;
public:
    plane_detection(ros::NodeHandle & n);
    void pointcloud_callback(sensor_msgs::PointCloud2::ConstPtr msg);
    ~plane_detection();
};

plane_detection::plane_detection(ros::NodeHandle & n):nh(n)
{
    pointcloud_sub = nh.subscribe("points", 1, &plane_detection::pointcloud_callback, this);
}

void plane_detection::pointcloud_callback(sensor_msgs::PointCloud2::ConstPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc);

    Eigen::Matrix4f ROBOTWORLD_T_CAMERA;
    ROBOTWORLD_T_CAMERA.setIdentity();
    initialMatrix(ROBOTWORLD_T_CAMERA);

    clock_t start_time_true = clock();

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
    std::cout<<"construct quatree costs "<<(double)(clock() - start_time_true) / CLOCKS_PER_SEC <<" s. "<<std::endl;
    cv::imshow("seg result", result);
    cv::waitKey(10);

    // 设计思路
    // 第一步：构建原始点云高程图，将高程图转化为有序点云，对点云求取平面区域，求（可以先将原地图进行膨胀，再寻找膨胀区域内的点）高城图点在这个平面参数之上的栅格点组成的高程图，再将这些点进行分类膝盖层、上半身层，利用这两层膨胀，去除每个区域的危险区域。对每一个平面都进行此操作，能够得到无碰撞的平面栅格，再在次基础上进行落脚点规划。
}

plane_detection::~plane_detection()
{
}
