#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <peac/PEAC_plane_detection.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
using namespace std;
plane_detection pd;

vector<Eigen::Matrix4d> steps_raw;
vector<Eigen::Matrix4d> steps_new;

Eigen::Vector3d Quaterniond2EulerAngles(Eigen::Quaterniond q) 
{
    Eigen::Vector3d angles;
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles(2) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles(0) = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}
Eigen::Vector3d Matrix3d2EulerAngles(Eigen::Matrix3d m)
{
    Eigen::Quaterniond qd(m);
    return Quaterniond2EulerAngles(qd);
}
void computeNewSteps()
{
    Eigen::Matrix4d world = Eigen::Matrix4d::Identity();
    world.block<3, 3>(0, 0) = steps_raw.at(0).block<3, 3>(0, 0); // Assuming the rotation is the same
    world.block<3, 1>(0, 3) = (steps_raw.at(0).block<3, 1>(0, 3) + steps_raw.at(1).block<3, 1>(0, 3)) / 2;
    LOG(INFO) << "World matrix: " << world;
    for (auto & step : steps_raw)
    {
        Eigen::Matrix4d new_step = world.inverse() * step;
        // LOG(INFO) << "Original step: " << step;
        steps_new.emplace_back(new_step);
        std::cout << new_step.block<3, 1>(0, 3).transpose().format(Eigen::IOFormat(3, 0, ", ", "", "", "", "[", "]"))
              << " " << Matrix3d2EulerAngles(new_step.block<3, 3>(0, 0)).transpose().format(Eigen::IOFormat(3, 0, ", ", "", "", "", "[", "]"));
    }
}
Eigen::Matrix3d computeRotationMatrix(const Eigen::Vector3d& u, const Eigen::Vector3d& v) 
{
    // 确保输入向量是单位向量
    Eigen::Vector3d u_normalized = u.normalized();
    Eigen::Vector3d v_normalized = v.normalized();
    
    // 计算旋转轴
    Eigen::Vector3d axis = u_normalized.cross(v_normalized);
    double sin_theta = axis.norm();
    double cos_theta = u_normalized.dot(v_normalized);
    
    // 防止数值误差影响
    if (sin_theta < 1e-10) {
        // 如果 sin(theta) 接近零，意味着 u 和 v 非常接近或相反
        if (cos_theta > 0.99999) {
            // u 和 v 非常接近
            return Eigen::Matrix3d::Identity();
        } else {
            // u 和 v 相反，选择一个正交向量作为旋转轴
            Eigen::Vector3d orthogonal = Eigen::Vector3d::UnitX();
            if (u_normalized.dot(orthogonal) > 0.9) {
                orthogonal = Eigen::Vector3d::UnitY();
            }
            axis = u_normalized.cross(orthogonal).normalized();
            return Eigen::AngleAxisd(M_PI, axis).toRotationMatrix();
        }
    }
    
    // 规范化旋转轴
    axis.normalize();
    
    // 构建 Rodrigues 旋转矩阵
    Eigen::Matrix3d K;
    K <<  0,         -axis.z(),  axis.y(),
          axis.z(),  0,         -axis.x(),
         -axis.y(),  axis.x(),  0;

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + sin_theta * K + (1 - cos_theta) * K * K;
    return R;
}


void computeRollPitch(Eigen::Vector3d normal, double yaw, Eigen::Vector3d & euler)
{
    // 注意根据斜面的法向量只能求取roll和pitch，yaw还是转角
    Eigen::Matrix3d T_f_m = computeRotationMatrix(Eigen::Vector3d::UnitZ(), normal);
    Eigen::AngleAxisd ad_m_w(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d T_f_w = T_f_m * ad_m_w.toRotationMatrix();
    euler = Matrix3d2EulerAngles(T_f_w);
}
void drawFilledRotatedRectangle(cv::Mat& image, cv::Point2f topLeftCorner, float width, float height, float angle) 
{
    // 创建矩形的初始点集 (以左上角点为参考点)
    cv::Point2f rectangleCorners[4] = {
        topLeftCorner,                                // 左上角
        cv::Point2f(topLeftCorner.x + width, topLeftCorner.y),           // 右上角
        cv::Point2f(topLeftCorner.x + width, topLeftCorner.y + height), // 右下角
        cv::Point2f(topLeftCorner.x, topLeftCorner.y + height)          // 左下角
    };

    // 构造旋转矩阵
    cv::Point2f center = topLeftCorner + cv::Point2f(width / 2.0f, height / 2.0f); // 中心点
    cv::Mat rotationMatrix = cv::getRotationMatrix2D(center, angle, 1.0);

    // 旋转每个矩形点
    cv::Point2f rotatedCorners[4];
    for (int i = 0; i < 4; i++) {
        cv::Mat rotatedPoint = rotationMatrix * (cv::Mat_<double>(3, 1) << rectangleCorners[i].x, rectangleCorners[i].y, 1);
        rotatedCorners[i] = cv::Point2f(rotatedPoint.at<double>(0, 0), rotatedPoint.at<double>(1, 0));
    }

    // 将点转化为整数坐标
    cv::Point intCorners[4];
    for (int i = 0; i < 4; i++) {
        intCorners[i] = rotatedCorners[i];
    }

    // 在图像上绘制填充的多边形
    cv::fillConvexPoly(image, intCorners, 4, 255);
}

visualization_msgs::MarkerArray getAreaMarker(vector<Eigen::Vector3d> steps, grid_map::GridMap & map)
{
    visualization_msgs::MarkerArray markerArray;
    
    // 根据点云提取平面，根据平面确定角度
    // 分为左右脚来
    for (int i = 0; i < steps.size(); i++)
    {
        if (i % 2 == 1)
        {
            // 绘制左脚
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "left_foot";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = steps.at(i).x(); // 中心点的x坐标
            marker.pose.position.y = steps.at(i).y(); // 中心点的y坐标

            grid_map::Position p(steps.at(i).x(), steps.at(i).y());
            grid_map::Index index;
            map.getIndex(p, index);
            int index_NO = -1;
            for (int i = 0; i < pd.planes.size(); i++)
            {
                if (pd.planes.at(i).at<uchar>(index.x(), index.y()) == 255)
                {
                    index_NO = i;
                    break;
                }
            }
            
            // pd.map.getIndex(p, index);
            Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
            if (index_NO != -1)
            {
                marker.pose.position.z = pd.planes_info.at(index_NO).getZ(steps.at(i).x(), steps.at(i).y()); 
                Eigen::Vector3d euler;
                // LOG(INFO)<<marker.pose.position.x<<" "<<marker.pose.position.y<<" "<<marker.pose.position.z;
                computeRollPitch(pd.planes_info.at(index_NO).normal, steps.at(i).z()/57.3, euler);
                // LOG(INFO)<<"euler: "<<euler(0)<<" "<<euler(1)<<" "<<euler(2);
                Eigen::Quaterniond q_new;
                q_new = Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
                tf.block<3, 3>(0, 0) = q_new.toRotationMatrix();
                tf.block<3, 1>(0, 3) = Eigen::Vector3d(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
                marker.pose.orientation.x = q_new.x();
                marker.pose.orientation.y = q_new.y();
                marker.pose.orientation.z = q_new.z();
                marker.pose.orientation.w = q_new.w();
            }
            steps_raw.emplace_back(tf);
            // marker.pose.position.z = 0; // 中心点的z坐标
    
            marker.scale.x = 0.27; // 矩形的宽度
            marker.scale.y = 0.13; // 矩形的高度
            marker.scale.z = 0.01; // 矩形的厚度

            // 设置颜色
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5;

            markerArray.markers.emplace_back(marker);
        }
        else
        {
            // 绘制右脚
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.ns = "right_foot";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = steps.at(i).x(); // 中心点的x坐标
            marker.pose.position.y = steps.at(i).y(); // 中心点的y坐标
            grid_map::Position p(steps.at(i).x(), steps.at(i).y());
            grid_map::Index index;
            map.getIndex(p, index);
            int index_NO = -1;
            for (int i = 0; i < pd.planes.size(); i++)
            {
                if (pd.planes.at(i).at<uchar>(index.x(), index.y()) == 255)
                {
                    index_NO = i;
                    break;
                }
            }
            Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
            if (index_NO != -1)
            {
                marker.pose.position.z = pd.planes_info.at(index_NO).getZ(steps.at(i).x(), steps.at(i).y()); 
                Eigen::Vector3d euler;
                computeRollPitch(pd.planes_info.at(index_NO).normal, steps.at(i).z()/57.3, euler);
                // LOG(INFO)<<marker.pose.position.x<<" "<<marker.pose.position.y<<" "<<marker.pose.position.z;
                // LOG(INFO)<<"euler: "<<euler(0)<<" "<<euler(1)<<" "<<euler(2);
                Eigen::Quaterniond q_new;
                q_new = Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
                tf.block<3, 3>(0, 0) = q_new.toRotationMatrix();
                tf.block<3, 1>(0, 3) = Eigen::Vector3d(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
                marker.pose.orientation.x = q_new.x();
                marker.pose.orientation.y = q_new.y();
                marker.pose.orientation.z = q_new.z();
                marker.pose.orientation.w = q_new.w();
            }
            steps_raw.emplace_back(tf);
            marker.scale.x = 0.27; // 矩形的宽度
            marker.scale.y = 0.13; // 矩形的高度
            marker.scale.z = 0.01; // 矩形的厚度

            // 设置颜色
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 0.5;
            markerArray.markers.emplace_back(marker);
        }
        
    }
    LOG(INFO)<<"markerArray.markers.size(): "<<markerArray.markers.size();
    return markerArray;
}

pcl::PointCloud<pcl::PointXYZ> gridMap2PointcloudOrganized(grid_map::GridMap & map)
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
int main(int argc, char **argv)
{
    ros::init(argc, argv, "grid_map_CBS");

    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<grid_map_msgs::GridMap>("map", 1, true);
    grid_map::GridMap map;
    map.add("elevation", 0.0);
    map.setFrameId("map");
    grid_map::Length length(5.0, 5.0);
    double resolution = 0.01;
    map.setGeometry(length, resolution, grid_map::Position(2.5, 0.0));

    grid_map::Position corner_start(5, 2.5);
    grid_map::Position corner_end(0, -2.5);
    grid_map::Index corner_start_index;
    grid_map::Index corner_end_index;
    map.getIndex(corner_start, corner_start_index);
    map.getIndex(corner_end, corner_end_index);
    LOG(INFO)<<"corner_start_index: "<<corner_start_index.x()<<" "<<corner_start_index.y();
    LOG(INFO)<<"corner_end_index: "<<corner_end_index.x()<<" "<<corner_end_index.y();

    for (int i = 0; i < 500; i++)
    {
        for (int j = 0; j < 500; j++)
        {
            map["elevation"](i, j) = 0.0;
        }
    }
    
    grid_map::Position slope_start(3, 0);
    grid_map::Position slope_end(slope_start.x() - 0.98, slope_start.y() - 1.2);
    grid_map::Index slope_start_index;
    grid_map::Index slope_end_index;
    map.getIndex(slope_start, slope_start_index);
    map.getIndex(slope_end, slope_end_index);

    LOG(INFO)<<"slope_start_index: "<<slope_start_index.x()<<" "<<slope_start_index.y();
    LOG(INFO)<<"slope_end_index: "<<slope_end_index.x()<<" "<<slope_end_index.y();
    for (int i = slope_start_index.x(); i < slope_end_index.x(); i++)
    {
        for (int j = slope_start_index.y(); j < slope_end_index.y(); j++)
        {
            // map["elevation"](i, j) = 0.15273 - resolution*(i - slope_start_index.x()) * 0.1405;
            map["elevation"](i, j) = 0.255 - resolution*(i - slope_start_index.x()) * 0.2449;
        }
    }
    
    cv::Mat map_mat = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);

    // 矩形参数 
    cv::Point2f topLeftCorner(270, 300); // 矩形左上角位置
    float width = 60;                  // 矩形宽度
    float height = 39.5;                 // 矩形高度
    float angle = 5;                   // 矩形旋转角度 (以度数为单位)

    // 绘制旋转矩形
    drawFilledRotatedRectangle(map_mat, topLeftCorner, width, height, angle);

    // 显示图像
    // cv::imshow("Rotated Rectangle", map_mat);
    // cv::waitKey(0);

    std::vector<cv::Point> whitePixels;
    cv::findNonZero(map_mat, whitePixels);
    for (auto & point : whitePixels)
    {
        grid_map::Index index(point.y, point.x);
        map["elevation"](index.x(), index.y()) = 0.04;
    }

    cv::Mat step_mat = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);

    // 矩形参数
    cv::Point2f step_topLeftCorner(195, 205); // 矩形左上角位置
    float step_width = 50;                  // 矩形宽度
    float step_height = 40;                 // 矩形高度
    float step_angle = 5;

    // 绘制旋转矩形
    drawFilledRotatedRectangle(step_mat, step_topLeftCorner, step_width, step_height, step_angle);
    // 显示图像
    // cv::imshow("Rotated Rectangle", step_mat);
    // cv::waitKey(0);


    std::vector<cv::Point> step_whitePixels;
    cv::findNonZero(step_mat, step_whitePixels);
    for (auto & point : step_whitePixels)
    {
        grid_map::Index index(point.y, point.x);
        map["elevation"](index.x(), index.y()) = 0.26;
    }

    cv::Mat obstacle_mat = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);

    // 矩形参数
    cv::Point2f obstacle_topLeftCorner(185, 280); // 矩形左上角位置
    float obstacle_width = 45;                  // 矩形宽度
    float obstacle_height = 30;                 // 矩形高度
    float obstacle_angle = 20;

    // 绘制旋转矩形
    drawFilledRotatedRectangle(obstacle_mat, obstacle_topLeftCorner, obstacle_width, obstacle_height, obstacle_angle);
    // 显示图像
    // cv::imshow("Rotated Rectangle", step_mat);
    // cv::waitKey(0);

    std::vector<cv::Point> obstacle_whitePixels;
    cv::findNonZero(obstacle_mat, obstacle_whitePixels);
    for (auto & point : obstacle_whitePixels)
    {
        grid_map::Index index(point.y, point.x);
        map["elevation"](index.x(), index.y()) = 1;
    }

    
    pcl::PointCloud<pcl::PointXYZ> org_pc = gridMap2PointcloudOrganized(map);
    pcl::io::savePCDFileASCII("/home/lichao/TCDS/src/pip_line/data/org_pc.pcd", org_pc);

    pd.initial("/home/lichao/TCDS/src/pip_line/config/plane_fitter_pcd.ini");
    pd.detect(org_pc);
    LOG(INFO)<<"PLANE SIZE: "<< pd.planes.size();


    // 根据角点确定地图中各元素的角点

    vector<vector<Eigen::Vector2d>> corner_points;
    for (int i = 0; i < pd.planes.size(); i++)
    {
        LOG(INFO)<<"image I: "<<i;
        auto image = pd.planes.at(i).clone();
        // Find contours in the binary image
        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            // Approximate contour to polygon
            vector<cv::Point> approx;
            cv::approxPolyDP(contour, approx, cv::arcLength(contour, true) * 0.02, true);

            // Check if the approximated contour has 4 points (i.e., it's a rectangle)
            if (approx.size() == 4) {
                // Draw the contour and the approximated polygon
                cv::drawContours(image, vector<vector<cv::Point>>{approx}, -1, cv::Scalar(255), 2);

                // Print the corner points
                vector<Eigen::Vector2d> corner_points_i;
                for (const auto& point : approx) {
                    LOG(INFO) << "Corner point: " << point;
                    Eigen::Vector2d corner_point;
                    map.getPosition(grid_map::Index(point.y, point.x), corner_point);
                    corner_points_i.emplace_back(corner_point);
                }
                corner_points.emplace_back(corner_points_i);
            }
        }
        // cv::imshow("image", image);
        // cv::waitKey(0);

    }
       

    vector<ahc::PlaneSeg::Stats> statses(pd.planes.size());
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            bool flag = false;
            for (int label = 0; label < pd.planes.size(); label++)
            {
                if (pd.planes.at(label).at<uchar>(i, j) == 255)
                {
                    flag = true;
                    grid_map::Position3 p3;
                    if (map.getPosition3("elevation", grid_map::Index(i, j), p3))
                    {
                        statses.at(label).push(p3.x(), p3.y(), p3.z());
                    }
                    break;
                }
            }
        }
    }
    
    pd.planes_info.clear();
    for (auto & stats : statses)
    {
        // plane_info.update();
        double center[3], normal[3];
        double mse, curvature;
        stats.compute(center, normal, mse, curvature);
        planeInfo pi;
        pi.center = Eigen::Vector3d(center[0], center[1], center[2]);
        if (normal[2] < 0)
        {
            pi.normal = Eigen::Vector3d(-normal[0], -normal[1], -normal[2]);
        }
        else
        {
            pi.normal = Eigen::Vector3d(normal[0], normal[1], normal[2]);
        }
        pd.planes_info.emplace_back(pi);
    }
    // for (auto & plane : pd.planes)
    // {
    //     cv::imshow("plane", plane);
    //     cv::waitKey(0);
    // }
    
    // planes_info = pd.planes_info;
    // single_results = pd.planes;

    // 针对障碍，除去可能发生碰撞的区域
    cv::Mat obstacle_mat_inflat = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
    for (int i = 0; i < obstacle_mat_inflat.rows; i++)
    {
        for (int j = 0; j < obstacle_mat_inflat.cols; j++)
        {
            if (map["elevation"](i, j) > 0.8)
            {
                obstacle_mat_inflat.at<uchar>(i, j) = 255;
            }
        }
    }
    // 膨胀操作
    int dilation_size = 30; // 膨胀的大小，可以根据需要调整

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1));
    // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                // cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                                // cv::Point(dilation_size, dilation_size));
    cv::dilate(obstacle_mat_inflat, obstacle_mat_inflat, element);
    // cv::imshow("obstacle_mat_inflat", obstacle_mat_inflat);
    // cv::waitKey(0);

    std::vector<cv::Point> inflat_whitePixels;
    cv::findNonZero(obstacle_mat_inflat, inflat_whitePixels);
    for (auto & point : inflat_whitePixels)
    {
        grid_map::Index index(point.y, point.x);
        map["elevation"](index.x(), index.y()) = NAN;
    }

    
    
    vector<Eigen::Vector3d> steps;
    Eigen::AngleAxisd ad(20.0/180*M_PI, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d start(1.15, -0.5, 0.0);
    
    // Eigen::AngleAxisd 
    Eigen::Vector3d right = ad.toRotationMatrix() * Eigen::Vector3d(0, -0.2, 0) + start;

    Eigen::Vector2d center = (start + right).head(2)/2.0;   
    for (auto & corner_points : corner_points)
    {
        for (auto & point : corner_points)
        {
            point -= center;
            LOG(INFO) << "corner_point: " << point;
        }
        cout<<endl<<endl;
    }

    
    steps.emplace_back(Eigen::Vector3d(right.x(), right.y(), 20));
    steps.emplace_back(Eigen::Vector3d(start.x(), start.y(), 20));
    
    Eigen::Vector3d step3 = Eigen::Vector3d(0.23, 0.05, 0) + right;
    steps.emplace_back(Eigen::Vector3d(step3.x(), step3.y(), 10));

    Eigen::Vector3d step2 = Eigen::Vector3d(0.57, 0.1, 0) + start;
    steps.emplace_back(Eigen::Vector3d(step2.x(), step2.y(), 10));

    Eigen::Vector3d step4 = Eigen::Vector3d(0.65, 0.08, 0) + right;
    steps.emplace_back(Eigen::Vector3d(step4.x(), step4.y(), 10));

    Eigen::Vector3d step5 = Eigen::Vector3d(1, 0.19, 0) + start;
    steps.emplace_back(Eigen::Vector3d(step5.x(), step5.y(), 15));

    Eigen::Vector3d step6 = Eigen::Vector3d(1.2, 0.2, 0) + right;
    steps.emplace_back(Eigen::Vector3d(step6.x(), step6.y(), 20));

    Eigen::Vector3d step7 = Eigen::Vector3d(1.35, 0.3, 0) + start;
    steps.emplace_back(Eigen::Vector3d(step7.x(), step7.y(), 25));

    Eigen::Vector3d step8 = Eigen::Vector3d(1.45, 0.3, 0) + right;
    steps.emplace_back(Eigen::Vector3d(step8.x(), step8.y(), 30));

    Eigen::Vector3d step9 = Eigen::Vector3d(1.4, 0.35, 0) + start;
    steps.emplace_back(Eigen::Vector3d(step9.x(), step9.y(), 40));

    Eigen::Vector3d step10 = Eigen::Vector3d(1.5, 0.4, 0) + right;
    steps.emplace_back(Eigen::Vector3d(step10.x(), step10.y(), 45));

    Eigen::Vector3d step11 = Eigen::Vector3d(1.4, 0.35, 0) + start;
    steps.emplace_back(Eigen::Vector3d(step11.x(), step11.y(), 55));

    Eigen::Vector3d step12 = Eigen::Vector3d(1.57, 0.55, 0) + right;
    steps.emplace_back(Eigen::Vector3d(step12.x(), step12.y(), 60));

    Eigen::Vector3d step13 = Eigen::Vector3d(1.51, 0.69, 0) + start;
    steps.emplace_back(Eigen::Vector3d(step13.x(), step13.y(), 60));

    Eigen::Vector3d step14 = Eigen::Vector3d(1.62, 0.88, 0) + right;
    steps.emplace_back(Eigen::Vector3d(step14.x(), step14.y(), 60));

    Eigen::Vector3d step15 = Eigen::Vector3d(1.55, 0.89, 0) + start;
    steps.emplace_back(Eigen::Vector3d(step15.x(), step15.y(), 55));

    Eigen::Vector3d step16 = Eigen::Vector3d(1.6, 0.88, 0) + right;
    steps.emplace_back(Eigen::Vector3d(step16.x(), step16.y(), 45));

    // Eigen::Vector3d step4 = Eigen::Vector3d(0.72, 0, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step4.x(), step4.y(), 0));

    // Eigen::Vector3d step6 = Eigen::Vector3d(1.05, 0., 0) + right;
    // steps.emplace_back(Eigen::Vector3d(step6.x(), step6.y(), 0));

    // Eigen::Vector3d step7 = Eigen::Vector3d(1.05, 0., 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step7.x(), step7.y(), 0));

    // Eigen::Vector3d step8 = Eigen::Vector3d(1.2, 0., 0) + right;
    // steps.emplace_back(Eigen::Vector3d(step8.x(), step8.y(), 10));

    // Eigen::Vector3d step9 = Eigen::Vector3d(1.2, 0.1, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step9.x(), step9.y(), 20));

    // Eigen::Vector3d step10 = Eigen::Vector3d(1.45, 0.15, 0) + right;
    // steps.emplace_back(Eigen::Vector3d(step10.x(), step10.y(), 25));

    // Eigen::Vector3d step11 = Eigen::Vector3d(1.45, 0.25, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step11.x(), step11.y(), 35));

    // Eigen::Vector3d step12 = Eigen::Vector3d(1.55, 0.23, 0) + right;
    // steps.emplace_back(Eigen::Vector3d(step12.x(), step12.y(), 40));

    // Eigen::Vector3d step13 = Eigen::Vector3d(1.45, 0.25, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step13.x(), step13.y(), 45));

    // Eigen::Vector3d step14 = Eigen::Vector3d(1.67, 0.38, 0) + right;
    // steps.emplace_back(Eigen::Vector3d(step14.x(), step14.y(), 50));

    // Eigen::Vector3d step15 = Eigen::Vector3d(1.45, 0.28, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step15.x(), step15.y(), 55));

    // Eigen::Vector3d step16 = Eigen::Vector3d(1.67, 0.44, 0) + right;
    // steps.emplace_back(Eigen::Vector3d(step16.x(), step16.y(), 65));

    // Eigen::Vector3d step17 = Eigen::Vector3d(1.52, 0.55, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step17.x(), step17.y(), 70));

    // Eigen::Vector3d step18 = Eigen::Vector3d(1.7, 0.8, 0) + right;
    // steps.emplace_back(Eigen::Vector3d(step18.x(), step18.y(), 65));

    // Eigen::Vector3d step19 = Eigen::Vector3d(1.52, 0.8, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step19.x(), step19.y(), 65));

    // Eigen::Vector3d step20 = Eigen::Vector3d(1.67, 0.8, 0) + right;
    // steps.emplace_back(Eigen::Vector3d(step20.x(), step20.y(), 55));

    // Eigen::Vector3d step7 = Eigen::Vector3d(1, 0.25, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step7.x(), step7.y(), 15));

    // Eigen::Vector3d step7 = Eigen::Vector3d(0.88, 0.25, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step7.x(), step7.y(), 15));

    // Eigen::Vector3d step8 = Eigen::Vector3d(1.12, 0.1, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step8.x(), step8.y(), 25));

    // Eigen::Vector3d step9 = Eigen::Vector3d(1.18, 0.4, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step9.x(), step9.y(), 40));

    // Eigen::Vector3d step10 = Eigen::Vector3d(1.42, 0.29, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step10.x(), step10.y(), 50));

    // Eigen::Vector3d step11 = Eigen::Vector3d(1.3, 0.46, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step11.x(), step11.y(), 65));

    // Eigen::Vector3d step12 = Eigen::Vector3d(1.52, 0.44, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step12.x(), step12.y(), 70));

    // Eigen::Vector3d step13 = Eigen::Vector3d(1.36, 0.78, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step13.x(), step13.y(), 70));

    // Eigen::Vector3d step14 = Eigen::Vector3d(1.55, 0.77, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step14.x(), step14.y(), 65));

    // Eigen::Vector3d step15 = Eigen::Vector3d(1.38, 1, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step15.x(), step15.y(), 55));

    // Eigen::Vector3d step16 = Eigen::Vector3d(1.53, 0.83, 0) + start;
    // steps.emplace_back(Eigen::Vector3d(step16.x(), step16.y(), 45));
    
    // 画步态点
    visualization_msgs::MarkerArray markerArray = getAreaMarker(steps, map);
    
    LOG(INFO)<<"markerArray.markers.size(): "<<markerArray.markers.size();
    // 发布Marker消息
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
    computeNewSteps();
    // ros::Publisher points_pub = nh.advertise<visualization_msgs::Marker>("obstacle_points", 1);

    grid_map_msgs::GridMap map_msg;
    grid_map::GridMapRosConverter::toMessage(map, map_msg);
    ros::Rate loop_rate(2);
    while (ros::ok())
    {
        map_pub.publish(map_msg);
        marker_pub.publish(markerArray);
        // points_pub.publish(obstacle_point);
        loop_rate.sleep();
    }
    return 0;

}
