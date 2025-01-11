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
#include <pcl_conversions/pcl_conversions.h>
using namespace std;


class CBS_image
{
private:
    ros::NodeHandle n;
    ros::Subscriber sub_image;
    bool first = true;
    ros::Subscriber sub_pc;
    cv::Mat scence_image;
    pcl::PointCloud<pcl::PointXYZ> org_pc;
    plane_detection pd;
    int seg_index = 0;

    unsigned int default_colors[10][3] =
    {
        {255, 0, 0},
        {255, 255, 0},
        {100, 20, 50},
        {0, 30, 255},
        {10, 255, 60},
        {80, 10, 100},
        {0, 255, 200},
        {10, 60, 60},
        {255, 0, 128},
        {60, 128, 128}
    };
public:
    CBS_image(ros::NodeHandle & nh);
    void senceimage_callback(const sensor_msgs::ImageConstPtr & msg);
    void pcd_callback(const sensor_msgs::PointCloud2ConstPtr & msg);
    void computeNewSteps();
};


CBS_image::CBS_image(ros::NodeHandle & nh):n(nh)
{
    this->sub_image = nh.subscribe("/camera/color/image_raw", 1, &CBS_image::senceimage_callback, this);
    this->sub_pc = nh.subscribe("/camera/depth/color/points", 1, &CBS_image::pcd_callback, this);
    pd.initial("/home/lichao/TCDS/src/pip_line/config/plane_fitter_pcd.ini");
}

void CBS_image::senceimage_callback(const sensor_msgs::ImageConstPtr & msg)
{
    ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception & e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    this->scence_image = cv_ptr->image;
    cv::imwrite("/home/lichao/TCDS/src/pip_line/data/CBS_paper_image/scence_image.jpg", scence_image);
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
    const double sigmaGauss = 2.0;
    const int kSizeGauss = 9;
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


void CBS_image::pcd_callback(const sensor_msgs::PointCloud2ConstPtr & msg)
{
    if (first)
    {
        ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());
        pcl::PointCloud<pcl::PointXYZ> org_pc;
        // pcl::fromROSMsg(*msg, org_pc);
        pcl::io::loadPCDFile("/home/lichao/TCDS/src/pip_line/data/CBS_paper_image/example.pcd", org_pc);
        // this->org_pc = org_pc;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = org_pc.makeShared();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *cloud = org_pc;
        
        GaussPointCloud(cloud_in, cloud);

        pd.detect(*cloud);

        cv::Mat color_image = cv::Mat::zeros(pd.result.size(), CV_8UC3);

        for (int i = 0; i < pd.planes.size(); i++)
        {
            auto plane = pd.planes.at(i).clone();
            cv::imwrite("/home/lichao/TCDS/src/pip_line/data/CBS_paper_image/plane" + std::to_string(seg_index) + std::to_string(i) + ".jpg", plane);
            // cv::imshow("plane", plane);
            // cv::waitKey(0);
            // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
            // cv::morphologyEx(plane, plane, cv::MORPH_OPEN, kernel);
            // cv::morphologyEx(plane, plane, cv::MORPH_CLOSE, kernel);

            // cv::imshow("plane1", plane);
            // cv::waitKey(0);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(plane, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            contours.erase(std::remove_if(contours.begin(), contours.end(),
            [](const std::vector<cv::Point>& contour) {
                return cv::contourArea(contour) < 5000; // Adjust the threshold as needed
            }), contours.end());

            // Approximate contours
            std::vector<std::vector<cv::Point>> approxContours(contours.size());
            for (size_t j = 0; j < contours.size(); ++j)
            {
                cv::approxPolyDP(contours[j], approxContours[j], 10, true); // Adjust the epsilon as needed
            }

            // Draw contours on the result image
            

            // Fill contours with default colors
            for (size_t j = 0; j < approxContours.size(); ++j)
            {
                cv::Scalar color(default_colors[i % 10][0], default_colors[i % 10][1], default_colors[i % 10][2]);
                cv::drawContours(color_image, approxContours, static_cast<int>(j), color, cv::FILLED);
            }

            // Draw contours on the result image
            cv::drawContours(color_image, approxContours, -1, cv::Scalar(0, 255, 0), 2);
        }
        cv::imwrite("/home/lichao/TCDS/src/pip_line/data/CBS_paper_image/seg_raw_result" + std::to_string(seg_index) + ".jpg", pd.result);
        // cv::imwrite("/home/lichao/TCDS/src/pip_line/data/CBS_paper_image/seg_result" + std::to_string(seg_index) + ".jpg", color_image);
        // pcl::io::savePCDFileASCII("/home/lichao/TCDS/src/pip_line/data/CBS_paper_image/org_pc" + std::to_string(seg_index) + ".pcd", org_pc);
        seg_index ++;
        return;
    }
    else
    {
        return;
    }
    
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grid_map_CBS_image");
    ros::NodeHandle nh;
    CBS_image cbs_image(nh);
    ros::spin();
    return 0;

}
