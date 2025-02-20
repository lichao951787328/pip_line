#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerTraditional.h>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <chrono>

void AstarHierarchicalFootstepPlannerTraditional::setCheckParam(double checkXupper_, double checkXButton_)
{
    checkXupper = checkXupper_;
    checkXButton = checkXButton_;
}

// tested 粗略检查
bool AstarHierarchicalFootstepPlannerTraditional::computeLandInfo(Eigen::Vector3d ankle, int & max_size, int & above_points, Eigen::Vector3d & plane_normal, double & step_height, double & pitch, double & roll)
{
    checktime++;
    auto start = std::chrono::high_resolution_clock::now();
   
    max_size = 0;
    above_points = 0;
    plane_normal = Eigen::Vector3d::Zero();
    step_height = -std::numeric_limits<double>::infinity();
    pitch = std::numeric_limits<double>::infinity();
    roll = std::numeric_limits<double>::infinity();
    // LOG(INFO)<<checkXupper<<" "<<checkXButton<<" "<<footparam.y_left<<" "<<footparam.y_right<<" "<<footparam.x_upper<<" "<<footparam.x_button;
    Eigen::AngleAxisd ax(ankle.z(), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d mid(ankle.x(), ankle.y(), 0);
    // 初次只用前后8cm的作为支撑平面的判断
    Eigen::Vector3d mid_top = ax.toRotationMatrix() * Eigen::Vector3d(checkXupper, 0, 0) + mid;
    Eigen::Vector3d mid_down = ax.toRotationMatrix() * Eigen::Vector3d(- checkXButton, 0, 0) + mid;
    Eigen::Vector3d top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left - 0.1, 0) + mid_top;
    Eigen::Vector3d top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - (footparam.y_right - 0.1), 0) + mid_top;
    Eigen::Vector3d down_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left - 0.1, 0) + mid_down;
    Eigen::Vector3d down_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - (footparam.y_right - 0.1), 0) + mid_down;
    // grid_map::Position top_left_l(top_left.x(), top_left.y());
    // grid_map::Position top_right_l(top_right.x(), top_right.y());
    // grid_map::Position down_left_l(down_left.x(), down_left.y());
    // grid_map::Position down_right_l(down_right.x(), down_right.y());
    vector<Eigen::Vector3d> points;
    if (SqurePoints(top_left.head(2), top_right.head(2), down_left.head(2), down_right.head(2), points))
    {
#ifdef DEBUG
        LOG(INFO)<<"points size "<<points.size();
#endif
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        for (auto & point : points)
        {
            sum += point;
        }
        Eigen::Vector3d center = sum / points.size();
        Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
        for (auto & point : points)
        {
            M += (point - center)*(point - center).transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(M);  
        Eigen::Vector3d eigenvalues = es.eigenvalues();  
        Eigen::Matrix3d eigenvectors = es.eigenvectors(); 
        if (eigenvalues(0)/eigenvalues.sum() < 0.05)
        {
            auto end = std::chrono::high_resolution_clock::now();
            total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
            if (eigenvectors.col(0).z() < 0 )
            {
                plane_normal = - eigenvectors.col(0);
            }
            else
            {
                plane_normal = eigenvectors.col(0);
            }
            vector<Eigen::Vector3d> all_points;
            Eigen::Vector3d all_mid_top = ax.toRotationMatrix() * Eigen::Vector3d(footparam.x_upper, 0, 0) + mid;
            Eigen::Vector3d all_mid_down = ax.toRotationMatrix() * Eigen::Vector3d(- footparam.x_button, 0, 0) + mid;
            Eigen::Vector3d all_top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid_top;
            Eigen::Vector3d all_top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + mid_top;
            Eigen::Vector3d all_down_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid_down;
            Eigen::Vector3d all_down_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + mid_down;
            if (SqurePoints(all_top_left.head(2), all_top_right.head(2), all_down_left.head(2), all_down_right.head(2), all_points))
            {
#ifdef DEBUG
                LOG(INFO)<<"all points size "<<all_points.size();
#endif
                for (auto & point : all_points)
                {
                    double dis = (point - center).dot(plane_normal);
                    if (dis < 0.02 && dis > - 0.02)
                    {
                        max_size++;
                    }
                    else if (dis > 0.02)
                    {
                        above_points++;
                    }
                }
#ifdef DEBUG
                LOG(INFO)<<"max_size: "<<max_size<<" above_points: "<<above_points;
#endif
                if (above_points > 8)
                {
                    // 结束计时
                    // auto end = std::chrono::high_resolution_clock::now();
                    // total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
#ifdef DEBUG
                    LOG(INFO)<<"too much above points";
#endif
                    return false;
                }
                if (max_size < 0.5* footsize_inmap)
                {
                    // auto end = std::chrono::high_resolution_clock::now();
                    // total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
                    LOG(INFO)<<"too small size";
                    return false;
                }
                

                Eigen::Vector3d eular;
                computeRollPitch(plane_normal, ankle.z(), eular);
                pitch = eular(1);
                roll = eular(2);
                double d = -center.dot(plane_normal);
                step_height = (-d - ankle.head(2).dot(plane_normal.head(2)))/plane_normal(2);
#ifdef DEBUG
                LOG(INFO)<<"checkXupper: "<<checkXupper<<" checkXButton: "<<checkXButton;
#endif
                // 考虑到实际平面的边角可能存在平面检测与判断是否位于同一平面的差异，所以确定平面编号的矩形比判断共面的矩形区域要小一些
                Eigen::Vector3d mid_top_image = ax.toRotationMatrix() * Eigen::Vector3d(checkXupper - 0.05, 0, 0) + mid;
                Eigen::Vector3d mid_down_image = ax.toRotationMatrix() * Eigen::Vector3d(- (checkXButton - 0.05), 0, 0) + mid;
                Eigen::Vector3d top_left_image = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left - 0.05, 0) + mid_top_image;
                Eigen::Vector3d top_right_image = ax.toRotationMatrix() * Eigen::Vector3d(0, - (footparam.y_right - 0.05), 0) + mid_top_image;
                Eigen::Vector3d down_left_image = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left - 0.05, 0) + mid_down_image;
                Eigen::Vector3d down_right_image = ax.toRotationMatrix() * Eigen::Vector3d(0, - (footparam.y_right - 0.05), 0) + mid_down_image;
#ifdef DEBUG
                LOG(INFO)<<"mid_top_image: "<<mid_top_image.transpose();
                LOG(INFO)<<"mid_down_image: "<<mid_top_image.transpose();
                LOG(INFO)<<"top_left_image: "<<top_left_image.transpose();
                LOG(INFO)<<"top_right_image: "<<top_right_image.transpose();
                LOG(INFO)<<"down_left_image: "<<down_left_image.transpose();
                LOG(INFO)<<"down_right_image: "<<down_right_image.transpose();
#endif
                vector<cv::Point> rectPoints;
                grid_map::Index top_left_index, top_right_index, down_left_index, down_right_index;
                label_localmap.getIndex(top_left_image.head(2), top_left_index);
                label_localmap.getIndex(top_right_image.head(2), top_right_index);
                label_localmap.getIndex(down_right_image.head(2), down_right_index);
                label_localmap.getIndex(down_left_image.head(2), down_left_index);
#ifdef DEBUG
                LOG(INFO)<<"top_left_index: "<<top_left_index.transpose();
                LOG(INFO)<<"top_right_index: "<<top_right_index.transpose();
                LOG(INFO)<<"down_left_index: "<<down_left_index.transpose();
                LOG(INFO)<<"down_right_index: "<<down_right_index.transpose();
#endif
                rectPoints.emplace_back(cv::Point(top_left_index.y(), top_left_index.x()));
                rectPoints.emplace_back(cv::Point(top_right_index.y(), top_right_index.x()));
                rectPoints.emplace_back(cv::Point(down_right_index.y(), down_right_index.x()));
                rectPoints.emplace_back(cv::Point(down_left_index.y(), down_left_index.x()));

                cv::Mat simage = cv::Mat::zeros(label_localmap.getSize().x(), label_localmap.getSize().y(), CV_8UC1);
                const cv::Point* pts = rectPoints.data(); // 获取顶点数组指针
                int numPoints = rectPoints.size();
                cv::polylines(simage, &pts, &numPoints, 1, true, 255, 2);
                cv::fillPoly(simage, std::vector<std::vector<cv::Point>>{rectPoints}, 255);
// #ifdef DEBUG
//                 cv::imshow("simage", simage);
//                 cv::waitKey(0);
//                 LOG(INFO) << "show image: ";
// #endif
                // plane_index = -1;
                // for (int i = 0; i < plane_images.size(); i++)
                // {
                //     cv::Mat intersection;
                //     cv::bitwise_and(plane_images.at(i), simage, intersection);
                //     if (cv::countNonZero(simage & (~intersection)) == 0)
                //     {
                //         plane_index = i;
                //         break;
                //     }
                // }
#ifdef DEBUG
                LOG(INFO)<<"plane index: "<<plane_index;
#endif
                // if (plane_index == -1)
                // {
                //     return false;
                // }
                // 结束计时
                // auto end = std::chrono::high_resolution_clock::now();
                // total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
#ifdef DEBUG
                LOG(INFO)<<"return ";
#endif
                return true;
            }
            else
            {
                return false;
            }
            
        }
        else
        {
            auto end = std::chrono::high_resolution_clock::now();
            total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
#ifdef DEBUG
            LOG(INFO)<<"can not get normal";
#endif
            return false;
        }
    }
    else
    {
        // 我们需要计算的是支撑平面的时间，这里只是判断了四个角点是否位于地图内，你计入
        // auto end = std::chrono::high_resolution_clock::now();
        // total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
#ifdef DEBUG
        LOG(INFO)<<"can not get support area points";
#endif
        return false;
    }
}


AstarHierarchicalFootstepPlannerTraditional::~AstarHierarchicalFootstepPlannerTraditional()
{

}