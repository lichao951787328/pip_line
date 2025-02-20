#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerPropose.h>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <chrono>

// 将脚面的支撑分为4个部分，保证对落脚点支撑的完全覆盖
bool AstarHierarchicalFootstepPlannerPropose::getPointsInFootArea(Eigen::Vector3d ankle, std::unordered_map<int, int> & fore_left_foot_counter, std::unordered_map<int, int> & fore_right_foot_counter, std::unordered_map<int, int> & hind_left_foot_counter, std::unordered_map<int, int> & hind_right_foot_counter)
{
#ifdef DEBUG
    clock_t start = clock();
#endif
    // auto start = std::chrono::high_resolution_clock::now();
    Eigen::AngleAxisd ax(ankle.z(), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d mid(ankle.x(), ankle.y(), 0);

    Eigen::Vector3d fore_top = ax.toRotationMatrix() * Eigen::Vector3d(footparam.x_upper, 0, 0) + mid;
    Eigen::Vector3d fore_button = ax.toRotationMatrix() * Eigen::Vector3d(footparam.x_fore_button, 0, 0) + mid;

    Eigen::Vector3d hind_top = ax.toRotationMatrix() * Eigen::Vector3d(- footparam.x_hind_top, 0, 0) + mid;
    Eigen::Vector3d hind_button = ax.toRotationMatrix() * Eigen::Vector3d(- footparam.x_button, 0, 0) + mid;

    Eigen::Vector3d fore_top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + fore_top;
    Eigen::Vector3d fore_top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + fore_top;
    Eigen::Vector3d fore_button_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + fore_button;
    Eigen::Vector3d fore_button_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + fore_button;

    Eigen::Vector3d hind_top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + hind_top;
    Eigen::Vector3d hind_top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + hind_top;
    Eigen::Vector3d hind_button_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + hind_button;
    Eigen::Vector3d hind_button_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + hind_button;

    grid_map::Position top_left_l(fore_top_left.x(), fore_top_left.y());
    grid_map::Position top_right_l(fore_top_right.x(), fore_top_right.y());
    grid_map::Position down_left_l(hind_button_left.x(), hind_button_left.y());
    grid_map::Position down_right_l(hind_button_right.x(), hind_button_right.y());
    // points_planes.clear();
    // points_planes.resize(planes);
    if (label_localmap.isInside(top_left_l) && label_localmap.isInside(top_right_l) && label_localmap.isInside(down_left_l) && label_localmap.isInside(down_right_l))
    {
        // 注意在这需要使用脚的实际参数计算
#ifdef DEBUG
        clock_t start1 = clock();
#endif
        // getSquareHistogramVoting(fore_top_left.head(2), fore_top_right.head(2), fore_button_left.head(2), fore_button_right.head(2), fore_foot_HV);
        getSquareCounter(fore_top_left.head(2), fore_top.head(2), fore_button_left.head(2), fore_button.head(2), fore_left_foot_counter);
        getSquareCounter(fore_top.head(2), fore_top_right.head(2), fore_button.head(2), fore_button_right.head(2), fore_right_foot_counter);
#ifdef DEBUG
        clock_t end1 = clock();
        LOG(INFO)<<"get point cost time 1: "<<double(end1 - start1) / CLOCKS_PER_SEC * 1000;
        clock_t start2 = clock();
#endif
        // getSquareHistogramVoting(hind_top_left.head(2), hind_top_right.head(2), hind_button_left.head(2), hind_button_right.head(2), hind_foot_HV);
        getSquareCounter(hind_top_left.head(2), hind_top.head(2), hind_button_left.head(2), hind_button.head(2), hind_left_foot_counter);
        getSquareCounter(hind_top.head(2), hind_top_right.head(2), hind_button.head(2), hind_button_right.head(2), hind_right_foot_counter);
#ifdef DEBUG
        clock_t end2 = clock();
        LOG(INFO)<<"get point cost time 2: "<<double(end2 - start2) / CLOCKS_PER_SEC * 1000;
        clock_t end = clock();
        LOG(INFO)<<"get point cost time: "<<double(end - start) / CLOCKS_PER_SEC * 1000;
#endif
        // auto end = std::chrono::high_resolution_clock::now();
        // LOG(INFO)<<"COST: "<<std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        return true;
    }
    else
    {
#ifdef DEBUG
        LOG(ERROR)<<"corner is out of map";
#endif
        // auto end = std::chrono::high_resolution_clock::now();
        // LOG(INFO)<<"COST: "<<std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        return false;
    }
}

bool AstarHierarchicalFootstepPlannerPropose::getSquareCounter(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, std::unordered_map<int, int> & counter)
{
    // 换成使用图像来确定区域
    if(label_localmap.isInside(TL) && label_localmap.isInside(TR) && label_localmap.isInside(BL) && label_localmap.isInside(BR))
    {
        grid_map::Index top_left_index, top_right_index, down_right_index, down_left_index;
        label_localmap.getIndex(TL, top_left_index);
        label_localmap.getIndex(TR, top_right_index);
        label_localmap.getIndex(BL, down_left_index);
        label_localmap.getIndex(BR, down_right_index);
        vector<cv::Point> rectPoints;
        rectPoints.emplace_back(cv::Point(top_left_index.y(), top_left_index.x()));
        rectPoints.emplace_back(cv::Point(top_right_index.y(), top_right_index.x()));
        rectPoints.emplace_back(cv::Point(down_right_index.y(), down_right_index.x()));
        rectPoints.emplace_back(cv::Point(down_left_index.y(), down_left_index.x()));
        cv::Mat simage = cv::Mat::zeros(label_localmap.getSize().x(), label_localmap.getSize().y(), CV_8UC1);
        const cv::Point* pts = rectPoints.data(); // 获取顶点数组指针
        int numPoints = rectPoints.size();
        cv::polylines(simage, &pts, &numPoints, 1, true, 255, 2);
        cv::fillPoly(simage, std::vector<std::vector<cv::Point>>{rectPoints}, 255);
        std::vector<cv::Point> whitePixels;
        cv::findNonZero(simage, whitePixels);
        for (auto & p : whitePixels)
        {
            grid_map::Index index(p.y, p.x);
            grid_map::Position3 position;
            if (label_localmap.getPosition3("label", index, position))
            {
                int label_index = static_cast<int>(label_localmap["label"](index.x(), index.y()));
                // HV.add(label_index, position);
                counter[label_index]++;
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool AstarHierarchicalFootstepPlannerPropose::getSupportPlaneIndex(std::unordered_map<int, int> & counter, Eigen::Vector3d & mid, int thred, int & plane_index, int & support_size)
{
    double max_height = -std::numeric_limits<double>::infinity();
    int support_plane = -1;
    int max_size = 0;
    for (auto & bin : counter)
    {
        if (bin.second > thred)
        {
            double temp_height = planes_info.at(bin.first).getZ(mid.head(2));
            if (max_height < temp_height)
            {
                max_height = temp_height;
                support_plane = bin.first;
                max_size = bin.second;
            }
        }
    }
    if (support_plane == -1)
    {
        return false;
    }
    else
    {
        plane_index = support_plane;
        support_size = max_size;
        return true;
    }
}

// tested 粗略检查
bool AstarHierarchicalFootstepPlannerPropose::computeLandInfo(Eigen::Vector3d ankle, int & max_size, int & above_points, Eigen::Vector3d & plane_normal, double & step_height, double & pitch, double & roll) 
{
    // 能否找到支撑平面
    // 前脚直方图
    // 后脚直方图
    // 匹配对
    // 支撑平面

    // 论文中这个函数对应的可通行性检测，要记录这个函数被调用的次数及总时间消耗
#ifdef COUNT_TIME
    checktime++;
    auto start = std::chrono::high_resolution_clock::now();
#endif
    max_size = 0;
    above_points = 0;
    plane_normal = Eigen::Vector3d::Zero();
    step_height = -std::numeric_limits<double>::infinity();
    pitch = std::numeric_limits<double>::infinity();
    roll = std::numeric_limits<double>::infinity();
    // HistogramVoting fore_foot_HV, hind_foot_HV;
    std::unordered_map<int, int> fore_left_foot_counter, fore_right_foot_counter, hind_left_foot_counter, hind_right_foot_counter;
    Eigen::Vector3d mid(ankle.x(), ankle.y(), 0);
    Eigen::AngleAxisd ad(ankle.z(), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d fore_mid = mid + ad.toRotationMatrix() * Eigen::Vector3d((footparam.x_upper + footparam.x_fore_button)/2.0, 0, 0);
    Eigen::Vector3d fore_left_mid = fore_mid + ad.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left/2, 0);
    Eigen::Vector3d fore_right_mid = fore_mid + ad.toRotationMatrix() * Eigen::Vector3d(0, -footparam.y_right/2, 0);
    Eigen::Vector3d hind_mid = mid + ad.toRotationMatrix() * Eigen::Vector3d(-(footparam.x_button + footparam.x_hind_top)/2.0, 0, 0);
    Eigen::Vector3d hind_left_mid = hind_mid + ad.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left/2, 0);
    Eigen::Vector3d hind_right_mid = hind_mid + ad.toRotationMatrix() * Eigen::Vector3d(0, -footparam.y_right/2, 0);
    if (getPointsInFootArea(ankle, fore_left_foot_counter, fore_right_foot_counter, hind_left_foot_counter, hind_right_foot_counter))
    {
#ifdef DEBUG
        LOG(INFO)<<ankle.transpose();
        for (auto & bin1 : fore_left_foot_counter)
        {
            LOG(INFO)<<bin1.first<<" "<<bin1.second;
        }
        for (auto & bin2 : fore_right_foot_counter)
        {
            LOG(INFO)<<bin2.first<<" "<<bin2.second;
        }
        for (auto & bin3 : hind_left_foot_counter)
        {
            LOG(INFO)<<bin3.first<<" "<<bin3.second;
        }
        for (auto & bin4 : hind_right_foot_counter)
        {
            LOG(INFO)<<bin4.first<<" "<<bin4.second;
        }
#endif
        int fore_left_support_plane = -1;
        int fore_right_support_plane = -1;
        int hind_left_support_plane = -1;
        int hind_right_support_plane = -1;
        int fore_left_support_size = 0;
        int fore_right_support_size = 0;
        int hind_left_support_size = 0;
        int hind_right_support_size = 0;
        double x_num = (std::min(footparam.y_left, footparam.y_right)/2.0);
        int thred = (std::floor(x_num/resolution)) * (std::ceil(0.02/resolution));

        if (!getSupportPlaneIndex(fore_left_foot_counter, fore_left_mid, thred, fore_left_support_plane, fore_left_support_size))
        {
#ifdef  DEBUG
            LOG(INFO)<<"can not get fore support plane";
#endif
#ifdef COUNT_TIME
            auto end = std::chrono::high_resolution_clock::now();
            total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
#endif
            return false;
        }

        if (!getSupportPlaneIndex(fore_right_foot_counter, fore_right_mid, thred, fore_right_support_plane, fore_right_support_size))
        {
#ifdef  DEBUG
            LOG(INFO)<<"can not get fore support plane";
#endif
#ifdef COUNT_TIME
            auto end = std::chrono::high_resolution_clock::now();
            total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
#endif
            return false;
        }

        if (!getSupportPlaneIndex(hind_left_foot_counter, hind_left_mid, thred, hind_left_support_plane, hind_left_support_size))
        {
#ifdef  DEBUG
            LOG(INFO)<<"can not get fore support plane";
#endif
#ifdef COUNT_TIME
            auto end = std::chrono::high_resolution_clock::now();
            total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
#endif
            return false;
        }

        if (!getSupportPlaneIndex(hind_right_foot_counter, hind_right_mid, thred, hind_right_support_plane, hind_right_support_size))
        {
#ifdef  DEBUG
            LOG(INFO)<<"can not get fore support plane";
#endif
#ifdef COUNT_TIME
            auto end = std::chrono::high_resolution_clock::now();
            total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
#endif
            return false;
        }

        if (fore_left_support_plane == fore_right_support_plane && hind_left_support_plane == hind_right_support_plane  && fore_right_support_plane == hind_left_support_plane)
        {
#ifdef COUNT_TIME
            auto end = std::chrono::high_resolution_clock::now();
            total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
#endif
            int plane_index = fore_left_support_plane;
            max_size = fore_left_support_size + fore_right_support_size + hind_left_support_size + hind_right_support_size;
            plane_normal = Eigen::Vector3d(planes_info.at(plane_index).normal.x(), planes_info.at(plane_index).normal.y(), planes_info.at(plane_index).normal.z());
            Eigen::Vector3d center = Eigen::Vector3d(planes_info.at(plane_index).center.x(), planes_info.at(plane_index).center.y(), planes_info.at(plane_index).center.z());
            double max_height = planes_info.at(plane_index).getZ(ankle.head(2));
#ifdef DEBUG
            LOG(INFO)<<"plane_normal: "<<plane_normal.transpose();
            LOG(INFO)<<"center: "<<center.transpose();
#endif
            step_height = max_height;
            Eigen::Vector3d eular;
            computeRollPitch(plane_normal, ankle.z(), eular);
            pitch = eular(1);
            roll = eular(2);
            vector<Eigen::Vector3d> allpoints;
            if (getLandAreaPoints(ankle, allpoints))
            {
                for (auto & point : allpoints)
                {
                    if ((point - center).dot(plane_normal) > 0.02)
                    {
#ifdef DEBUG
                        LOG(INFO)<<"into foot"<<(point - center).dot(plane_normal);
#endif
                        above_points ++;
                    }
                }
                if (above_points > 0)
                {
#ifdef DEBUG
                    LOG(INFO)<<"too much above points"<<above_points;
#endif
                    return false;
                }   
                else
                {
                    return true;
                }
            }
            else
            {
    #ifdef DEBUG
                LOG(INFO)<<"can not get all points";
    #endif
    // LOG(INFO)<<"can not get all points";
                // auto end = std::chrono::high_resolution_clock::now();
                // total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
                return false;
            }

        }
        else
        {
    #ifdef DEBUG
            LOG(ERROR)<<"can not get area";
    #endif
    // LOG(ERROR)<<"can not get area";
#ifdef COUNT_TIME
            auto end = std::chrono::high_resolution_clock::now();
            total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
#endif
            return false;
        }
    }
    else
    {
        return false;
    }
    
}

AstarHierarchicalFootstepPlannerPropose::~AstarHierarchicalFootstepPlannerPropose()
{

}