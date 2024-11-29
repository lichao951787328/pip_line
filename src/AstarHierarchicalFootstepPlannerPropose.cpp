#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerPropose.h>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <chrono>

bool AstarHierarchicalFootstepPlannerPropose::isStartFeasible(Eigen::Vector3d start, Eigen::Vector3d & left_foot, Eigen::Vector3d & right_foot) 
{
    if (label_localmap.isInside(start.head(2)))
    {
        // 保证四个点都在地图内
        Eigen::AngleAxisd ad(start.z(), Eigen::Vector3d::UnitZ());
        Eigen::Vector3d mid = Eigen::Vector3d::Zero();
        mid.head(2) = start.head(2);
        Eigen::Vector3d left_offset(0, footparam.y_left, 0);
        Eigen::Vector3d right_offset(0, -footparam.y_right, 0);
        Eigen::Vector3d left_mid = ad.toRotationMatrix() * left_offset + mid;
        Eigen::Vector3d right_mid = ad.toRotationMatrix() * right_offset + mid;
        Eigen::Vector3d up_offset(footparam.x_upper, 0, 0);
        Eigen::Vector3d up_left = ad.toRotationMatrix() * up_offset + left_mid;
        Eigen::Vector3d up_right = ad.toRotationMatrix() * up_offset + right_mid;
        Eigen::Vector3d button_offset(-footparam.x_button, 0, 0);
        Eigen::Vector3d button_left = ad.toRotationMatrix() * button_offset + left_mid;
        Eigen::Vector3d button_right = ad.toRotationMatrix() * button_offset + right_mid;
        if (label_localmap.isInside(left_mid.head(2)) && label_localmap.isInside(right_mid.head(2)) && label_localmap.isInside(up_left.head(2)) && label_localmap.isInside(up_right.head(2)))
        {
            // 计算左右脚的位置，并返回合适的左右落脚点
            Eigen::Vector3d half_hip_width = Eigen::Vector3d(0, hip_width/2, 0);
            Eigen::Vector3d left_foot_tmp = ad.toRotationMatrix() * half_hip_width + mid;
            Eigen::Vector3d right_foot_tmp = ad.toRotationMatrix() * -half_hip_width + mid;
            double height_left,  height_right;
            int left_support_index = -1;
            int right_support_index = -1;
            double left_roll, right_roll, left_pitch, right_pitch;
            if (getPointInfoInPlane(left_foot_tmp, height_left, left_support_index, left_pitch, left_roll) && getPointInfoInPlane(right_foot_tmp, height_right, right_support_index, right_pitch, right_roll))
            {
                left_foot = left_foot_tmp;
                right_foot = right_foot_tmp;
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
        // 保证传统方法下左右脚都共面
    }
    else
    {
        return false;
    }
}

bool AstarHierarchicalFootstepPlannerPropose::getPointsInFootArea(Eigen::Vector3d ankle, HistogramVoting & fore_foot_HV, HistogramVoting & hind_foot_HV)
{
#ifdef DEBUG
    clock_t start = clock();
#endif
    // auto start = std::chrono::high_resolution_clock::now();
    Eigen::AngleAxisd ax(ankle.z(), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d mid(ankle.x(), ankle.y(), 0);

    Eigen::Vector3d fore_top = ax.toRotationMatrix() * Eigen::Vector3d(footparam.x_upper, 0, 0) + mid;
    Eigen::Vector3d fore_button = ax.toRotationMatrix() * Eigen::Vector3d(- footparam.x_fore_button, 0, 0) + mid;

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


    // Eigen::Vector3d mid_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid;
    // Eigen::Vector3d mid_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + mid;

    // Eigen::Vector3d top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid_top;
    // Eigen::Vector3d top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + mid_top;

    // Eigen::Vector3d down_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid_down;
    // Eigen::Vector3d down_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + mid_down;

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
        getSquareHistogramVoting(fore_top_left.head(2), fore_top_right.head(2), fore_button_left.head(2), fore_button_right.head(2), fore_foot_HV);
#ifdef DEBUG
        clock_t end1 = clock();
        LOG(INFO)<<"get point cost time 1: "<<double(end1 - start1) / CLOCKS_PER_SEC * 1000;
        clock_t start2 = clock();
#endif
        getSquareHistogramVoting(hind_top_left.head(2), hind_top_right.head(2), hind_button_left.head(2), hind_button_right.head(2), hind_foot_HV);
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

void AstarHierarchicalFootstepPlannerPropose::getSquareHistogramVoting(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, HistogramVoting & HV)
{
    grid_map::LineIterator iterator_start(label_localmap, BR, BL);
    grid_map::LineIterator iterator_end(label_localmap, TR, TL);
    for (; !iterator_start.isPastEnd()&&!iterator_end.isPastEnd(); ++iterator_start, ++iterator_end)
    {
        grid_map::Index start_index(*iterator_start);
        grid_map::Index end_index(*iterator_end);
        for (grid_map::LineIterator iterator_l(label_localmap, start_index, end_index); !iterator_l.isPastEnd(); ++iterator_l)
        {
            const grid_map::Index index_l(*iterator_l);
            grid_map::Position3 cor_position;
            if (label_localmap.getPosition3("elevation", index_l, cor_position))
            {
                if (!std::isnan(cor_position.z()))
                {
#ifdef DEBUG
                    // LOG(INFO)<<label_localmap["label"](index_l.x(), index_l.y());
#endif
                    if (!std::isnan(label_localmap["label"](index_l.x(), index_l.y())))
                    {
                        int label_index = static_cast<int>(label_localmap["label"](index_l.x(), index_l.y()));
                        HV.add(label_index, cor_position);
                    }
                    else
                    {
                        HV.addNANPoints();// 编号是nan
#ifdef DEBUG
                        LOG(INFO)<<"nan";
#endif
                    }
                }
                else
                {
                    HV.addNANPoints(); // 点是nan
#ifdef DEBUG
                    LOG(INFO)<<"cor nan";
#endif
                }
            }
            else
            {
                HV.addNANPoints(); // 不能得到此栅格处的点
#ifdef DEBUG
                LOG(INFO)<<"can not get cor";
#endif
            }

        }
    }
}


bool AstarHierarchicalFootstepPlannerPropose::nodeExtension(FootstepNodePtr current_node, FootstepNodePtr pre_node, vector<FootstepNodePtr> & child_nodes) 
{
    // 基础节点，在地图坐标系下的节点
    child_nodes.clear();
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> special_transitions = basicTransitions(current_node);

#ifdef DEBUG
    LOG(INFO)<<"node size: "<<special_transitions.size();
    LOG(INFO)<<plane_image.size();
    cv::imshow("plane_image", plane_image);
    cv::waitKey(0);
    for (auto & node : special_transitions)
    {
        grid_map::Index index;
        if (localmap.getIndex(node.second.head(2), index))
        {
            plane_image.at<cv::Vec3b>(index.x(), index.y()) = cv::Vec3b(0, 0, 255);
        }
    }

    // 原始图像尺寸
    int originalWidth = plane_image.cols;
    int originalHeight = plane_image.rows;

    // 放大倍数
    float scaleFactor = 2.0;  // 放大为原来的两倍

    // 计算放大后的尺寸
    int scaledWidth = static_cast<int>(originalWidth * scaleFactor);
    int scaledHeight = static_cast<int>(originalHeight * scaleFactor);

    // 创建放大后的图像
    cv::Mat scaledImg;
    resize(plane_image, scaledImg, cv::Size(scaledWidth, scaledHeight));  // 调整图像大小

    cv::imshow("nodeextension", scaledImg);
    cv::waitKey(0);
#endif
    // 将这些点投影到localmap上，去掉不符合的点，并对每个点进行打分

    // 基础节点
    std::priority_queue<ScoreMarkerNodePtr, std::vector<ScoreMarkerNodePtr>, ScoreMarkerNodeCompare> basicScoreNodes;
    // std::priority_queue<ScoreMarkerNodePtr, std::vector<ScoreMarkerNodePtr>, ScoreMarkerNodeCompare> dangerousBasicScoreNodes;
    for (auto & transition : special_transitions)
    {
        double score;
        bool dangerous = false;
        double height = 0.;
        Eigen::Vector3d normal;
        int plane_index;
        double pitch, roll;
        if (computeTransitionScore(transition, current_node, pre_node, dangerous, score, height, normal, plane_index, pitch, roll))
        {
            if (dangerous)// 这个节点需要微调
            {
                // transition.first 基础偏移量
                // ScoreMarkerNodePtr node_p = std::make_shared<ScoreMarkerNode>(transition.first, score);
                // dangerousBasicScoreNodes.push(node_p);

                // 如果是危险节点，那么把这个节点进行微调，如果合理再加入basicScoreNodes中
                vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> trans = fineTransitions(transition.first, current_node);
                std::priority_queue<ScoreMarkerNodePtr, std::vector<ScoreMarkerNodePtr>, ScoreMarkerNodeCompare> tmpFineScoreNodes;
                for (auto & tr : trans)
                {
                    double tmp_score;
                    double height = 0;
                    Eigen::Vector3d fine_normal;
                    int plane_index;
                    double pitch, roll;
                    bool fine_dangerous = false;
                    if (computeTransitionScore(tr, current_node, pre_node, fine_dangerous, tmp_score, height, fine_normal, plane_index, pitch, roll))
                    {
                        if (!fine_dangerous)
                        {
                            ScoreMarkerNodePtr node_p = std::make_shared<ScoreMarkerNode>(tr.second, tmp_score, height, fine_normal, plane_index, roll, pitch);
                            tmpFineScoreNodes.push(node_p);
                        }
                    }
                }

                int total = 2;
                while (total >= 0 && !tmpFineScoreNodes.empty())
                {
                    auto node_tmp = tmpFineScoreNodes.top();
                    tmpFineScoreNodes.pop();
                    basicScoreNodes.push(node_tmp);
                }
            }
            else// 不需要微调
            {
                // transition.first 基础偏移量 transition.second 地图中实际的偏移
                ScoreMarkerNodePtr node_p = std::make_shared<ScoreMarkerNode>(transition.second, score, height, normal, plane_index, roll, pitch);
                // computeTransitionStrictScore 是不是有点多余
                basicScoreNodes.push(node_p);
            }
        }
    }

     if (basicScoreNodes.empty())
    {
        return false;
    }
    
#ifdef DEBUG
    cv::Mat basicImage = cv::Mat::zeros(plane_image.size(), CV_8UC3);

    auto tmpbasicScoreNodes = basicScoreNodes;
    while (tmpbasicScoreNodes.empty())
    {
        auto basicNode = tmpbasicScoreNodes.top();
        tmpbasicScoreNodes.pop();
        grid_map::Index index;
        if (localmap.getIndex(basicNode->point.head(2), index))
        {
            plane_image.at<cv::Vec3b>(index.x(), index.y()) = cv::Vec3b(0, 0, 255);
        }
    }
    cv::Mat enlarge;
    resize(plane_image, enlarge, cv::Size(scaledWidth, scaledHeight));
    cv::imshow("enlarge", enlarge);
    cv::waitKey(0);
#endif

    // traverserbility check，使用两个圆柱体进行检查，上半身圆柱体和膝盖圆柱体
    std::priority_queue<ScoreMarkerNodePtr, std::vector<ScoreMarkerNodePtr>, ScoreMarkerNodeCompare> passableNodes;
    while (!basicScoreNodes.empty())
    {
        auto node = basicScoreNodes.top();
        basicScoreNodes.pop();
        if (traversibilityCheck(node))
        {
            passableNodes.push(node);
        }
    }
    
    while (!passableNodes.empty())
    {
        auto node = passableNodes.top();
        // 由基础偏移量转到实际位置
        passableNodes.pop();
        FootstepNodePtr stepnode = std::make_shared<FootstepNode>(node->point, node->height, node->roll, node->pitch, current_node->footstep.getInverseRobotSide());
        stepnode->plane_index = node->plane_index;
        stepnode->PreFootstepNode = current_node;
        // 跨平面运动时对规划的落脚点限制，如果不在一个平面上，就保证两不的距离不超过0.35
        if (stepnode->plane_index != stepnode->PreFootstepNode->plane_index)
        {
            Eigen::Vector2d dis_v1(stepnode->footstep.x, stepnode->footstep.y);
            Eigen::Vector2d dis_v2(stepnode->PreFootstepNode->PreFootstepNode->footstep.x, stepnode->PreFootstepNode->PreFootstepNode->footstep.y);
            // 距离超过0.4
            if (abs((dis_v1 - dis_v2).norm()) > 0.35)
            {
                continue;
            }
            // 不在同一平面上且角度相差太大，也舍弃
            if (abs(stepnode->footstep.yaw - stepnode->PreFootstepNode->footstep.yaw) > 3 /57.3)
            {
                continue;
            }
        }


        // 如果两脚pitch较大，那就不要迈大步长，5关节可能会达到62度
        // 如果此时机器人行走在斜面上，那就不要迈大步长，因为5关节可能会超限
        if (stepnode->footstep.pitch <= - 5/57.3 && stepnode->PreFootstepNode->footstep.pitch <= - 5/57.3)
        {
            Eigen::Vector2d dis_v1(stepnode->footstep.x, stepnode->footstep.y);
            Eigen::Vector2d dis_v2(stepnode->PreFootstepNode->PreFootstepNode->footstep.x, stepnode->PreFootstepNode->PreFootstepNode->footstep.y);
            if (abs((dis_v1 - dis_v2).norm()) > 0.33)
            {
                continue;
            }
        }

        if (!computeHcost(stepnode, stepnode->Hcost))
        {
            continue;
        }

        if (!computeGcost(stepnode, stepnode->Gcost))
        {
            continue;
        }
        stepnode->cost = stepnode->Hcost + stepnode->Gcost;
        child_nodes.emplace_back(stepnode);
    }
    
    if (child_nodes.empty())
    {
        return false;
    }
    else
    {
        return true;
    }
}

// tested 粗略检查
bool AstarHierarchicalFootstepPlannerPropose::computeLandInfo(Eigen::Vector3d ankle, int & max_size, int & above_points, Eigen::Vector3d & plane_normal, double & step_height, int & plane_index, double & pitch, double & roll) 
{
    // 能否找到支撑平面
    // 前脚直方图
    // 后脚直方图
    // 匹配对
    // 支撑平面

    // 论文中这个函数对应的可通行性检测，要记录这个函数被调用的次数及总时间消耗
    checktime++;
    auto start = std::chrono::high_resolution_clock::now();
    max_size = 0;
    above_points = 0;
    plane_normal = Eigen::Vector3d::Zero();
    step_height = -std::numeric_limits<double>::infinity();
    plane_index = -1;
    pitch = std::numeric_limits<double>::infinity();
    roll = std::numeric_limits<double>::infinity();
    HistogramVoting fore_foot_HV, hind_foot_HV;
    Eigen::Vector3d mid(ankle.x(), ankle.y(), 0);
    Eigen::AngleAxisd ad(ankle.z(), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d fore_mid = mid + ad.toRotationMatrix() * Eigen::Vector3d((footparam.x_upper + footparam.x_fore_button)/2.0, 0, 0);
    Eigen::Vector3d hind_mid = mid + ad.toRotationMatrix() * Eigen::Vector3d(-(footparam.x_button + footparam.x_hind_top)/2.0, 0, 0);
    if (getPointsInFootArea(ankle, fore_foot_HV, hind_foot_HV))
    {
#ifdef DEBUG
    LOG(INFO)<<ankle.transpose();
    for (auto & bin1 : fore_foot_HV.counter)
    {
        LOG(INFO)<<bin1.first<<" "<<bin1.second;
    }
    for (auto & bin2 : hind_foot_HV.counter)
    {
        LOG(INFO)<<bin2.first<<" "<<bin2.second;
    }
    
#endif

        vector<std::pair<int, int>> candidate_support_planes;
        int thred = 0.25 * footsize_inmap;
        double max_height = -std::numeric_limits<double>::infinity();
        int fore_support_plane = -1;
        int fore_max_size = 0;
        for (auto & bin1 : fore_foot_HV.counter)
        {
            if (bin1.second > thred)
            {
                double temp_height = planes_info.at(bin1.first).getZ(fore_mid.head(2));
                if (max_height < temp_height)
                {
                    max_height = temp_height;
                    fore_support_plane = bin1.first;
                    fore_max_size = bin1.second;
                }
            }
        }
        // LOG(INFO)<<fore_support_plane<<" "<<fore_max_size;
        if (fore_support_plane == -1)
        {
#ifdef  DEBUG
            LOG(INFO)<<"can not get fore support plane";
#endif
            // clock_t end = clock();
            // total_time += double(end - start) / CLOCKS_PER_SEC * 1000;
            auto end = std::chrono::high_resolution_clock::now();
            total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
            return false;
        }
        

        max_height = -std::numeric_limits<double>::infinity();
        int hind_support_plane = -1;
        int hind_max_size = 0;
#ifdef DEBUG
        LOG(INFO)<<"thred: "<<thred;
#endif
        for (auto & bin1 : hind_foot_HV.counter)
        {
#ifdef DEBUG
            LOG(INFO)<<bin1.first<<" "<<bin1.second;
#endif
            if (bin1.second > thred)
            {
                double temp_height = planes_info.at(bin1.first).getZ(hind_mid.head(2));
                if (max_height < temp_height)
                {
                    max_height = temp_height;
                    hind_support_plane = bin1.first;
                    hind_max_size = bin1.second;
                }
            }
        }
        // LOG(INFO)<<hind_support_plane<<" "<<hind_max_size;

        if (hind_support_plane == -1)
        {
#ifdef  DEBUG
            LOG(INFO)<<"can not get hind support plane";
#endif
            auto end = std::chrono::high_resolution_clock::now();
            total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
            return false;
        }

        if (hind_support_plane == fore_support_plane)
        {
            plane_index = hind_support_plane;
            max_size = hind_max_size + fore_max_size;
            plane_normal = Eigen::Vector3d(planes_info.at(plane_index).normal.x(), planes_info.at(plane_index).normal.y(), planes_info.at(plane_index).normal.z());
            Eigen::Vector3d center = Eigen::Vector3d(planes_info.at(plane_index).center.x(), planes_info.at(plane_index).center.y(), planes_info.at(plane_index).center.z());
            max_height = planes_info.at(plane_index).getZ(ankle.head(2));
#ifdef DEBUG
            LOG(INFO)<<"plane_normal: "<<plane_normal.transpose();
            LOG(INFO)<<"center: "<<center.transpose();
#endif
            step_height = max_height;
            Eigen::Vector3d eular;
            computeRollPitch(plane_normal, ankle.z(), eular);
            pitch = eular(1);
            roll = eular(2);
            // LOG(INFO)<<"GET ROLL PITCH";
            // 获取所有点的
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
                if (above_points > 8)
                {
#ifdef DEBUG
                    LOG(INFO)<<"too much above points"<<above_points;
#endif
                    auto end = std::chrono::high_resolution_clock::now();
                    total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
                    return false;
                }   
                else
                {
                    auto end = std::chrono::high_resolution_clock::now();
                    total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
                    return true;
                }
            }
            else
            {
#ifdef DEBUG
                LOG(INFO)<<"can not get all points";
#endif
                auto end = std::chrono::high_resolution_clock::now();
                total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
                return false;
            }
            
            
        }
        {
#ifdef DEBUG
            LOG(INFO)<<"can not get support plane";
#endif
            auto end = std::chrono::high_resolution_clock::now();
            total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
            return false;
        }
    }
    else
    {
#ifdef DEBUG
        LOG(ERROR)<<"can not get area";
#endif
        auto end = std::chrono::high_resolution_clock::now();
        total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
        return false;
    }
}

bool AstarHierarchicalFootstepPlannerPropose::computerLeftRightGoal(Eigen::Vector3d goal) 
{
    // LOG(INFO)<<"function: computerLeftRightGoal";
    Eigen::Vector3d left_offset(0, hip_width/2.0, 0);
    Eigen::Vector3d right_offset(0, -hip_width/2.0, 0);
    Eigen::AngleAxisd ad(goal.z(), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d left_goal, right_goal;
    left_goal.head(2) = (ad.toRotationMatrix() * left_offset + Eigen::Vector3d(goal.x(), goal.y(), 0)).head(2);
    left_goal.z() = goal.z();
    right_goal.head(2) = (ad.toRotationMatrix() * right_offset + Eigen::Vector3d(goal.x(), goal.y(), 0)).head(2);
    right_goal.z() = goal.z();
    LOG(INFO)<<"left_goal: "<<left_goal.transpose();
    LOG(INFO)<<"right_goal: "<<right_goal.transpose();
#ifdef DEBUG
    LOG(INFO)<<"left_goal: "<<left_goal.transpose();
    LOG(INFO)<<"right_goal: "<<right_goal.transpose();
#endif
    if (localmap.isInside(left_goal.head(2)) && localmap.isInside(right_goal.head(2)))
    {
        // 还需要判断此状态下机器人能不能落脚
        vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> left_cands = fineLandPoint(left_goal);
        vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> right_cands = fineLandPoint(right_goal);
        double score = - std::numeric_limits<double>::infinity();
        double opt_height, opt_pitch, opt_roll;
        int opt_plane_index = -1;
        Eigen::Vector3d left_opt;
        for (auto & cand : left_cands)
        {
            double tmpscore;
            int plane_index = -1;
            double height = -std::numeric_limits<double>::infinity();
            double pitch = std::numeric_limits<double>::infinity();
            double roll = std::numeric_limits<double>::infinity();
            if (computeLandPointScore(cand, tmpscore, height, plane_index, pitch, roll))
            {
                // LOG(INFO)<<"tmpscore: "<<tmpscore;
                if (tmpscore > score)
                {
                    left_opt = cand.second;
                    // LOG(INFO)<<left_opt.transpose();
                    score = tmpscore;
                    opt_height = height;
                    opt_pitch = pitch;
                    opt_roll = roll; 
                    opt_plane_index = plane_index;
                }
            }
        }
        if (opt_plane_index != -1)
        {
            end_left_p = std::make_shared<FootstepNode>(left_opt, opt_height, opt_roll, opt_pitch, 0);
            end_left_p->plane_index = opt_plane_index;
        }
        else
        {
            return false;
        }
        
        opt_plane_index = -1;
        score = - std::numeric_limits<double>::infinity();
        Eigen::Vector3d right_opt;
        for (auto & cand : right_cands)
        {
            double tmpscore;
            int plane_index = -1;
            double height = -std::numeric_limits<double>::infinity();
            double pitch = std::numeric_limits<double>::infinity();
            double roll = std::numeric_limits<double>::infinity();
            if (computeLandPointScore(cand, tmpscore, height, plane_index, pitch, roll))
            {
                // LOG(INFO)<<"tmpscore: "<<tmpscore;
                if (tmpscore > score)
                {
                    right_opt = cand.second;
                    // LOG(INFO)<<left_opt.transpose();
                    score = tmpscore;
                    opt_height = height;
                    opt_pitch = pitch;
                    opt_roll = roll; 
                    opt_plane_index = plane_index;
                }
            }
        }
        if (opt_plane_index != -1)
        {
            end_right_p = std::make_shared<FootstepNode>(right_opt, opt_height, opt_roll, opt_pitch, 1);
            end_right_p->plane_index = opt_plane_index;
        }
        else
        {
            return false;
        }

#ifdef DEBUG
        LOG(INFO)<<"left_opt: "<<left_opt.transpose();
        LOG(INFO)<<"right_opt: "<<right_opt.transpose();
#endif
        return true;
        // LOG(INFO)<<"left_opt: "<<left_opt.transpose();
        // LOG(INFO)<<"right_opt: "<<right_opt.transpose();
        // end_left_p = std::make_shared<FootstepNode>(left_opt, 0);
        // end_right_p = std::make_shared<FootstepNode>(right_opt, 1);
        // double z_left, z_right;
        // Eigen::Vector3d eular_left, eular_right;
        // int left_index, right_index;
        // if (computeZRollPitch(left_opt, z_left, eular_left, left_index) && computeZRollPitch(right_opt, z_right, eular_right, right_index))
        // {
        //     end_left_p->footstep.z = z_left;
        //     end_left_p->footstep.roll = eular_left(2);
        //     end_left_p->footstep.pitch = eular_left(1);
        //     end_left_p->footstep.yaw = eular_left(0);
        //     end_left_p->plane_index = left_index;
        //     end_right_p->footstep.z = z_right;
        //     end_right_p->footstep.roll = eular_right(2);
        //     end_right_p->footstep.pitch = eular_right(1);
        //     end_right_p->footstep.yaw = eular_right(0);
        //     end_right_p->plane_index = right_index;
        //     cout<<"goal: left foot "<<end_left_p->footstep.x<<" "<<end_left_p->footstep.y<<" "<<end_left_p->footstep.z<<" "<<end_left_p->footstep.roll<<" "<<end_left_p->footstep.pitch<<" "<<end_left_p->footstep.yaw<<endl;
        //     cout<<"goal: right foot "<<end_right_p->footstep.x<<" "<<end_right_p->footstep.y<<" "<<end_right_p->footstep.z<<" "<<end_right_p->footstep.roll<<" "<<end_right_p->footstep.pitch<<" "<<end_right_p->footstep.yaw<<endl;
        //     return true;
        // }
        // else
        // {
        //     LOG(INFO)<<"foot param is not suitable";
        //     end_left_p = nullptr;
        //     end_right_p = nullptr;
        //     return false;
        // }
    }
    else
    {
        LOG(INFO)<<"left foot or right foot not in map";
        end_left_p = nullptr;
        end_right_p = nullptr;
        return false;
    }
}

bool AstarHierarchicalFootstepPlannerPropose::checkFeasibleGoal(Eigen::Vector3d goal) 
{
    Eigen::Vector3d left_offset(0, hip_width/2.0, 0);
    Eigen::Vector3d right_offset(0, -hip_width/2.0, 0);
    Eigen::AngleAxisd ad(goal.z(), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d left_goal, right_goal;
    left_goal.head(2) = (ad.toRotationMatrix() * left_offset + Eigen::Vector3d(goal.x(), goal.y(), 0)).head(2);
    left_goal.z() = goal.z();
    right_goal.head(2) = (ad.toRotationMatrix() * right_offset + Eigen::Vector3d(goal.x(), goal.y(), 0)).head(2);
    right_goal.z() = goal.z();
    LOG(INFO)<<"left_goal: "<<left_goal.transpose();
    LOG(INFO)<<"right_goal: "<<right_goal.transpose();
#ifdef DEBUG
    LOG(INFO)<<"left_goal: "<<left_goal.transpose();
    LOG(INFO)<<"right_goal: "<<right_goal.transpose();
#endif
    if (localmap.isInside(left_goal.head(2)) && localmap.isInside(right_goal.head(2)))
    {
        // 还需要判断此状态下机器人能不能落脚
        vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> left_cands = fineLandPoint(left_goal);
        vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> right_cands = fineLandPoint(right_goal);
        double score = - std::numeric_limits<double>::infinity();
        double opt_height, opt_pitch, opt_roll;
        int opt_plane_index = -1;
        Eigen::Vector3d left_opt;
        for (auto & cand : left_cands)
        {
            double tmpscore;
            int plane_index = -1;
            double height = -std::numeric_limits<double>::infinity();
            double pitch = std::numeric_limits<double>::infinity();
            double roll = std::numeric_limits<double>::infinity();
            if (computeLandPointScore(cand, tmpscore, height, plane_index, pitch, roll))
            {
                // LOG(INFO)<<"tmpscore: "<<tmpscore;
                if (tmpscore > score)
                {
                    left_opt = cand.second;
                    // LOG(INFO)<<left_opt.transpose();
                    score = tmpscore;
                    opt_height = height;
                    opt_pitch = pitch;
                    opt_roll = roll; 
                    opt_plane_index = plane_index;
                }
            }
        }
        if (opt_plane_index == -1)
        {
            return false;
        }
        
        opt_plane_index = -1;
        score = - std::numeric_limits<double>::infinity();
        Eigen::Vector3d right_opt;
        for (auto & cand : right_cands)
        {
            double tmpscore;
            int plane_index = -1;
            double height = -std::numeric_limits<double>::infinity();
            double pitch = std::numeric_limits<double>::infinity();
            double roll = std::numeric_limits<double>::infinity();
            if (computeLandPointScore(cand, tmpscore, height, plane_index, pitch, roll))
            {
                // LOG(INFO)<<"tmpscore: "<<tmpscore;
                if (tmpscore > score)
                {
                    right_opt = cand.second;
                    // LOG(INFO)<<left_opt.transpose();
                    score = tmpscore;
                    opt_height = height;
                    opt_pitch = pitch;
                    opt_roll = roll; 
                    opt_plane_index = plane_index;
                }
            }
        }
        if (opt_plane_index == -1)
        {
            return false;
        }
#ifdef DEBUG
        LOG(INFO)<<"left_opt: "<<left_opt.transpose();
        LOG(INFO)<<"right_opt: "<<right_opt.transpose();
#endif
        return true;
    }
    else
    {
        LOG(INFO)<<"left foot or right foot not in map";
        return false;
    }
}

AstarHierarchicalFootstepPlannerPropose::~AstarHierarchicalFootstepPlannerPropose()
{

}