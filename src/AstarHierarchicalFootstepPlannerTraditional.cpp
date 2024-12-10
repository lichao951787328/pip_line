#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerTraditional.h>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <chrono>

void AstarHierarchicalFootstepPlannerTraditional::setCheckParam(double checkXupper_, double checkXButton_)
{
    checkXupper = checkXupper_;
    checkXButton = checkXButton_;
}

bool AstarHierarchicalFootstepPlannerTraditional::isStartFeasible(Eigen::Vector3d start, Eigen::Vector3d & left_foot, Eigen::Vector3d & right_foot)
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
            left_foot_tmp.z() = start.z();
            Eigen::Vector3d right_foot_tmp = ad.toRotationMatrix() * -half_hip_width + mid;
            right_foot_tmp.z() = start.z();

            vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> left_cands = fineLandPoint(left_foot_tmp);
            vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> right_cands = fineLandPoint(right_foot_tmp);
            double score = - std::numeric_limits<double>::infinity();
            double opt_height = -std::numeric_limits<double>::infinity();
            double opt_pitch = -std::numeric_limits<double>::infinity();
            double opt_roll = -std::numeric_limits<double>::infinity();
            int opt_plane_index = -1;
            Eigen::Vector3d left_opt;
            for (auto & cand : left_cands)
            {
                double tmpscore  = -std::numeric_limits<double>::infinity();
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
    #ifdef DEBUG
                LOG(INFO)<<"No Feasible Left Foot";
    #endif
                return false;
            }
        
            opt_plane_index = -1;
            score = - std::numeric_limits<double>::infinity();
            opt_height = -std::numeric_limits<double>::infinity();
            opt_pitch = -std::numeric_limits<double>::infinity();
            opt_roll = -std::numeric_limits<double>::infinity();
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
    #ifdef DEBUG
                LOG(INFO)<<"No Feasible Transition";
    #endif
                return false;
            }
            left_foot = left_opt;
            right_foot = right_opt;
            return true;
//             int max_size_left, max_size_right;
//             int above_points_left, above_points_right;
//             Eigen::Vector3d left_plane_normal, right_plane_normal;
//             int left_plane_index, right_plane_index;
//             double left_step_height, right_step_height;
//             double left_roll, right_roll, left_pitch, right_pitch;
//             if (computeLandInfo(left_foot_tmp, max_size_left, above_points_left, left_plane_normal, left_step_height, left_plane_index, left_pitch, left_roll) && computeLandInfo(right_foot_tmp, max_size_right, above_points_right, right_plane_normal, right_step_height, right_plane_index, right_pitch, right_roll))
//             {
//                 if (max_size_left < 0.5 * footsize_inmap || max_size_right < 0.5 * footsize_inmap)
//                 {
// #ifdef DEBUG
//                     LOG(INFO)<<" is too small";
//                     LOG(INFO)<<"left size: "<<max_size_left<<" right size: "<<max_size_right<<" footsize_inmap: "<<footsize_inmap;
// #endif
//                     return false;
//                 }
//                 if (above_points_left > 0 || above_points_right > 0)
//                 {
// #ifdef DEBUG
//                     LOG(INFO)<<"above points is not zero";
// #endif
//                     return false;
//                 }
//                 left_foot = left_foot_tmp;
//                 right_foot = right_foot_tmp;
//                 return true;
//             }
//             else
//             {
// #ifdef DEBUG
//                 LOG(INFO)<<"compute land info failed";
// #endif
//                 return false;
//             }
        }
        else
        {
#ifdef DEBUG
            LOG(INFO)<<"FOOT is not in localmap";
#endif
            return false;
        }
    }
    else
    {
#ifdef DEBUG
        LOG(INFO)<<"start point is not in localmap";
#endif
        return false;
    }
}

bool AstarHierarchicalFootstepPlannerTraditional::nodeExtension(FootstepNodePtr current_node, FootstepNodePtr pre_node, vector<FootstepNodePtr> & child_nodes)
{
    // 基础节点，在地图坐标系下的节点
    child_nodes.clear();
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> special_transitions = basicTransitions(current_node);

#ifdef DEBUG
    LOG(INFO)<<"node size: "<<special_transitions.size();
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
        int plane_index = -1;
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
                    int fine_plane_index = -1;
                    double pitch, roll;
                    bool fine_dangerous = false;
                    if (computeTransitionScore(tr, current_node, pre_node, fine_dangerous, tmp_score, height, fine_normal, fine_plane_index, pitch, roll))
                    {
                        if (!fine_dangerous)
                        {
                            ScoreMarkerNodePtr node_p = std::make_shared<ScoreMarkerNode>(tr.second, tmp_score, height, fine_normal, fine_plane_index, roll, pitch);
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
        stepnode->PreFootstepNode = current_node;
        // 跨平面运动时对规划的落脚点限制
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
        if (stepnode->footstep.pitch <= - 5/57.3 && stepnode->PreFootstepNode->footstep.pitch <= - 5/57.3)
        {
            Eigen::Vector2d dis_v1(stepnode->footstep.x, stepnode->footstep.y);
            Eigen::Vector2d dis_v2(stepnode->PreFootstepNode->PreFootstepNode->footstep.x, stepnode->PreFootstepNode->PreFootstepNode->footstep.y);
            if (abs((dis_v1 - dis_v2).norm()) > 0.32)
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
bool AstarHierarchicalFootstepPlannerTraditional::computeLandInfo(Eigen::Vector3d ankle, int & max_size, int & above_points, Eigen::Vector3d & plane_normal, double & step_height, int & plane_index, double & pitch, double & roll)
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
                plane_index = -1;
                for (int i = 0; i < plane_images.size(); i++)
                {
                    cv::Mat intersection;
                    cv::bitwise_and(plane_images.at(i), simage, intersection);
// #ifdef DEBUG
//                     cv::imshow("intersection", intersection);
//                     cv::waitKey(0);
//                     LOG(INFO) << "show image: ";
// #endif
                    if (cv::countNonZero(simage & (~intersection)) == 0)
                    {
                        plane_index = i;
                        break;
                    }
                }
#ifdef DEBUG
                LOG(INFO)<<"plane index: "<<plane_index;
#endif
                if (plane_index == -1)
                {
                    return false;
                }
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

bool AstarHierarchicalFootstepPlannerTraditional::computerLeftRightGoal(Eigen::Vector3d goal)
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
        double opt_height = -std::numeric_limits<double>::infinity();
        double opt_pitch = -std::numeric_limits<double>::infinity();
        double opt_roll = -std::numeric_limits<double>::infinity();
        int opt_plane_index = -1;
        Eigen::Vector3d left_opt;
        for (auto & cand : left_cands)
        {
            double tmpscore = -std::numeric_limits<double>::infinity();
            double height = -std::numeric_limits<double>::infinity();
            int plane_index = -1;
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
        }
        else
        {
            return false;
        }
        opt_plane_index = -1;
        score = - std::numeric_limits<double>::infinity();
        opt_height = -std::numeric_limits<double>::infinity();
        opt_pitch = -std::numeric_limits<double>::infinity();
        opt_roll = -std::numeric_limits<double>::infinity();
        Eigen::Vector3d right_opt;
        for (auto & cand : right_cands)
        {
            double tmpscore = -std::numeric_limits<double>::infinity();
            double height = -std::numeric_limits<double>::infinity();
            int plane_index = -1;
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

bool AstarHierarchicalFootstepPlannerTraditional::checkFeasibleGoal(Eigen::Vector3d goal)
{
    Eigen::Vector3d left_offset(0, hip_width/2.0, 0);
    Eigen::Vector3d right_offset(0, -hip_width/2.0, 0);
    Eigen::AngleAxisd ad(goal.z(), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d left_goal, right_goal;
    left_goal.head(2) = (ad.toRotationMatrix() * left_offset + Eigen::Vector3d(goal.x(), goal.y(), 0)).head(2);
    left_goal.z() = goal.z();
    right_goal.head(2) = (ad.toRotationMatrix() * right_offset + Eigen::Vector3d(goal.x(), goal.y(), 0)).head(2);
    right_goal.z() = goal.z();
    // LOG(INFO)<<"left_goal: "<<left_goal.transpose();
    // LOG(INFO)<<"right_goal: "<<right_goal.transpose();
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
        double opt_height = -std::numeric_limits<double>::infinity();
        double opt_pitch = -std::numeric_limits<double>::infinity();
        double opt_roll = -std::numeric_limits<double>::infinity();
        int opt_plane_index = -1;
        Eigen::Vector3d left_opt;
        for (auto & cand : left_cands)
        {
            double tmpscore  = -std::numeric_limits<double>::infinity();
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
#ifdef DEBUG
            LOG(INFO)<<"No Feasible Left Foot";
#endif
            return false;
        }
        
        opt_plane_index = -1;
        score = - std::numeric_limits<double>::infinity();
        opt_height = -std::numeric_limits<double>::infinity();
        opt_pitch = -std::numeric_limits<double>::infinity();
        opt_roll = -std::numeric_limits<double>::infinity();
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
#ifdef DEBUG
            LOG(INFO)<<"No Feasible Transition";
#endif
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

// bool AstarHierarchicalFootstepPlannerTraditional::SqurePoints(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, vector<Eigen::Vector3d> & points)
// {
// #ifdef DEBUG
//     LOG(INFO)<<"SqurePoints: "<<TL.transpose()<<" "<<TR.transpose()<<" "<<BL.transpose()<<" "<<BR.transpose();
//     LOG(INFO)<<label_localmap.getLength().transpose()<<" "<<label_localmap.getSize().transpose();
// #endif
//     if (label_localmap.isInside(TL) && label_localmap.isInside(TR) && label_localmap.isInside(BL) && label_localmap.isInside(BR))
//     {
//         grid_map::LineIterator iterator_start(label_localmap, BR, BL);
//         grid_map::LineIterator iterator_end(label_localmap, TR, TL);
//         for (; !iterator_start.isPastEnd()&&!iterator_end.isPastEnd(); ++iterator_start, ++iterator_end)
//         {
//             grid_map::Index start_index(*iterator_start);
//             grid_map::Index end_index(*iterator_end);
//             for (grid_map::LineIterator iterator_l(label_localmap, start_index, end_index); !iterator_l.isPastEnd(); ++iterator_l)
//             {
//                 const grid_map::Index index_l(*iterator_l);
//                 grid_map::Position position_l;
//                 if (label_localmap.getPosition(index_l, position_l))
//                 {
//                     grid_map::Position3 cor_position;
//                     if (label_localmap.getPosition3("elevation", index_l, cor_position))
//                     {
//                         if (!std::isnan(cor_position.z()))
//                         {
//                             points.emplace_back(cor_position);
//                         }
//                     }
//                 }
//             }
//         }
// #ifdef DEBUG
//         LOG(INFO)<<"SqurePoints: "<<points.size();
// #endif
//         return true;
//     }
//     else
//     {
// #ifdef DEBUG
//         LOG(INFO)<<"SqurePoints: one or more points not in map";
// #endif
//         return false;
//     }
// }


AstarHierarchicalFootstepPlannerTraditional::~AstarHierarchicalFootstepPlannerTraditional()
{

}