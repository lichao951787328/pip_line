#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerTra.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <glog/logging.h>
// #include <PEAC_AHFP/plane_fitter_pcl_AHFP.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <pcl/io/pcd_io.h>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <Eigen/Dense>
// 这个构造函数，有问题，暂时不要使用
AstarHierarchicalFootstepPlanner::AstarHierarchicalFootstepPlanner(grid_map::GridMap & lm, FootParam footparam_, double hip_width_)
{
    localmap = lm;
    // 加一个对localmap提取平面的操作
    resolution = localmap.getResolution();
    // debug_pub = 
    mapsize = localmap.getSize().x() * localmap.getSize().y();
    // LOG(INFO)<<localmap.getSize().transpose();
    label_index.resize(localmap.getSize().x(), localmap.getSize().y());
    // LOG(INFO)<<localmap.getSize().y()<<" "<<localmap.getSize().x();
    // LOG(INFO)<<label_index.rows()<<" "<<label_index.cols();
    // computePlanarInfor();
    footparam = footparam_;
    hip_width = hip_width_;
    footsize_inmap = (ceil((footparam.x_button + footparam.x_upper)/resolution)) * (ceil((footparam.y_left + footparam.y_right)/resolution));

#ifdef DEBUG
    LOG(INFO)<<"resolution: "<<resolution;
    LOG(INFO)<<"mapsize: "<<mapsize;

    LOG(INFO)<<"footparam: "<<footparam.x_upper<<" "<<footparam.x_button<<" "<<footparam.y_left<<" "<<footparam.y_right;
    LOG(INFO)<<"hip_width: "<<hip_width;
    LOG(INFO)<<"footsize_inmap: "<<footsize_inmap;
    // outfile = std::ofstream("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/data/out.txt");
    outfile = std::ofstream("/home/lichao/TCDS/src/pip_line/data/out.txt");
#endif
    // 每个transition x y yaw 右脚往左扩展的步态点
    initial_transitions();
    // LOG(INFO)<<"transitions size: "<<transitions.size();
}

// 请确保不会出现某个栅格点的label为nan，而其周围的栅格点label确实一个值，如果按照自己的平面提取算法，应该不会出现这个情况
AstarHierarchicalFootstepPlanner::AstarHierarchicalFootstepPlanner(grid_map::GridMap & label_map, cv::Mat & plane_iamage_, vector<cv::Mat> & planes_image_, vector<planeInfo> & planes_info_, FootParam footparam_, double hip_width_):label_localmap(label_map),footparam(footparam_), hip_width(hip_width_), plane_image(plane_iamage_), plane_images(planes_image_),planes_info(planes_info_)
{
    localmap = label_localmap;

    // localmap.erase("label");  // 不知道为什么会报错

    resolution = localmap.getResolution();
    mapsize = localmap.getSize().x() * localmap.getSize().y();
    footsize_inmap = ((ceil)((footparam.x_button + footparam.x_upper)/resolution)) * ((ceil)((footparam.y_left + footparam.y_right)/resolution));
    planes = plane_images.size();
#ifdef DEBUG
    LOG(INFO)<<"resolution: "<<resolution;
    LOG(INFO)<<"mapsize: "<<mapsize;

    LOG(INFO)<<"footparam: "<<footparam.x_upper<<" "<<footparam.x_button<<" "<<footparam.y_left<<" "<<footparam.y_right;
    LOG(INFO)<<"hip_width: "<<hip_width;
    LOG(INFO)<<"footsize_inmap: "<<footsize_inmap;
    // outfile = std::ofstream("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/data/out.txt");
    outfile = std::ofstream("/home/lichao/TCDS/src/pip_line/data/out.txt");
#endif
    initial_transitions();
    LOG(INFO)<<"construct planner over";
}

void AstarHierarchicalFootstepPlanner::initial_transitions()
{
    for (int i = -1; i < 4; i++)
    {
        for (int j = -2; j < 5; j++)
        {
            for (int k = -1; k < 3; k++)
            {
                if (j == -1 && (k == -3 || k == -2))// 靠的太近时角度不允许内转太多
                {
                    continue;
                }
                if (j == 0 && k ==-3)
                {
                    continue;
                }
                
                Eigen::Vector3d transition = Eigen::Vector3d(i * 0.095,   0.02 * j + 0.24,  k*3/57.3);
                // LOG(INFO)<<transition.transpose();
                transitions.emplace_back(transition);
            }
        }
    } 

    for (int i = -1; i < 2; i++)
    {
        for (int j = -2; j < 3; j++)
        {
            for (int k = -1; k < 2; k++)
            {
                if (j == -1 && k == -2)// 靠的太近时角度不允许内转太多
                {
                    continue;
                }
                
                Eigen::Vector3d transition = Eigen::Vector3d(i * 0.06,   0.02 * j + 0.24,  k*3/57.3);
                // LOG(INFO)<<transition.transpose();
                combine_transitions.emplace_back(transition);
            }
        }
    }
}

// 默认起点和终点位置是对的
bool AstarHierarchicalFootstepPlanner::initial(Eigen::Vector3d start, Eigen::Vector3d prestart, int support_side, Eigen::Vector3d goal)
{
    if (support_side == 0) // 左脚支撑
    {
        // 左脚支撑时，证明start是左脚
        CHECK(start.y() > prestart.y());
        start_p = std::make_shared<FootstepNode>(start, 0);
        prestart_p = std::make_shared<FootstepNode>(prestart, 1);
        if (!(startPoint2Node(start, start_p) && startPoint2Node(prestart, prestart_p)))
        {
            LOG(ERROR)<<"error initial start state";
            return false;
        }
        start_p->Hcost = 0;
        start_p->Gcost = 0;
        start_p->cost = 0;
        start_p->PreFootstepNode = prestart_p;
        prestart_p->Hcost = 0.0;
        prestart_p->Gcost = 0.0;
        prestart_p->cost = 0.0;
    }
    else if (support_side == 1) // 右脚支撑
    {
        // 右脚支撑时，证明右脚是start，左脚是prestart
        CHECK(start.y() < prestart.y());
        start_p = std::make_shared<FootstepNode>(start, 1);
        prestart_p = std::make_shared<FootstepNode>(prestart, 0);
        if (!(startPoint2Node(start, start_p) && startPoint2Node(prestart, prestart_p)))
        {
            LOG(ERROR)<<"error initial start state";
            return false;
        }
        start_p->Hcost = 0;
        start_p->Gcost = 0;
        start_p->cost = 0;
        start_p->PreFootstepNode = prestart_p;
        prestart_p->Hcost = 0.0;
        prestart_p->Gcost = 0.0;
        prestart_p->cost = 0.0;
    }
    else if (support_side == 2) // 双脚支撑 暂时还没想好怎么写 得根据实际情况来写
    {
        /* code */
    }
    else // 出现错误
    {
        LOG(ERROR)<<"error about support_side";
    }
    LOG(INFO)<<prestart_p->footstep.x<<" "<<prestart_p->footstep.y<<" "<<prestart_p->footstep.z<<" "<<prestart_p->footstep.roll<<" "<<prestart_p->footstep.pitch<<" "<<prestart_p->footstep.yaw<<" "<<prestart_p->footstep.robot_side<<" "<<start_p->Hcost<<" "<<start_p->Gcost<<" "<<start_p->cost;
    LOG(INFO)<<start_p->footstep.x<<" "<<start_p->footstep.y<<" "<<start_p->footstep.z<<" "<<start_p->footstep.roll<<" "<<start_p->footstep.pitch<<" "<<start_p->footstep.yaw<<" "<<start_p->footstep.robot_side<<" "<<start_p->Hcost<<" "<<start_p->Gcost<<" "<<start_p->cost;
    
#ifdef DEBUG
    LOG(INFO)<<start_p->footstep.x<<" "<<start_p->footstep.y<<" "<<start_p->footstep.z<<" "<<start_p->footstep.roll<<" "<<start_p->footstep.pitch<<" "<<start_p->footstep.yaw<<" "<<start_p->footstep.robot_side<<" "<<start_p->Hcost<<" "<<start_p->Gcost<<" "<<start_p->cost;
    LOG(INFO)<<prestart_p->footstep.x<<" "<<prestart_p->footstep.y<<" "<<prestart_p->footstep.z<<" "<<prestart_p->footstep.roll<<" "<<prestart_p->footstep.pitch<<" "<<prestart_p->footstep.yaw<<" "<<prestart_p->footstep.robot_side<<" "<<start_p->Hcost<<" "<<start_p->Gcost<<" "<<start_p->cost;
    grid_map::Index start_index, pre_start_index;
    // if (localmap.getIndex(grid_map::Position(start_p->footstep.x, start_p->footstep.y), start_index) && localmap.getIndex(grid_map::Position(prestart_p->footstep.x, prestart_p->footstep.y), pre_start_index))
    // {
    //     cv::circle(plane_image, cv::Point(start_index.y(), start_index.x()), 3, cv::Scalar(0, 0, 255), 2);
    //     cv::circle(plane_image, cv::Point(pre_start_index.y(), pre_start_index.x()), 3, cv::Scalar(0, 0, 255), 2);
    // }
    // cv::imshow("start pre start", plane_image);
    // cv::waitKey(0);
#endif
    end_p = std::make_shared<FootstepNode>(goal, 0);
    if (!computerLeftRightGoal(goal))
    {
        LOG(ERROR)<<"EEROR GOAL";
        return false;
    }
    LOG(INFO)<<"GOAL FINISH";
#ifdef DEBUG
    if (!outfile.is_open()) 
    {
        std::cerr << "无法打开文件" << std::endl;
        return false;
    }
#endif
    return true;
}

bool AstarHierarchicalFootstepPlanner::getPointInfoInPlane(Eigen::Vector3d p, double & height, double & pitch, double & roll)
{
    int max_size = 0, above_points = 0;
    Eigen::Vector3d plane_normal = Eigen::Vector3d::Zero();
    height = -std::numeric_limits<double>::infinity();
    pitch = std::numeric_limits<double>::infinity();
    roll = std::numeric_limits<double>::infinity();
    if (computeLandInfo(p, max_size, above_points, plane_normal, height, pitch, roll))
    {
#ifdef DEBUG
        LOG(INFO)<<"plane_normal = "<<plane_normal.transpose();
#endif
        if (plane_normal.z() > 0.85)
        {
            if (above_points == 0)
            {
                if (max_size > 0.65 * footsize_inmap)
                {
                    return true;
                }
                else
                {
#ifdef DEBUG
                    LOG(INFO)<<"support error";
#endif
                    return false;
                }
            }
            else
            {
#ifdef DEBUG
                LOG(INFO)<<"above points error";
#endif
                return false;
            }
        }
        else
        {
#ifdef DEBUG
            LOG(INFO)<<"normal is error";
#endif
            return false;
        }
    }
    else
    {
#ifdef DEBUG
        LOG(INFO)<<"get land info error";
#endif
        return false;
    }
}

// 是不是可以换成平面的形式
bool AstarHierarchicalFootstepPlanner::startPoint2Node(Eigen::Vector3d p, FootstepNodePtr node)
{
    double height = 0.0, pitch = 0, roll = 0;

    if (getPointInfoInPlane(p, height, pitch, roll))
    {
        node->footstep.x = p(0);
        node->footstep.y = p(1);
        node->footstep.z = height;
        node->footstep.yaw = p(2);
        node->footstep.roll = roll;
        node->footstep.pitch = pitch;
        return true;
    }
    else
    {
#ifdef DEBUG
        LOG(ERROR)<<"get info error";
#endif
        return false;
    }
}

// 改用xy坐标计算平面点的z坐标更合适
// bool AstarHierarchicalFootstepPlanner::computeTransitionHeight(Eigen::Vector3d transition, double & transition_height)
// {
//     int plane_index;
//     if (getPointHeightInPlane(transition.head(2), transition_height, plane_index))
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }

// tested
// 是否是危险的，是分数为0，并需要微调
// 不是危险的，计算得分，得分过低的不再作为扩展节点
bool AstarHierarchicalFootstepPlanner::computeTransitionScore(std::pair<Eigen::Vector3d, Eigen::Vector3d> transition, FootstepNodePtr current_node, FootstepNodePtr pre_node, bool & dangerous, double & score, double & height, Eigen::Vector3d & plane_normal, double & pitch, double & roll)
{
    dangerous = false;
    score = 0;
    // LOG(INFO)<<"function computeTransitionScore";
    // 如果采集的支撑区域位于同一平面，肯定可以，如果采集的支撑区域绝大部分区域位于同一平面，且其他位于平面之下，注意不是高度值小于平面。注意在此处需要考虑分层的打分，比方说缺少的区域或者发生碰撞的区域，如果发生碰撞的区域位于边缘区域，那么可以通过打分再通过后续细化
    int max_size = 0; // 属于同一最大平面的点数
    int above_points = 0; // 超出限制部分的点数
    plane_normal = Eigen::Vector3d::Zero();
    height = -std::numeric_limits<double>::infinity();
    pitch = std::numeric_limits<double>::infinity();
    roll = std::numeric_limits<double>::infinity();
    if (!computeLandInfo(transition.second, max_size, above_points, plane_normal, height, pitch, roll))
    {
#ifdef DEBUG
        LOG(INFO)<<"CANNOT computeLandInfo";
#endif
        return false;
    }
    // int judge_num = max(area_num, footsize_inmap);
    // 判断是否是危险的
    // 超出点为15-30， 支撑小于。 则认为是危险的，需要微调，这样才能满足落脚需求
    // if (above_points > 5 || max_size < 0.7 * footsize_inmap)
    // {
    //     dangerous = true;
    // }
    // 不管支撑面积，只管有没有刺脚板
    if (above_points > 0)
    {
        dangerous = true;
    }

    // LOG(INFO)<<"1";
    // double transition_height;
    // if (!computeTransitionHeight(transition.second, transition_height))
    // {
    //     LOG(INFO)<<"CANNOT computeTransitionHeight";
    //     return false;
    // }
    // 需要通过平面获取高度

    // 高度太高，则去除，高度也会是一个打分项
    double height_change = height - current_node->footstep.z;
    if (height_change > 0.16 || height_change < -0.15)
    {
#ifdef DEBUG
        LOG(INFO)<<"height is unsuaitable";
        LOG(INFO)<<"height: "<<height;
#endif
        // LOG(INFO)<<transition_height<<" "<<current_node->footstep.z;
        return false;
    }
    // LOG(INFO)<<"2";
    // 摆动脚跨越高度，这也是一个打分项
    double height_change_swing, swing_height_max;
    // LOG(INFO)<<pre_node->footstep.x<<" "<<pre_node->footstep.y<<" "<<transition.second.head(2).transpose();
    if (!swingHeight(grid_map::Position(pre_node->footstep.x, pre_node->footstep.y), transition.second.head(2), height_change_swing, swing_height_max))
    {
#ifdef DEBUG
        LOG(INFO)<<"height_change_swing: "<<height_change_swing<<", swing_height_max: "<<swing_height_max;
#endif
        return false;
    }
    if (swing_height_max > 0.45)
    {
#ifdef DEBUG
        LOG(INFO)<<"swing_height_max";
#endif
        return false;
    }
    if (abs(height_change_swing) > 0.35)
    {
#ifdef DEBUG
        LOG(INFO)<<"height_change_swing";
#endif
        return false;
    }
    // LOG(INFO)<<"compute height";

    // 根据这些信息能确定脚的角度roll和pitch
    // double roll, pitch;
    // Eigen::Vector3d eular;
    // computeRollPitch(plane_normal, transition.second.z(), eular);
    // 根据上面的点能对每个transition进行打分
    // 打分的标准1. 超出高度数量； 2. 总数量； 3. 哪个位置更好；4：roll角
    // double score1 = - (int(above_points)) * 0.01; // 这个是不是可以不参与排序
    // double score2 = 0;
    // if (footsize_inmap > max_size)
    // {
    //     score2 = -((footsize_inmap - max_size)) * 0.001;
    // }
    double score3 = - abs(height_change) * 0.05;
    double score4 = - abs(height_change_swing) * 0.05;
    double score5 = - abs(swing_height_max) * 0.05;
    // double score6 = 没有对应关系了，怎么确定分数xy方向的分数
    double score6 = - abs(transition.first.x() - 0.3) * 0.2;
    double score7 = - abs(transition.first.y() - 0.2) * 0.15;
    double score10 = - abs(transition.first.z()) * 1;// yaw角打分
    // double score10 = yaw
    double score8 = - abs(roll) * 5;
    double score9 = - abs(pitch) * 3;
    // score = score1 + score2 + score3 + score4 + score5 + score6 + score7 + score8 + score9 + score10;
    score = score3 + score4 + score5 + score6 + score7 + score8 + score9 + score10;
    // LOG(INFO)<<score1<<" "<<score2<<" "<<score3<<" "<<score4<<" "<<score5<<" "<<score6<<" "<<score7<<" "<<score8<<" "<<score9<<" "<<score10;
    // LOG(INFO)<<"score: "<<score;
    return true;
}

// bool  AstarHierarchicalFootstepPlanner::point2Node(Eigen::Vector3d p, FootstepNodePtr node)
// {
//     double z;
//     Eigen::Vector3d eular;
//     int plane_index;
//     if (computeZRollPitch(p, z, eular, plane_index))
//     {
//         node->footstep.z = z;
//         node->footstep.roll = eular(2);
//         node->footstep.pitch = eular(1);
//         node->footstep.yaw = p.z();
//         node->plane_index = plane_index;
//         return true;
//     }
//     else
//     {
// #ifdef DEBUG
//         LOG(INFO)<<"can not set node";
// #endif
//         return false;
//     }
// }

// tested 计算落脚点的得分
bool AstarHierarchicalFootstepPlanner::computeLandPointScore(std::pair<Eigen::Vector3d, Eigen::Vector3d> land_point, double & score, double & height, double & pitch, double & roll)
{
    int max_size = 0; // 属于同一最大平面的点数
    int above_points = 0; // 超出限制部分的点数
    Eigen::Vector3d plane_normal = Eigen::Vector3d::Zero();
    height = -std::numeric_limits<double>::infinity();
    pitch = std::numeric_limits<double>::infinity();
    roll = std::numeric_limits<double>::infinity();
    if (!computeLandInfo(land_point.second, max_size, above_points, plane_normal, height, pitch, roll))
    {
#ifdef DEBUG
        LOG(INFO)<<"can not get land info";
#endif
        return false;
    }
    if (above_points > 0)
    {
        return false;
    }
    double score2 = 0; 
    if (footsize_inmap > max_size)
    {
        score2 = -(footsize_inmap - max_size) * 0.001;
    }
    
    double score6 = - abs(land_point.first.x()) * 0.1;
    double score7 = - abs(land_point.first.y()) * 0.15;
    double score10 = - abs(land_point.first.z()) * 1;// yaw角打分
    double score8 = - abs(roll) * 7;
    double score9 = - abs(pitch) * 5;
    score = score2 + score6 + score7 + score8 + score9 + score10;
    return true;
}

// 计算落脚位置对应的z，roll，pitch值
// bool AstarHierarchicalFootstepPlanner::computeZRollPitch(Eigen::Vector3d point, double & z, Eigen::Vector3d & eular, int & plane_index)
// {
//     int max_size = 0; // 属于同一最大平面的点数
//     int above_points = 0; // 超出限制部分的点数
//     Eigen::Vector3d plane_normal;
//     double height = 0;
//     if (!computeLandInfo(point, max_size, above_points, plane_normal, height))
//     {
// #ifdef DEBUG
//         LOG(INFO)<<"not land info";
// #endif
//         return false;
//     }

//     // 根据这些信息能确定脚的角度roll和pitch
//     computeRollPitch(plane_normal, point.z(), eular);
//     if (eular(2) > 10/57.3 || eular(1) > 15/57.3)
//     {
//         LOG(INFO)<<"roll or pitch is unsuitable";
//         LOG(INFO)<<(eular*57.3).transpose();
//         return false;
//     }
//     return true;
// }

// tested 更严格的标准判断是否合格
// bool AstarHierarchicalFootstepPlanner::computeTransitionStrictScore(std::pair<Eigen::Vector3d, Eigen::Vector3d> transition, FootstepNodePtr current_node, FootstepNodePtr pre_node, double & score, double & height, Eigen::Vector3d & plane_normal)
// {
//     int max_size = 0; // 属于同一最大平面的点数
//     int above_points = 0; // 超出限制部分的点数
//     // Eigen::Vector3d plane_normal;
//     // double step_height = 0.;
//     if (!computeLandInfo(transition.second, max_size, above_points, plane_normal, height))
//     {
//         return false;
//     }
//     if (above_points > 0)// 这个值也需要调
//     {
//         return false;
//     }
//     // if (max_size < 0.85 * footsize_inmap)
//     // {
//     //     return false;
//     // }
//     // double transition_height;
//     // if (!computeTransitionHeight(transition.second, transition_height))
//     // {
//     //     return false;
//     // }
//     // 高度太高，则去除，高度也会是一个打分项
//     double height_change = height - current_node->footstep.z;
//     // LOG(INFO)<<transition_height<<" "<<current_node->footstep.z;
//     if (height_change > 0.2 || height_change < -0.15)
//     {
//         return false;
//     }
//     // 摆动脚跨越高度，这也是一个打分项
//     double height_change_swing, swing_height_max;
//     if (!swingHeight(grid_map::Position(pre_node->footstep.x, pre_node->footstep.y), transition.second.head(2), height_change_swing, swing_height_max))
//     {
//         return false;
//     }
//     if (swing_height_max > 0.45)
//     {
//         return false;
//     }
//     if (abs(height_change_swing) > 0.35)
//     {
//         return false;
//     }
//     Eigen::Vector3d eular;
//     computeRollPitch(plane_normal, transition.second.z(), eular);
//     // 根据上面的点能对每个transition进行打分
//     // 打分的标准1. 超出高度数量； 2. 总数量； 3. 哪个位置更好；4：roll角
//     double score2 = 0;
//     if (footsize_inmap > max_size)
//     {
//         score2 = -((footsize_inmap - max_size)) * 0.001;
//     }
//     double score3 = - abs(height_change) * 0.05;
//     double score4 = - abs(height_change_swing) * 0.05;
//     double score5 = - abs(swing_height_max) * 0.05;
//     // double score6 = 没有对应关系了，怎么确定分数xy方向的分数
//     double score6 = - abs(transition.first.x() - 0.3) * 0.2;
//     double score7 = - abs(transition.first.y() - 0.2) * 0.15;
//     double score10 = - abs(transition.first.z()) * 1;// yaw角打分
//     // double score10 = yaw
//     double score8 = - abs(eular(2)) * 5;
//     double score9 = - abs(eular(1)) * 3;
//     score = score2 + score3 + score4 + score5 + score6 + score7 + score8 + score9 + score10;
//     // LOG(INFO)<<score2<<" "<<score3<<" "<<score4<<" "<<score5<<" "<<score6<<" "<<score7<<" "<<score8<<" "<<score9<<" "<<score10;
//     return true;
// }

bool AstarHierarchicalFootstepPlanner::nodeExtension(FootstepNodePtr current_node, FootstepNodePtr pre_node, vector<FootstepNodePtr> & child_nodes)
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
        double pitch, roll;
        if (computeTransitionScore(transition, current_node, pre_node, dangerous, score, height, normal, pitch, roll))
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
                    double pitch, roll;
                    bool fine_dangerous = false;
                    if (computeTransitionScore(tr, current_node, pre_node, fine_dangerous, tmp_score, height, fine_normal, pitch, roll))
                    {
                        if (!fine_dangerous)
                        {
                            ScoreMarkerNodePtr node_p = std::make_shared<ScoreMarkerNode>(tr.second, tmp_score, height, fine_normal, roll, pitch);
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
                ScoreMarkerNodePtr node_p = std::make_shared<ScoreMarkerNode>(transition.second, score, height, normal, roll, pitch);
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

    while (!basicScoreNodes.empty())
    {
        auto node = basicScoreNodes.top();
        // 由基础偏移量转到实际位置
        basicScoreNodes.pop();
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

// 根据变换前后的向量求变换矩阵 u变换前的向量 v变换后的向量
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

// 当旋转矩阵接近单位阵时，转换的欧拉角可能是一个接近pai的角，这个函数用于调整欧拉角为一个接近0的角
Eigen::Vector3d AstarHierarchicalFootstepPlanner::adjustEulerAngles(const Eigen::Vector3d& eulerAngles) 
{
    Eigen::Vector3d adjusted = eulerAngles;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(adjusted[i]) > M_PI / 2) {
            if (adjusted[i] > 0) {
                adjusted[i] = adjusted[i] - M_PI;
            } else {
                adjusted[i] = adjusted[i] + M_PI;
            }
        }
    }
    return adjusted;
}

// 四元数转欧拉角的方案
// 四元数 --> 欧拉角(Z-Y-X，即RPY)（确保pitch的范围[-pi/2, pi/2]）
Eigen::Vector3d AstarHierarchicalFootstepPlanner::Quaterniond2EulerAngles(Eigen::Quaterniond q) 
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

// yaw pitch roll
Eigen::Vector3d AstarHierarchicalFootstepPlanner::Matrix3d2EulerAngles(Eigen::Matrix3d m)
{
    Eigen::Quaterniond qd(m);
    return Quaterniond2EulerAngles(qd);
}


// 已确定
// tested
// 注意eulerAngles(2, 1, 0) 出来的分别是绕z，y，x的旋转角
// 但是eigen本身的eulerAngles是有问题的，所以使用网上使用的Matrix3d2EulerAngles
void AstarHierarchicalFootstepPlanner::computeRollPitch(Eigen::Vector3d normal, double yaw, Eigen::Vector3d & euler)
{
    // 注意根据斜面的法向量只能求取roll和pitch，yaw还是转角
    Eigen::Matrix3d T_f_m = computeRotationMatrix(Eigen::Vector3d::UnitZ(), normal);
    // LOG(INFO)<<T_f_m;
    Eigen::AngleAxisd ad_m_w(yaw, Eigen::Vector3d::UnitZ());
    // LOG(INFO)<<ad_m_w.toRotationMatrix();
    Eigen::Matrix3d T_f_w = T_f_m * ad_m_w.toRotationMatrix();
    euler = Matrix3d2EulerAngles(T_f_w);
    // Eigen::Vector3d euler_cand = T_f_w.eulerAngles(2, 1, 0);// z, y, x
    // euler = adjustEulerAngles(euler_cand);
#ifdef DEBUG
    // LOG(INFO)<<"roll: "<<roll<<" pitch: "<<pitch;
#endif

}

// tested
bool AstarHierarchicalFootstepPlanner::swingHeight(grid_map::Position start, grid_map::Position end, double & swing_height_change, double & swing_height_max)
{
    // LOG(INFO)<<"function swingHeight";
    swing_height_max = 0.0;
    swing_height_change = 0.0;
    grid_map::Index start_index, end_index;
    if (label_localmap.getIndex(start, start_index) && label_localmap.getIndex(end, end_index))
    {
        grid_map::Position3 start_p3, end_p3;
        if (label_localmap.getPosition3("elevation", start_index, start_p3))
        {
            if (label_localmap.getPosition3("elevation", end_index, end_p3))
            {
                swing_height_change = end_p3.z() - start_p3.z();
                // LOG(INFO)<<"start z: "<<start_p3.z()<<", end z: "<<end_p3.z();
                for (grid_map::LineIterator iterator_l(label_localmap, start_index, end_index); !iterator_l.isPastEnd(); ++iterator_l)
                {
                    const grid_map::Index index_l(*iterator_l);
                    grid_map::Position3 p3;
                    
                    if (label_localmap.getPosition3("elevation", index_l, p3))
                    {
                        if (!std::isnan(p3.z()))
                        {
                            // LOG(INFO)<<"P3: "<<p3.transpose();
                            if ((p3.z() - start_p3.z()) > swing_height_max)
                            {
                                swing_height_max = (p3.z() - start_p3.z());
                            }
                        }
                    }
                }
            }
            else
            {
#ifdef DEBUG
                LOG(ERROR)<<end.transpose();
                LOG(ERROR)<<end_index.transpose();
                LOG(ERROR)<<"can not get end elevation.";
#endif
                return false;
            }
        }
        else
        {
#ifdef DEBUG
            LOG(ERROR)<<start.transpose();
            LOG(ERROR)<<start_index.transpose();
            LOG(ERROR)<<"can not get start elevation.";
#endif
            return false;
        }
        return true;
    }
    else
    {
#ifdef DEBUG
        LOG(ERROR)<<"start or end out of map";
#endif
        return false;
    }
}

// tested
vector<Eigen::Vector3d> AstarHierarchicalFootstepPlanner::fineTransitionsBasic(Eigen::Vector3d parent_transition)
{
    vector<Eigen::Vector3d> child_transitions;
    for (double x = - 2 * resolution; x <=  2* resolution; x = x + resolution)
    {
        for (double y = - resolution; y <=  resolution; y = y + resolution)
        {
            Eigen::Vector3d child1_trans(parent_transition.x() + x, parent_transition.y() + y, parent_transition.z() - 2/57.3);            
            Eigen::Vector3d child2_trans(parent_transition.x() + x, parent_transition.y() + y, parent_transition.z() + 2/57.3); 
            Eigen::Vector3d child3_trans(parent_transition.x() + x, parent_transition.y() + y, parent_transition.z()); 
            child_transitions.emplace_back(child1_trans);
            child_transitions.emplace_back(child2_trans);
            child_transitions.emplace_back(child3_trans);
        }
    }
    return child_transitions;
}

// 落脚点细化函数
// tested
vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> AstarHierarchicalFootstepPlanner::fineLandPoint(Eigen::Vector3d land_point)
{
    // 偏移量和偏移后的坐标
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> fine_points;
    for (double x = - 0.025; x <=  0.025; x = x + resolution)
    {
        for (double y = - 0.025; y <=  0.025; y = y + resolution)
        {
            Eigen::Vector3d child1_trans(land_point.x() + x, land_point.y() + y, land_point.z() - 2/57.3);
            Eigen::Vector3d offset1(x, y, -2/57.3); 
            Eigen::Vector3d child2_trans(land_point.x() + x, land_point.y() + y, land_point.z() + 2/57.3); 
            Eigen::Vector3d offset2(x, y, 2/57.3);
            Eigen::Vector3d child3_trans(land_point.x() + x, land_point.y() + y, land_point.z()); 
            Eigen::Vector3d offset3(x, y, 0);
            fine_points.emplace_back(std::make_pair(offset1, child1_trans));
            fine_points.emplace_back(std::make_pair(offset2, child2_trans)) ;
            fine_points.emplace_back(std::make_pair(offset3, child3_trans));
        }
    }

#ifdef  DEBUG
    // for (int i = 0; i < fine_points.size(); i++)
    // {
    //     LOG(INFO)<<i<<" "<<fine_points.at(i).first.transpose()<<"--"<<fine_points.at(i).second.transpose();
    // }
#endif

    return fine_points;
}



bool AstarHierarchicalFootstepPlanner::getPointsInForeFoot(Eigen::Vector3d ankle, HistogramVoting & fore_foot_HV)
{
    Eigen::AngleAxisd ax(ankle.z(), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d mid(ankle.x(), ankle.y(), 0);
    Eigen::Vector3d mid_top = ax.toRotationMatrix() * Eigen::Vector3d(footparam.x_upper, 0, 0) + mid;
    Eigen::Vector3d top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid_top;
    Eigen::Vector3d top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + mid_top;
    Eigen::Vector3d mid_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid;
    Eigen::Vector3d mid_right = ax.toRotationMatrix() * Eigen::Vector3d(0, -footparam.y_right, 0) + mid;
    return SqureHistogramVoting(top_left.head(2), top_right.head(2), mid_left.head(2), mid_right.head(2), fore_foot_HV);
}

bool AstarHierarchicalFootstepPlanner::SqureHistogramVoting(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, HistogramVoting & HV)
{
    if (label_localmap.isInside(TL) && label_localmap.isInside(TR) && label_localmap.isInside(BL) && label_localmap.isInside(BR))
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
                        LOG(INFO)<<label_localmap["label"](index_l.x(), index_l.y());
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
        return true;
    }
    else
    {
        return false;
    }
    
}

bool AstarHierarchicalFootstepPlanner::getPointsInHindFoot(Eigen::Vector3d ankle, HistogramVoting & hind_foot_HV)
{
    Eigen::AngleAxisd ax(ankle.z(), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d mid(ankle.x(), ankle.y(), 0);
    Eigen::Vector3d mid_down = ax.toRotationMatrix() * Eigen::Vector3d(- footparam.x_button, 0, 0) + mid;
    Eigen::Vector3d down_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid_down;
    Eigen::Vector3d down_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + mid_down;
    Eigen::Vector3d mid_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid;
    Eigen::Vector3d mid_right = ax.toRotationMatrix() * Eigen::Vector3d(0, -footparam.y_right, 0) + mid;

    return SqureHistogramVoting(mid_left.head(2), mid_right.head(2), down_left.head(2), down_right.head(2), hind_foot_HV);
}

bool AstarHierarchicalFootstepPlanner::getPointsInFootArea(Eigen::Vector3d ankle, HistogramVoting & fore_foot_HV, HistogramVoting & hind_foot_HV)
{
    Eigen::AngleAxisd ax(ankle.z(), Eigen::Vector3d::UnitZ());

    Eigen::Vector3d mid(ankle.x(), ankle.y(), 0);

    Eigen::Vector3d mid_top = ax.toRotationMatrix() * Eigen::Vector3d(footparam.x_upper, 0, 0) + mid;
    Eigen::Vector3d mid_down = ax.toRotationMatrix() * Eigen::Vector3d(- footparam.x_button, 0, 0) + mid;

    Eigen::Vector3d top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid_top;
    Eigen::Vector3d top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + mid_top;

    Eigen::Vector3d down_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid_down;
    Eigen::Vector3d down_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + mid_down;

    grid_map::Position top_left_l(top_left.x(), top_left.y());
    grid_map::Position top_right_l(top_right.x(), top_right.y());
    grid_map::Position down_left_l(down_left.x(), down_left.y());
    grid_map::Position down_right_l(down_right.x(), down_right.y());
    // points_planes.clear();
    // points_planes.resize(planes);
    if (label_localmap.isInside(top_left_l) && label_localmap.isInside(top_right_l) && label_localmap.isInside(down_left_l) && label_localmap.isInside(down_right_l))
    {
        return getPointsInForeFoot(ankle, fore_foot_HV) && getPointsInHindFoot(ankle, hind_foot_HV);
    }
    else
    {
#ifdef DEBUG
        LOG(ERROR)<<"corner is out of map";
#endif
        return false;
    }
}

bool AstarHierarchicalFootstepPlanner::getPointsInFootArea(Eigen::Vector3d ankle, IndexPlanePoints & index_plane)
{
    // LOG(INFO)<<",,,,l";
    Eigen::AngleAxisd ax(ankle.z(), Eigen::Vector3d::UnitZ());

    Eigen::Vector3d mid(ankle.x(), ankle.y(), 0);

    Eigen::Vector3d mid_top = ax.toRotationMatrix() * Eigen::Vector3d(footparam.x_upper, 0, 0) + mid;
    Eigen::Vector3d mid_down = ax.toRotationMatrix() * Eigen::Vector3d(- footparam.x_button, 0, 0) + mid;

    Eigen::Vector3d top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid_top;
    Eigen::Vector3d top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + mid_top;

    Eigen::Vector3d down_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid_down;
    Eigen::Vector3d down_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + mid_down;

    grid_map::Position top_left_l(top_left.x(), top_left.y());
    grid_map::Position top_right_l(top_right.x(), top_right.y());
    grid_map::Position down_left_l(down_left.x(), down_left.y());
    grid_map::Position down_right_l(down_right.x(), down_right.y());
    // points_planes.clear();
    // points_planes.resize(planes);
    if (label_localmap.isInside(top_left_l) && label_localmap.isInside(top_right_l) && label_localmap.isInside(down_left_l) && label_localmap.isInside(down_right_l))
    {
        // LOG(INFO)<<"ALL INSIDE";
        grid_map::LineIterator iterator_start(label_localmap, down_right_l, down_left_l);
        grid_map::LineIterator iterator_end(label_localmap, top_right_l, top_left_l);
        for (; !iterator_start.isPastEnd()&&!iterator_end.isPastEnd(); ++iterator_start, ++iterator_end)
        {
            grid_map::Index start_index(*iterator_start);
            grid_map::Index end_index(*iterator_end);
            for (grid_map::LineIterator iterator_l(label_localmap, start_index, end_index); !iterator_l.isPastEnd(); ++iterator_l)
            {
                const grid_map::Index index_l(*iterator_l);
                grid_map::Position position_l;
                if (label_localmap.getPosition(index_l, position_l))
                {
                    grid_map::Position3 cor_position;
                    if (label_localmap.getPosition3("elevation", index_l, cor_position))
                    {
                        if (!std::isnan(cor_position.z()))
                        {
                            if (!std::isnan(label_localmap["label"](index_l.x(), index_l.y())))
                            {
                                int label_index = static_cast<int>(label_localmap["label"](index_l.x(), index_l.y()));
                                index_plane.add(label_index, cor_position);
                            }
                            else
                            {
#ifdef DEBUG
                                LOG(INFO)<<"nan";
#endif
                            }
                        }
#ifdef DEBUG
                        else
                        {
#ifdef DEBUG
                            LOG(INFO)<<"cor nan";
#endif
                        }
#endif
                    }
#ifdef DEBUG
                    else
                    {
#ifdef DEBUG
                        LOG(INFO)<<"can not get cor";
#endif
                    }
#endif
                }
#ifdef DEBUG
                else
                {
#ifdef DEBUG
                    LOG(INFO)<<"can not get cor2";
#endif
                }
#endif
            }
        }
#ifdef DEBUG
        // for (auto & plane_points : points_planes)
        // {
        //     LOG(INFO)<<plane_points.size();
        // }
#endif
        return true;
    }
    else
    {
#ifdef DEBUG
        LOG(ERROR)<<"corner is out of map";
#endif
        return false;
    }
}

// tested backup
// bool AstarHierarchicalFootstepPlanner::getPointsInFootArea(Eigen::Vector3d ankle, vector<vector<Eigen::Vector3d>> & points_planes, vector<Eigen::Vector3d> & allpoints)
// {
//     // LOG(INFO)<<"function getPointsInFootArea";
//     // 计算四个点
//     Eigen::AngleAxisd ax(ankle.z(), Eigen::Vector3d::UnitZ());
//     Eigen::Vector3d mid(ankle.x(), ankle.y(), 0);
//     Eigen::Vector3d mid_top = ax.toRotationMatrix() * Eigen::Vector3d(footparam.x_upper, 0, 0) + mid;
//     Eigen::Vector3d mid_down = ax.toRotationMatrix() * Eigen::Vector3d(- footparam.x_button, 0, 0) + mid;
//     Eigen::Vector3d top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid_top;
//     Eigen::Vector3d top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + mid_top;
//     Eigen::Vector3d down_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid_down;
//     Eigen::Vector3d down_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + mid_down;
//     grid_map::Position top_left_l(top_left.x(), top_left.y());
//     grid_map::Position top_right_l(top_right.x(), top_right.y());
//     grid_map::Position down_left_l(down_left.x(), down_left.y());
//     grid_map::Position down_right_l(down_right.x(), down_right.y());
//     points_planes.clear();
//     points_planes.resize(planes);
//     if (label_localmap.isInside(top_left_l) && label_localmap.isInside(top_right_l) && label_localmap.isInside(down_left_l) && label_localmap.isInside(down_right_l))
//     {
//         grid_map::LineIterator iterator_start(label_localmap, down_right_l, down_left_l);
//         grid_map::LineIterator iterator_end(label_localmap, top_right_l, top_left_l);
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
//                             allpoints.emplace_back(cor_position);// 将所有点都添加
//                             grid_map::Position3 label3;
//                             if (label_localmap.getPosition3("label", index_l, label3))
//                             {
//                                 if (!std::isnan(label3.z()))
//                                 {
//                                     // LOG(INFO)<<"label3.z(): "<<label3.z();
//                                     points_planes.at(label3.z() -1).emplace_back(cor_position);
//                                 }
//                             }
//                         }
//                     }
//                 }
//             }
//         }
//         vector<vector<Eigen::Vector3d>> points;
//         for (auto & ps : points_planes)
//         {
//             if (ps.size() > 20)
//             {
//                 points.emplace_back(ps);
//             }
//         }
//         points_planes = points;
// #ifdef DEBUG
//         for (auto & plane_points : points_planes)
//         {
//             LOG(INFO)<<plane_points.size();
//         }
// #endif
//         return true;
//     }
//     else
//     {
// #ifdef DEBUG
//         LOG(INFO)<<"corner is out of map";
// #endif
//         return false;
//     }
// }

bool AstarHierarchicalFootstepPlanner::SqurePoints(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, vector<Eigen::Vector3d> & points)
{
    if (label_localmap.isInside(TL) && label_localmap.isInside(TR) && label_localmap.isInside(BL) && label_localmap.isInside(BR))
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
                grid_map::Position position_l;
                if (label_localmap.getPosition(index_l, position_l))
                {
                    grid_map::Position3 cor_position;
                    if (label_localmap.getPosition3("elevation", index_l, cor_position))
                    {
                        if (!std::isnan(cor_position.z()))
                        {
                            points.emplace_back(cor_position);
                        }
                    }
                }
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool AstarHierarchicalFootstepPlanner::computeLandInfo(Eigen::Vector3d ankle, int & max_size, int & above_points, Eigen::Vector3d & plane_normal, double & step_height, double & pitch, double & roll)
{
    checktime++;
    auto start = std::chrono::high_resolution_clock::now();
   
    max_size = 0;
    above_points = 0;
    plane_normal = Eigen::Vector3d::Zero();
    step_height = -std::numeric_limits<double>::infinity();
    pitch = std::numeric_limits<double>::infinity();
    roll = std::numeric_limits<double>::infinity();

    Eigen::AngleAxisd ax(ankle.z(), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d mid(ankle.x(), ankle.y(), 0);
    // 初次只用前后8cm的作为支撑平面的判断
    Eigen::Vector3d mid_top = ax.toRotationMatrix() * Eigen::Vector3d(0.08, 0, 0) + mid;
    Eigen::Vector3d mid_down = ax.toRotationMatrix() * Eigen::Vector3d(- 0.08, 0, 0) + mid;
    Eigen::Vector3d top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid_top;
    Eigen::Vector3d top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + mid_top;
    Eigen::Vector3d down_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid_down;
    Eigen::Vector3d down_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + mid_down;
    // grid_map::Position top_left_l(top_left.x(), top_left.y());
    // grid_map::Position top_right_l(top_right.x(), top_right.y());
    // grid_map::Position down_left_l(down_left.x(), down_left.y());
    // grid_map::Position down_right_l(down_right.x(), down_right.y());
    vector<Eigen::Vector3d> points;
    if (SqurePoints(top_left.head(2), top_right.head(2), down_left.head(2), down_right.head(2), points))
    {
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

                if (above_points > 8)
                {
                    // 结束计时
                    auto end = std::chrono::high_resolution_clock::now();
                    total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
                    return false;
                }
                if (max_size < (0.16/(0.11+0.15))* footsize_inmap)
                {
                    auto end = std::chrono::high_resolution_clock::now();
                    total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
                    return false;
                }
                

                Eigen::Vector3d eular;
                computeRollPitch(plane_normal, ankle.z(), eular);
                pitch = eular(1);
                roll = eular(2);
                double d = -center.dot(plane_normal);
                step_height = (-d - ankle.head(2).dot(plane_normal.head(2)))/plane_normal(2);
                // 结束计时
                auto end = std::chrono::high_resolution_clock::now();
                total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
                return true;
            }
            else
            {
                auto end = std::chrono::high_resolution_clock::now();
                total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
                return false;
            }
            
        }
        else
        {
            auto end = std::chrono::high_resolution_clock::now();
            total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
            return false;
        }
        
    }
    else
    {
        auto end = std::chrono::high_resolution_clock::now();
        total_time += (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0;
        return false;
    }
    
}


// tested 粗略检查
// bool AstarHierarchicalFootstepPlanner::computeLandInfo(Eigen::Vector3d ankle, int & max_size, int & above_points, Eigen::Vector3d & plane_normal, double & step_height, int & plane_index, double & pitch, double & roll)
// {
//     // 能否找到支撑平面
//         // 前脚直方图
//         // 后脚直方图
//         // 匹配对
//         // 支撑平面
//     // 论文中这个函数对应的可通行性检测，要记录这个函数被调用的次数及总时间消耗
//     checktime++;
//     clock_t start = clock();
//     max_size = 0;
//     above_points = 0;
//     plane_normal = Eigen::Vector3d::Zero();
//     step_height = -std::numeric_limits<double>::infinity();
//     plane_index = -1;
//     pitch = std::numeric_limits<double>::infinity();
//     roll = std::numeric_limits<double>::infinity();
//     HistogramVoting fore_foot_HV, hind_foot_HV;
//     if (getPointsInFootArea(ankle, fore_foot_HV, hind_foot_HV))
//     {
//         vector<std::pair<int, int>> candidate_support_planes;
//         for (auto & bin1 : fore_foot_HV.counter)
//         {
//             if (bin1.second.size() > 0.35 * footsize_inmap)
//             {
//                 for (auto & bin2 : hind_foot_HV.counter)
//                 {
//                     if (bin1.first == bin2.first)
//                     {
//                         if (bin2.second.size() > 0.25 * footsize_inmap)
//                         {
//                             // 存在一个匹配对
//                             candidate_support_planes.emplace_back(std::make_pair(bin1.first, bin1.second.size() + bin2.second.size()));
//                         }
//                     }
//                 }
//             }
//         }
// #ifdef DEBUG
//         for (int i = 0; i < candidate_support_planes.size(); i++)
//         {
//             LOG(INFO)<<i<<": "<<candidate_support_planes.at(i).first<<" "<<candidate_support_planes.at(i).second;
//         }     
// #endif
//         if (!candidate_support_planes.empty())
//         {
//             Eigen::Vector3d center;
//             double max_height = -std::numeric_limits<double>::infinity();
//             for (auto & cand_plane : candidate_support_planes)
//             {
//                 double tmp_elevation = planes_info.at(cand_plane.first).getZ(ankle.head(2));
//                 // LOG(INFO)<<"tmp_elevation: "<<tmp_elevation;
//                 if (max_height < tmp_elevation)
//                 {
//                     // LOG(INFO)<<"RESET...";
//                     plane_index = cand_plane.first;
//                     max_size = cand_plane.second;
//                     plane_normal = Eigen::Vector3d(planes_info.at(cand_plane.first).normal.x(), planes_info.at(cand_plane.first).normal.y(), planes_info.at(cand_plane.first).normal.z());
//                     center = Eigen::Vector3d(planes_info.at(cand_plane.first).center.x(), planes_info.at(cand_plane.first).center.y(), planes_info.at(cand_plane.first).center.z());
//                     max_height = tmp_elevation;
//                 }
//             }
// #ifdef DEBUG
//             LOG(INFO)<<"plane_normal: "<<plane_normal.transpose();
//             LOG(INFO)<<"center: "<<center.transpose();
// #endif
//             step_height = max_height;
//             Eigen::Vector3d eular;
//             computeRollPitch(plane_normal, ankle.z(), eular);
//             pitch = eular(1);
//             roll = eular(2);
//             for (auto & points : fore_foot_HV.counter)
//             {
//                 for (auto & point : points.second)
//                 {
//                     if ((point - center).dot(plane_normal) > 0.01)
//                     {
//                         above_points ++;
//                     }
//                 }
//             }
//             for (auto & points : hind_foot_HV.counter)
//             {
//                 for (auto & point : points.second)
//                 {
//                     if ((point - center).dot(plane_normal) > 0.01)
//                     {
//                         above_points ++;
//                     }
//                 }
//             }                
//             if (above_points > 8)
//             {
// #ifdef DEBUG
//                 LOG(INFO)<<"too much above points";
// #endif
//                 clock_t end = clock();
//                 total_time += double(end - start) / CLOCKS_PER_SEC * 1000;
//                 return false;
//             }
//             else
//             {
//                 clock_t end = clock();
//                 total_time += double(end - start) / CLOCKS_PER_SEC * 1000;
//                 return true;
//             }                     
//         }
//         else
//         {
// #ifdef DEBUG
//             LOG(INFO)<<"can not get support plane";
// #endif
//             clock_t end = clock();
//             total_time += double(end - start) / CLOCKS_PER_SEC * 1000;
//             return false;
//         }
//     }
//     else
//     {
// #ifdef DEBUG
//         LOG(ERROR)<<"can not get area";
// #endif
//         clock_t end = clock();
//         total_time += double(end - start) / CLOCKS_PER_SEC * 1000;
//         return false;
//     }
// }

// tested
vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> AstarHierarchicalFootstepPlanner::fineTransitions(Eigen::Vector3d point, FootstepNodePtr current_node)
{
    vector<Eigen::Vector3d> fineBasicTransitions = fineTransitionsBasic(point);
    if (current_node->footstep.robot_side == 0) // 支撑脚为左脚
    {
        vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> left_transitions;
        for (auto & transition : fineBasicTransitions)
        {
            auto left_transition = Eigen::Vector3d(transition.x(), -transition.y(), -transition.z());
            Eigen::AngleAxisd ad(current_node->footstep.yaw, Eigen::Vector3d::UnitZ());
            Eigen::Vector3d ts(left_transition.x(), left_transition.y(), 0);
            Eigen::Vector3d ts_t = ad.toRotationMatrix() * ts + Eigen::Vector3d(current_node->footstep.x, current_node->footstep.y, 0);
            left_transitions.emplace_back(std::make_pair(transition, Eigen::Vector3d(ts_t.x(), ts_t.y(), current_node->footstep.yaw + left_transition.z())));
        }
        return left_transitions;
    }
    else // 支撑脚为右脚
    {
        vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> right_transitions;
        for (auto & tansition : fineBasicTransitions)
        {
            Eigen::AngleAxisd ad(current_node->footstep.yaw, Eigen::Vector3d::UnitZ());
            Eigen::Vector3d ts(tansition.x(), tansition.y(), 0);
            Eigen::Vector3d ts_t = ad.toRotationMatrix() * ts + Eigen::Vector3d(current_node->footstep.x, current_node->footstep.y, 0);
            right_transitions.emplace_back(std::make_pair(tansition, Eigen::Vector3d(ts_t.x(), ts_t.y(), current_node->footstep.yaw + tansition.z())));
        }
        return right_transitions;
    }
}

// tested
vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> AstarHierarchicalFootstepPlanner::basicTransitions(FootstepNodePtr current_node)
{
    // 根据当前两步不属于统一平面，就执行并步
    if (current_node->footstep.robot_side == 0) // 支撑脚为左脚
    {
        vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> left_transitions;
        if (current_node->plane_index == current_node->PreFootstepNode->plane_index)
        {
            for (auto & transition : transitions)
            {
                auto left_transition = Eigen::Vector3d(transition.x(), -transition.y(), -transition.z());
                Eigen::AngleAxisd ad(current_node->footstep.yaw, Eigen::Vector3d::UnitZ());
                Eigen::Vector3d ts(left_transition.x(), left_transition.y(), 0);
                Eigen::Vector3d ts_t = ad.toRotationMatrix() * ts + Eigen::Vector3d(current_node->footstep.x, current_node->footstep.y, 0);
                left_transitions.emplace_back(std::make_pair(transition, Eigen::Vector3d(ts_t.x(), ts_t.y(), current_node->footstep.yaw + left_transition.z())));
            }
        }
        else
        {
            // cout<<"using combine_transitions"<<endl;
            for (auto & transition : combine_transitions)
            {
                auto left_transition = Eigen::Vector3d(transition.x(), -transition.y(), -transition.z());
                Eigen::AngleAxisd ad(current_node->footstep.yaw, Eigen::Vector3d::UnitZ());
                Eigen::Vector3d ts(left_transition.x(), left_transition.y(), 0);
                Eigen::Vector3d ts_t = ad.toRotationMatrix() * ts + Eigen::Vector3d(current_node->footstep.x, current_node->footstep.y, 0);
                left_transitions.emplace_back(std::make_pair(transition, Eigen::Vector3d(ts_t.x(), ts_t.y(), current_node->footstep.yaw + left_transition.z())));
            }
        }
        return left_transitions;
    }
    else // 支撑脚为右脚
    {
        vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> right_transitions;
        if (current_node->plane_index == current_node->PreFootstepNode->plane_index)
        {
            for (auto & tansition : transitions)
            {
                Eigen::AngleAxisd ad(current_node->footstep.yaw, Eigen::Vector3d::UnitZ());
                Eigen::Vector3d ts(tansition.x(), tansition.y(), 0);
                Eigen::Vector3d ts_t = ad.toRotationMatrix() * ts + Eigen::Vector3d(current_node->footstep.x, current_node->footstep.y, 0);
                right_transitions.emplace_back(std::make_pair(tansition, Eigen::Vector3d(ts_t.x(), ts_t.y(), current_node->footstep.yaw + tansition.z())));
            }
        }
        else
        {
            // cout<<"using combine_transitions"<<endl;
            for (auto & tansition : combine_transitions)
            {
                Eigen::AngleAxisd ad(current_node->footstep.yaw, Eigen::Vector3d::UnitZ());
                Eigen::Vector3d ts(tansition.x(), tansition.y(), 0);
                Eigen::Vector3d ts_t = ad.toRotationMatrix() * ts + Eigen::Vector3d(current_node->footstep.x, current_node->footstep.y, 0);
                right_transitions.emplace_back(std::make_pair(tansition, Eigen::Vector3d(ts_t.x(), ts_t.y(), current_node->footstep.yaw + tansition.z())));
            }
        }
        return right_transitions;
    }
}

// tested 构造函数时
// void AstarHierarchicalFootstepPlanner::computePlanarInfor()
// {
//     pcl::PointCloud<pcl::PointXYZ> pc = gridMap2Pointcloud();
// #ifdef DEBUG
//     // pcl::io::savePCDFile("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/data/map.pcd", pc);
//     pcl::io::savePCDFile("/home/bhr/catkin_beijing4th/src/elevation_mapping/AstarHierarchicalFootstepPlanner/data/map.pcd", pc);
// #endif

//     AHFP::PlanarContourExtraction pce(pc);
//     pce.run();
//     vector<cv::Mat> seg_planes = pce.getSegPlanes();
//     plane_images = seg_planes;
// #ifdef DEBUG
//     LOG(INFO)<<"plane size: "<<seg_planes.size();
//     for (int i = 0; i < seg_planes.size(); i++)
//     {
//         cv::imshow("seg_image", seg_planes.at(i));
//         // cv::waitKey(0);
//         // cv::imwrite("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/data/" + std::to_string(i) + ".jpg", seg_planes.at(i));
//         cv::imwrite("/home/bhr/catkin_beijing4th/src/elevation_mapping/AstarHierarchicalFootstepPlanner/data/" + std::to_string(i) + ".jpg", seg_planes.at(i));
//     }
//     // 测试时，优化起点终点的位置
//     // grid_map::Index start_index, start_pre_index, left_goal_index, gight_goal_index;
// #endif
//     planes = seg_planes.size();
//     label_localmap = localmap;
//     label_localmap.add("label");
//     // LOG(INFO)<<label_localmap.getSize().transpose();
//     // LOG(INFO)<<seg_planes.at(0).rows<<" "<<seg_planes.at(0).cols;
//     // LOG(INFO)<<seg_planes.at(0).size();
//     plane_image = cv::Mat::zeros(label_localmap.getSize().x(), label_localmap.getSize().y(), CV_8UC3);
//     for (int i = 0; i < label_localmap.getSize().x(); i++)
//     {
//         for (int j = 0; j < label_localmap.getSize().y(); j++)
//         {
//             bool flag = false;
//             for (int label = 1; label <= seg_planes.size(); label++)
//             {
//                 if (seg_planes.at(label - 1).at<uchar>(i, j) == 255)
//                 {
//                     flag = true;
//                     label_localmap["label"](i ,j) = label;
//                     // 后续计算
//                     plane_image.at<cv::Vec3b>(i, j) = cv::Vec3b(default_colors[int(label)][0], default_colors[int(label)][1], default_colors[int(label)][2]);
// #ifdef DEBUG
//                     plane_image.at<cv::Vec3b>(i, j) = cv::Vec3b(default_colors[int(label)][0], default_colors[int(label)][1], default_colors[int(label)][2]);
// #endif
//                     break;
//                 }
//             }
//             if (!flag)
//             {
//                 label_localmap["label"](i,j) = NAN;
//             }
//         }
//     }
// #ifdef DEBUG
//     cv::imshow("plane image", plane_image);
//     // cv::imwrite("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/data/planes.jpg", plane_image);
//     cv::imwrite("/home/bhr/catkin_beijing4th/src/elevation_mapping/AstarHierarchicalFootstepPlanner/data/planes.jpg", plane_image);
//     pcl::PointCloud<pcl::PointXYZ> planepoints;
//     for (int i = 0; i < plane_image.rows; i++)
//     {
//         for (int j = 0; j < plane_image.cols; j++)
//         {
//             if (plane_image.at<cv::Vec3b>(i, j) != cv::Vec3b(0, 0, 0))
//             {
//                 grid_map::Position3 p3;
//                 if (localmap.getPosition3("elevation", grid_map::Index(i, j), p3))
//                 {
//                     planepoints.emplace_back(pcl::PointXYZ(p3.x(), p3.y(), p3.z()));
//                 }
//             }
//         }
//     }
//     pcl::io::savePCDFile("/home/bhr/catkin_beijing4th/src/elevation_mapping/AstarHierarchicalFootstepPlanner/data/planepoints.pcd", planepoints);
//     // pcl::io::savePCDFile("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/data/planepoints.pcd", planepoints);
//     // cv::waitKey(0);
// #endif
// }

// tested
pcl::PointCloud<pcl::PointXYZ> AstarHierarchicalFootstepPlanner::gridMap2Pointcloud()
{
    pcl::PointCloud<pcl::PointXYZ> pc;
    for (int i = 0; i < localmap.getSize().x(); i++)
    {
        for (int j = 0; j < localmap.getSize().y(); j++)
        {
            grid_map::Index index(i, j);
            grid_map::Position3 p3;
            if (localmap.getPosition3("elevation", index, p3))
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
    pc.width = localmap.getSize().y();
    pc.height = localmap.getSize().x();
    return pc;
}


// 权重应该较大
// tested
bool AstarHierarchicalFootstepPlanner::computeHcost(FootstepNodePtr node, double & hcost)
{
    grid_map::Position p2(node->footstep.x, node->footstep.y);
    if (localmap.isInside(p2))
    {
        double dis, angle_diff, dis_z;
        if (node->footstep.robot_side == LEFT)
        {
            Eigen::AngleAxisd ad1(end_left_p->footstep.yaw, Eigen::Vector3d::UnitZ());
            Eigen::Vector2d direct1 = (ad1.toRotationMatrix() * Eigen::Vector3d::UnitX()).head(2);
            Eigen::AngleAxisd ad2(end_left_p->footstep.yaw + 3.14159/2, Eigen::Vector3d::UnitZ());
            Eigen::Vector2d direct2 = (ad2.toRotationMatrix() * Eigen::Vector3d::UnitX()).head(2);
            Eigen::Vector2d v_t = Eigen::Vector2d(end_left_p->footstep.x, end_left_p->footstep.y) - Eigen::Vector2d(node->footstep.x, node->footstep.y);
            dis = v_t.norm();
            // dis1 = abs(v_t.dot(direct1));
            // dis2 = abs(v_t.dot(direct2));
            dis_z = abs(node->footstep.z - end_left_p->footstep.z);
            // LOG(INFO)<<v_t.transpose();
            // LOG(INFO)<<direct1.transpose();
            // LOG(INFO)<<direct2.transpose();
            angle_diff = abs(node->footstep.yaw - end_left_p->footstep.yaw);
        }
        else
        {
            Eigen::AngleAxisd ad1(end_right_p->footstep.yaw, Eigen::Vector3d::UnitZ());
            Eigen::Vector2d direct1 = (ad1.toRotationMatrix() * Eigen::Vector3d::UnitX()).head(2);
            Eigen::AngleAxisd ad2(end_right_p->footstep.yaw + 3.14159/2, Eigen::Vector3d::UnitZ());
            Eigen::Vector2d direct2 = (ad2.toRotationMatrix() * Eigen::Vector3d::UnitX()).head(2);
            Eigen::Vector2d v_t = Eigen::Vector2d(end_right_p->footstep.x, end_right_p->footstep.y) - Eigen::Vector2d(node->footstep.x, node->footstep.y);
            dis = v_t.norm();
            // dis1 = abs(v_t.dot(direct1));
            // dis2 = abs(v_t.dot(direct2));
            dis_z = abs(node->footstep.z - end_right_p->footstep.z);
            // LOG(INFO)<<v_t.transpose();
            // LOG(INFO)<<direct1.transpose();
            // LOG(INFO)<<direct2.transpose();
            angle_diff = abs(node->footstep.yaw - end_right_p->footstep.yaw);
        }
        // LOG(INFO)<<dis1<<" "<<dis2<<" "<<angle_diff;
        hcost = ((dis)*4 + dis_z * 2 + angle_diff * 0.3);
        return true;
    }
    else
    {
        return false;
    }
}

bool AstarHierarchicalFootstepPlanner::computeGcost(FootstepNodePtr node, double & gcost)
{
    double dis, angle_diff, height_diff;
    dis = (Eigen::Vector2d(node->footstep.x, node->footstep.y) - Eigen::Vector2d(node->PreFootstepNode->footstep.x, node->PreFootstepNode->footstep.y)).norm();
    angle_diff = abs(node->footstep.yaw - node->PreFootstepNode->footstep.yaw);
    height_diff = abs(node->footstep.z - node->PreFootstepNode->footstep.z);
    gcost = dis  /* angle_diff * 0.5 /*+ height_diff */ + node->PreFootstepNode->Gcost;
    return true;
}

// 先结算出正常状态下机器人的双脚位置，但在局部规划时并不鲁棒，因为你需要考虑双脚在此状态下能不能在此位置落脚
// tested
bool AstarHierarchicalFootstepPlanner::computerLeftRightGoal(Eigen::Vector3d goal)
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
        Eigen::Vector3d left_opt;
        for (auto & cand : left_cands)
        {
            double tmpscore = -std::numeric_limits<double>::infinity();
            double height = -std::numeric_limits<double>::infinity();
            double pitch = std::numeric_limits<double>::infinity();
            double roll = std::numeric_limits<double>::infinity();
            if (computeLandPointScore(cand, tmpscore, height, pitch, roll))
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
                }
            }
        }
        if (opt_height != -std::numeric_limits<double>::infinity())
        {
            end_left_p = std::make_shared<FootstepNode>(left_opt, opt_height, opt_roll, opt_pitch, 0);
        }
        else
        {
            return false;
        }
        
        score = - std::numeric_limits<double>::infinity();
        opt_height = -std::numeric_limits<double>::infinity();
        opt_pitch = -std::numeric_limits<double>::infinity();
        opt_roll = -std::numeric_limits<double>::infinity();
        Eigen::Vector3d right_opt;
        for (auto & cand : right_cands)
        {
            double tmpscore;
            double height = -std::numeric_limits<double>::infinity();
            double pitch = std::numeric_limits<double>::infinity();
            double roll = std::numeric_limits<double>::infinity();
            if (computeLandPointScore(cand, tmpscore, height, pitch, roll))
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
                }
            }
        }
        if (opt_height != -std::numeric_limits<double>::infinity())
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

// tested
bool AstarHierarchicalFootstepPlanner::getNodeString(FootstepNodePtr node, string & s)
{
    s.clear();
    auto iter = node;
    while (iter != start_p && iter)
    {
        grid_map::Index index;
        if (localmap.getIndex(grid_map::Position(iter->footstep.x, iter->footstep.y), index))
        {
            if (iter->footstep.robot_side == 0)
            {
                s += std::to_string(index.x()) + std::to_string(index.y()) + std::to_string(int(iter->footstep.yaw * 57.3)) + "L";
            }
            else
            {
                s += std::to_string(index.x()) + std::to_string(index.y()) + std::to_string(int(iter->footstep.yaw * 57.3)) + "R";
            }
            iter = iter->PreFootstepNode;
        }
        else
        {
            return false;
        }   
    }
    return true;
}

// tested
bool AstarHierarchicalFootstepPlanner::arriveGoal(FootstepNodePtr node)
{
    double dis, angle_diff;
    if (node->footstep.robot_side == 0)
    {
        dis = (Eigen::Vector2d(end_left_p->footstep.x, end_left_p->footstep.y) - Eigen::Vector2d(node->footstep.x, node->footstep.y)).norm();
        angle_diff = abs(end_left_p->footstep.yaw - node->footstep.yaw);
    }
    else
    {
        dis = (Eigen::Vector2d(end_right_p->footstep.x, end_right_p->footstep.y) - Eigen::Vector2d(node->footstep.x, node->footstep.y)).norm();
        angle_diff = abs(end_right_p->footstep.yaw - node->footstep.yaw);
    }
    // LOG(INFO)<<dis<<" "<<angle_diff;
    if (dis < 0.1 && angle_diff <= 5/57.3)
    {
        return true;
    }
    else
    {
        // LOG(INFO)<<"NOT ARR";
        return false;
    }
}


// 计算终点
cv::Point calculateEndPoint(cv::Point start, double length, double angle) 
{
    int end_x = static_cast<int>(start.x + length * cos(angle));
    int end_y = static_cast<int>(start.y + length * sin(angle));
    return cv::Point(end_x, end_y);
}

bool AstarHierarchicalFootstepPlanner::plan()
{
    // LOG(INFO)<<"ENTER PLAN";
    p_queue.push(start_p);
    while (!p_queue.empty())
    {
        auto current_node = p_queue.top();
#ifdef DEBUG
        outfile<<current_node->footstep.x<<" "<<current_node->footstep.y<<" "<<current_node->footstep.z<<" "<<current_node->footstep.roll<<" "<<current_node->footstep.pitch<<" "<<current_node->footstep.yaw<<" "<<current_node->footstep.robot_side<<endl;
        // if (abs(current_node->footstep.yaw) > 5/57.3)
        // {
        //     auto debug_q = p_queue;
        //     while (!debug_q.empty())
        //     {
        //         auto debug_node = debug_q.top();
        //         debug_q.pop();
        //         LOG(INFO)<<debug_node->footstep.x<<" "<<debug_node->footstep.y<<" "<<debug_node->footstep.yaw<<" "<<debug_node->footstep.robot_side;
        //     }
        // }
#endif
        p_queue.pop();
        string ss;
        if (getNodeString(current_node, ss))
        {
            if (close_set.find(ss) == close_set.end())
            {
                close_set.insert(ss);
            }
            else
            {
                continue;
            }
        }
        else
        {
            continue;
        }
        
        if (arriveGoal(current_node))
        {
            LOG(INFO)<<"arr: "<<current_node->footstep.x<<" "<<current_node->footstep.y<<" "<<current_node->footstep.z<<" "<<current_node->footstep.roll<<" "<<current_node->footstep.pitch<<" "<<current_node->footstep.yaw<<" "<<current_node->footstep.robot_side;
            
            if (getFootsteps(current_node))
            {
                LOG(INFO)<<"times: "<<checktime<<" "<<total_time<<" "<<steps.size();
                return true;
            }
            else
            {
#ifdef DEBUG
                LOG(ERROR)<<"some error ...";
#endif
                return false;
            }
            
        }
        else
        {
            vector<FootstepNodePtr> child_nodes;
            if (nodeExtension(current_node, current_node->PreFootstepNode, child_nodes))
            {
                // LOG(INFO)<<child_nodes.size();
#ifdef DEBUG
                // cv::Mat tmp_image = plane_image.clone();
                cv::Mat tmp_image = cv::Mat::zeros(plane_image.size(), CV_8UC3);
                LOG(INFO)<<"current node: "<<current_node->footstep.x<<" "<<current_node->footstep.y<<" "<<current_node->footstep.yaw<<" "<<current_node->footstep.robot_side;
                grid_map::Index index;
                if (localmap.getIndex(grid_map::Position(current_node->footstep.x, current_node->footstep.y), index))
                {
                    cv::Point end_point = calculateEndPoint(cv::Point(index.y(), index.x()), 6, current_node->footstep.yaw - CV_PI/2);
                    cout<<"start_point: "<<index.y()<<" "<<index.x()<<endl;

                    cout<<"end_point: "<<end_point.y<<" "<<end_point.x<<endl;

                    if (current_node->footstep.robot_side == 0)
                    {
                        // cv::circle(tmp_image, cv::Point(index.y(), index.x()), 3, cv::Scalar(0, 0, 255), 2);
                        cv::arrowedLine(tmp_image, cv::Point(index.y(), index.x()), end_point, cv::Scalar(255, 255, 255), 1);
                    }
                    else
                    {
                        // cv::circle(tmp_image, cv::Point(index.y(), index.x()), 3, cv::Scalar(0, 255, 0), 2);
                        cv::arrowedLine(tmp_image, cv::Point(index.y(), index.x()), end_point, cv::Scalar(0, 255, 0), 1);
                    }
                }

                for (auto & child : child_nodes)
                {
                    // LOG(INFO)<<child->footstep.x<<" "<<child->footstep.y<<" "<<child->footstep.yaw<<" "<<child->footstep.robot_side;
                    grid_map::Index child_index;
                    if (localmap.getIndex(grid_map::Position(child->footstep.x, child->footstep.y), child_index))
                    {
                        tmp_image.at<cv::Vec3b>(child_index.x(), child_index.y()) = cv::Vec3b(0, 0, 255);
                    }
                }

                grid_map::Index left_index, right_index;
                if (localmap.getIndex(grid_map::Position(end_left_p->footstep.x, end_left_p->footstep.y), left_index) && localmap.getIndex(grid_map::Position(end_right_p->footstep.x, end_right_p->footstep.y), right_index))
                {
                    cv::circle(tmp_image, cv::Point(left_index.y(), left_index.x()), 3, cv::Scalar(255, 255, 255), 2);
                    cv::circle(tmp_image, cv::Point(right_index.y(), right_index.x()), 3, cv::Scalar(255, 255, 255), 2);
                }
                // cv::imshow("tmp_image", tmp_image);
                double scaleFactor = 3.0;

                // 定义放大后的图像尺寸
                cv::Size newSize(static_cast<int>(tmp_image.cols * scaleFactor), static_cast<int>(tmp_image.rows * scaleFactor));

                // 放大图像
                cv::Mat enlargedImage;
                cv::resize(tmp_image, enlargedImage, newSize);
                cv::imshow("Enlarged Image", enlargedImage);
                // cout<<"..."<<endl;
                cv::waitKey(0);
#endif
                for (auto & p_node : child_nodes)
                {
                    string s_tmp;
                    if (getNodeString(p_node, s_tmp))
                    {
                        // p_queue.push(p_node);
                        if (close_set.find(s_tmp) == close_set.end())
                        {
                            p_queue.push(p_node);
                        }
                    }
                }
            }
        }
    }
}

// 检查文件是否存在
bool fileExists(const string& filename) {
    ifstream file(filename);
    return file.good();
}

void saveImageWithAutoIncrement(const cv::Mat& image, const string& filename) {
    string baseName = filename;
    string extension = ".jpg";  // 可以根据需要修改文件扩展名

    int counter = 0;
    string newFilename;

    do {
        if (counter == 0) {
            newFilename = baseName + extension;
        } else {
            // 根据计数器添加副本编号
            stringstream ss;
            ss << baseName << "_" << setfill('0') << setw(2) << counter << extension;
            newFilename = ss.str();
        }

        counter++;
    } while (fileExists(newFilename));  // 检查文件是否已存在

    // 保存图像到新的文件名
    cv::imwrite(newFilename, image);

    cout << "Image saved as: " << newFilename << endl;
}

bool AstarHierarchicalFootstepPlanner::getFootsteps(FootstepNodePtr node)
{
    auto iter_P = node;
    while (iter_P)
    {
        LOG(INFO)<<iter_P->footstep.x<<" "<<iter_P->footstep.y<<" "<<iter_P->footstep.z<<" "<<iter_P->footstep.roll<<" "<<iter_P->footstep.pitch<<" "<<iter_P->footstep.yaw<<" "<<iter_P->footstep.robot_side;
        Footstep foot_step = iter_P->footstep;
        steps.emplace_back(foot_step);
        if (iter_P == start_p)
        {
            steps.pop_back();// 不要起点

            // 这种情况表示机器人下一个落脚即到达了终点，此时不再进行规划。
            if (steps.empty())
            {
                LOG(INFO)<<"just need repair a step";
                // 仅仅需要补充并步
                Footstep step;
                if (repairStanceStep(start_p->footstep, step))
                {
                    steps.emplace_back(step);
                }
                return true;
            }
            
            std::reverse(steps.begin(), steps.end());
            Footstep step;
            if (repairStanceStep(steps.back(), step))
            {
                LOG(INFO)<<step.x<<" "<<step.y<<" "<<step.z<<" "<<step.roll<<" "<<step.pitch<<" "<<step.yaw<<" "<<step.robot_side;
                steps.emplace_back(step);
                // 这是修正的量，需要删除
                // steps.at(steps.size() - 2).z += 0.01;
                // steps.at(steps.size() - 1).z += 0.01;
                // for (int i = 0; i < steps.size(); i++)
                // {
                //     // 对踏上第二阶台阶之后，2步之后的步态点全部+0.01
                //     if (steps.at(i).z > 0.17)
                //     {
                //         for (int j = i; j < steps.size(); j++)
                //         {
                //             steps.at(j).z += 0.01;
                //         }
                //         break;
                //     }
                // }
                
                
                // 保存规划的落脚点，并查看
                grid_map::Index start_index, pre_start_index;
                if (localmap.getIndex(Eigen::Vector2d(start_p->footstep.x, start_p->footstep.y), start_index) && localmap.getIndex(Eigen::Vector2d(prestart_p->footstep.x, prestart_p->footstep.y), pre_start_index))
                {
                    plane_image.at<cv::Vec3b>(start_index.x(), start_index.y()) = cv::Vec3b(0, 255, 0);
                    plane_image.at<cv::Vec3b>(pre_start_index.x(), pre_start_index.y()) = cv::Vec3b(0, 255, 0);
                }
                for (auto & s : steps)
                {
                    grid_map::Index s_index;
                    if (localmap.getIndex(grid_map::Position(s.x, s.y), s_index))
                    {
                        plane_image.at<cv::Vec3b>(s_index.x(), s_index.y()) = cv::Vec3b(0, 0, 255);
                    }
                }
                // cv::imwrite("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/data/plan_result/reslut.jpg", plane_image);
                
                // saveImageWithAutoIncrement(plane_image, "/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/data/plan_result/");
                return true;
            }
            else
            {
                LOG(INFO)<<"can not repair a foot";
                return false;
            }
        }
        iter_P = iter_P->PreFootstepNode;
    }
    return false;
}

bool AstarHierarchicalFootstepPlanner::repairStanceStep(Footstep current_step, Footstep & footstep)
{
    Eigen::AngleAxisd ad(current_step.yaw, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d v_t(0, current_step.robot_side == 0 ? (-hip_width):hip_width, 0);
    Eigen::Vector3d current(current_step.x, current_step.y, current_step.z);
    Eigen::Vector3d repair = ad.toRotationMatrix() * v_t + current;
    repair.z() = current_step.yaw;
    int max_size = 0; // 属于同一最大平面的点数
    int above_points = 0; // 超出限制部分的点数
    Eigen::Vector3d plane_normal = Eigen::Vector3d::Zero();
    double height = -std::numeric_limits<double>::infinity();
    double roll = std::numeric_limits<double>::infinity();
    double pitch = std::numeric_limits<double>::infinity();
    if (!computeLandInfo(repair, max_size, above_points, plane_normal, height, pitch, roll))
    {
        return false;
    }
    if (above_points == 0)
    {
        footstep = Footstep(repair, height, roll, pitch, current_step.getInverseRobotSide());
        return true;
    }
    else
    {
        return false;
    }
}

// untested
// 这里只能将平面分割完好的场景能找到，对于不能完全分割的场景并不行
vector<vector<Eigen::Vector3d>> AstarHierarchicalFootstepPlanner::computeAvoidPoints()
{
    // 计算平面与摆动脚连线的交点，并从交点中选取障碍点
    auto tmpsteps = steps;
    tmpsteps.insert(tmpsteps.begin(), start_p->footstep);
    tmpsteps.insert(tmpsteps.begin(), prestart_p->footstep);
    vector<vector<Eigen::Vector3d>> all_avoid_points;
    for (int i = 2; i < tmpsteps.size(); i++)
    {
        cv::Vec3b cuurent_color;
        grid_map::Position swing_start(tmpsteps.at(i-2).x, tmpsteps.at(i-2).y);
        grid_map::Position swing_end(tmpsteps.at(i).x, tmpsteps.at(i).y);

        vector<Eigen::Vector3d> avoid_points;
        avoid_points.emplace_back(Eigen::Vector3d(tmpsteps.at(i-2).x, tmpsteps.at(i-2).y, tmpsteps.at(i-2).z));
        grid_map::LineIterator iter(localmap, swing_start, swing_end);
        while (!iter.isPastEnd())
        {
            grid_map::Index iter_index(*iter);
            ++iter;
            // 保证下一个需要使用的点不超出范围
            if (iter.isPastEnd())
            {
                break;
            }
            grid_map::Index iter_index_plus(*iter);
            for (auto & plane_image_8 : plane_images)
            {
                if (plane_image_8.at<uchar>(iter_index.x(), iter_index.y()) != plane_image_8.at<uchar>(iter_index_plus.x(), iter_index_plus.y()))
                {
                    if (plane_image_8.at<uchar>(iter_index.x(), iter_index.y()) == 255)
                    {
                        grid_map::Position3 p3;
                        if (localmap.getPosition3("elevation", iter_index, p3))
                        {
                            avoid_points.emplace_back(p3);
                        }
                    }
                    else
                    {
                        grid_map::Position3 p3;
                        if (localmap.getPosition3("elevation", iter_index_plus, p3))
                        {
                            avoid_points.emplace_back(p3);
                        }
                    }
                }
            }
        }
        avoid_points.emplace_back(tmpsteps.at(i).x, tmpsteps.at(i).y, tmpsteps.at(i).z);
        
        auto tmp_points = avoid_points;
        avoid_points.clear();
        for (int i = 1; i < tmp_points.size() - 1; i++)
        {
            Eigen::Vector3d convex_point1, convex_point2;
            if (getConvexPoint(tmp_points.at(i - 1), tmp_points.at(i), convex_point1))
            {
                avoid_points.emplace_back(convex_point1);
            }
            double dis_two = (tmp_points.at(i - 1) - tmp_points.at(i + 1)).head(2).norm();
            double dis_mid = (tmp_points.at(i - 1) - tmp_points.at(i)).head(2).norm();
            double height_mid = (tmp_points.at(i + 1) - tmp_points.at(i - 1)).z()*(dis_mid / dis_two) + tmp_points.at(i - 1).z();
            if (height_mid + 0.03 < tmp_points.at(i).z())
            {
                avoid_points.emplace_back(tmp_points.at(i));
            }
            if (getConvexPoint(tmp_points.at(i), tmp_points.at(i + 1), convex_point2))
            {
                avoid_points.emplace_back(convex_point2);
            }
            
        }
        all_avoid_points.emplace_back(avoid_points);
    }
    CHECK(all_avoid_points.size() == steps.size());
    return all_avoid_points;
}

bool AstarHierarchicalFootstepPlanner::getConvexPoint(Eigen::Vector3d start, Eigen::Vector3d end, Eigen::Vector3d convex_point)
{
    double height = max(start.z(), end.z());
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    int size = 0;
    for (grid_map::LineIterator iter(localmap, grid_map::Position(start.x(), start.y()), grid_map::Position(end.x(), end.y())); !iter.isPastEnd(); ++iter)
    {
        grid_map::Position3 p3;
        if (localmap.getPosition3("elevation", *iter, p3))
        {
            if (p3.z() - height > 0.04)// 4cm 也是一个经验阈值
            {
                sum += p3;
                size ++;
            }
        }
    }
    if (size == 0)
    {
        return false;
    }
    else
    {
        convex_point = sum / sum.size();
        return true;
    }
}

bool AstarHierarchicalFootstepPlanner::checkFootstepsResult()
{
#ifdef DEBUG
    for (auto & step : steps)
    {
        cout<<"******************"<<endl;
        LOG(INFO)<<step.robot_side<<" "<<step.x<<" "<<step.y<<" "<<step.z<<" "<<step.roll<<" "<<step.pitch<<" "<<step.yaw;
        int above_points = 0;
        int max_size = 0;
        Eigen::Vector3d normal = Eigen::Vector3d::Zero();
        double height = -std::numeric_limits<double>::infinity();
        double roll = std::numeric_limits<double>::infinity();
        double pitch = std::numeric_limits<double>::infinity();
        
        if (computeLandInfo(Eigen::Vector3d(step.x, step.y, step.yaw), max_size, above_points, normal, height, pitch, roll))
        {
            LOG(INFO)<<"above_points: "<<above_points;
            LOG(INFO)<<"max_size: "<<max_size;
            LOG(INFO)<<"normal: "<<normal.transpose();
        }
        else
        {
            LOG(INFO)<<"can not get the step info";
        }
    }
#endif

    // 检查两脚之间的参数
    for (int i = 0; i < steps.size(); i++)
    {
        Footstep last_step, current_step;
        double dis, height_diff, yaw_diff;
        if (i == 0)
        {
            last_step = start_p->footstep;
            current_step = steps.at(i);
        }
        else
        {
            last_step = steps.at(i-1);
            current_step = steps.at(i);
        }
        dis = (Eigen::Vector2d(last_step.x, last_step.y) - Eigen::Vector2d(current_step.x, current_step.y)).norm();
        height_diff = abs(last_step.z - current_step.z);
        yaw_diff = abs(last_step.yaw - current_step.yaw);
        
        if (dis > 0.6 || height_diff > 0.4 || yaw_diff > 15/57.3)
        {
#ifdef DEBUG
            LOG(ERROR)<<"planned steps error";
            LOG(INFO)<<dis<<" "<<height_diff<<" "<<yaw_diff;
#endif
            return false;
        }        
    }
    // LOG(INFO)<<"1 CHECK";
    // 更细致的检查
    for (int i = 0; i < steps.size(); i++)
    {
        Footstep last_step, current_step;
        if (i == 0)
        {
            last_step = start_p->footstep;
            current_step = steps.at(i);
        }
        else
        {
            last_step = steps.at(i - 1);
            current_step = steps.at(i);
        }
        // 当是左脚时
        Eigen::Vector2d direct_v(current_step.x - last_step.x, current_step.y - last_step.y);
        Eigen::AngleAxisd ad(last_step.yaw, Eigen::Vector3d::UnitZ());
        direct_v = (ad.toRotationMatrix().inverse() * (Eigen::Vector3d(direct_v.x(), direct_v.y(), 0))).head(2);
        // 先转到机器人上一只的脚坐标系下
        if (last_step.robot_side == LEFT)
        {
            // x-y平面
            if (!(direct_v.x() >= -0.15 && direct_v.x() <= 0.4 && direct_v.y() > - 0.4 && direct_v.y() <= -0.1))
            {
                LOG(ERROR)<<direct_v.transpose();
                LOG(ERROR)<<ad.toRotationMatrix();
                LOG(ERROR)<<last_step.x<<" "<<last_step.y<<" "<<last_step.z<<" "<<last_step.yaw;
                LOG(ERROR)<<current_step.x<<" "<<current_step.y<<" "<<current_step.z<<" "<<current_step.yaw;
                return false;
            }
        }
        else // 当是右脚
        {
            if (!(direct_v.x() >= -0.15 && direct_v.x() <= 0.5 && direct_v.y() <= 0.45 && direct_v.y() >= 0.1))
            {
                LOG(ERROR)<<direct_v.transpose();
                LOG(ERROR)<<ad.toRotationMatrix();
                LOG(ERROR)<<last_step.x<<" "<<last_step.y<<" "<<last_step.z<<" "<<last_step.yaw;
                LOG(ERROR)<<current_step.x<<" "<<current_step.y<<" "<<current_step.z<<" "<<current_step.yaw;
                return false;
            }
        }
        // roll方向
        if (abs(last_step.roll - current_step.roll) > 15/57.3)
        {
            LOG(ERROR)<<"roll";
            return false;
        }
        // pitch方向
        if (abs(last_step.pitch - current_step.pitch) > 30/57.3)
        {
            LOG(ERROR)<<"PITCH";
            return false;
        }
    }
    return true;
}

bool AstarHierarchicalFootstepPlanner::testNodeExtension()
{
    vector<FootstepNodePtr> extensions;
    LOG(INFO)<<start_p->footstep.x<<" "<<start_p->footstep.y<<" "<<prestart_p->footstep.x<<" "<<prestart_p->footstep.y;
    return nodeExtension(start_p, prestart_p, extensions);
}

bool AstarHierarchicalFootstepPlanner::testBasicTransitions()
{
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> result = basicTransitions(start_p);
    cv::Mat image1 = cv::Mat::zeros(localmap.getSize().x(), localmap.getSize().y(), CV_8UC3);
    cv::Mat image2 = image1.clone();
    for (auto & pair : result)
    {
        grid_map::Index index1, index2;
        if (localmap.getIndex(pair.first.head(2), index1) && localmap.getIndex(pair.second.head(2), index2))
        {
            image1.at<cv::Vec3b>(index1.x(), index1.y()) = cv::Vec3b(255, 0, 0);
            image1.at<cv::Vec3b>(index2.x(), index2.y()) = cv::Vec3b(0, 0, 255);
        }
    }

    // 原始图像尺寸
    int originalWidth = image1.cols;
    int originalHeight = image1.rows;

    // 放大倍数
    float scaleFactor = 3.0;  // 放大为原来的两倍

    // 计算放大后的尺寸
    int scaledWidth = static_cast<int>(originalWidth * scaleFactor);
    int scaledHeight = static_cast<int>(originalHeight * scaleFactor);

    // 创建放大后的图像
    cv::Mat scaledImg;
    resize(image1, scaledImg, cv::Size(scaledWidth, scaledHeight));  // 调整图像大小

    cv::imshow("iamge 1", scaledImg);
    cv::waitKey(0);

    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> result2 = basicTransitions(start_p->PreFootstepNode);
    for (auto & pair : result2)
    {
        grid_map::Index index1, index2;
        if (localmap.getIndex(pair.first.head(2), index1) && localmap.getIndex(pair.second.head(2), index2))
        {
            image2.at<cv::Vec3b>(index1.x(), index1.y()) = cv::Vec3b(255, 0, 0);
            image2.at<cv::Vec3b>(index2.x(), index2.y()) = cv::Vec3b(0, 0, 255);
        }
    }
    // 创建放大后的图像
    cv::Mat scaledImg2;
    resize(image2, scaledImg2, cv::Size(scaledWidth, scaledHeight));  // 调整图像大小
    cv::imshow("iamge 2", scaledImg2);
    cv::waitKey(0);
}

bool AstarHierarchicalFootstepPlanner::testComputeHcost()
{
    double start_hcost, pre_start_hcost;
    computeHcost(start_p, start_hcost);
    computeHcost(start_p->PreFootstepNode, pre_start_hcost);
    LOG(INFO)<<"cost: "<<start_hcost<<" "<<pre_start_hcost;
}

bool AstarHierarchicalFootstepPlanner::testGetNodeString()
{
    auto node1 = std::make_shared<FootstepNode>(Eigen::Vector3d(0.2, 0.11, 0.1), 0);
    node1->PreFootstepNode = start_p;
    node1->footstep.robot_side = start_p->footstep.getInverseRobotSide();
    LOG(INFO)<<node1->footstep.robot_side;
    LOG(INFO)<<start_p->footstep.robot_side;
    auto node2 = std::make_shared<FootstepNode>(Eigen::Vector3d(0.11, -0.21, -0.03), 0);
    node2->PreFootstepNode = node1;
    node2->footstep.robot_side = node1->footstep.getInverseRobotSide();
    LOG(INFO)<<node2->footstep.robot_side;
    LOG(INFO)<<node1->footstep.robot_side;
    LOG(INFO)<<start_p->footstep.robot_side;
    string ss1, ss2;
    if (getNodeString(node1, ss1) && getNodeString(node2, ss2))
    {
        LOG(INFO)<<ss1;
        LOG(INFO)<<ss2;
        return true;
    }
    else
    {
        return false;
    }

}

bool AstarHierarchicalFootstepPlanner::testArriveGoal()
{
    if (arriveGoal(start_p))
    {
        LOG(INFO)<<"ARR";
    }
    else
    {
        LOG(INFO)<<"NOT ARR";
    }

    auto node1 = std::make_shared<FootstepNode>(Eigen::Vector3d(1.2, 0.05, 0.05), 0);

    if (arriveGoal(node1))
    {
        LOG(INFO)<<"ARR";
    }
    else
    {
        LOG(INFO)<<"NOT ARR";
    }
    
}

// bool AstarHierarchicalFootstepPlanner::testGetPointsInFootArea()
// {
//     Eigen::Vector3d p1(0.2, 0.1, 0.1);
//     Eigen::Vector3d p2(0.3, 0.1, 0.1);
//     vector<vector<Eigen::Vector3d>> plane_points1, plane_points2;
//     vector<Eigen::Vector3d> allpoints1, allpoints2;
//     if (getPointsInFootArea(p1, plane_points1, allpoints1))
//     {
//         LOG(INFO)<<plane_points1.size()<<" "<<allpoints1.size();
//     }
//     cv::Mat image = plane_image.clone();
//     for (auto & points : plane_points1)
//     {
//         for (auto & p : points)
//         {
//             grid_map::Index index;
//             if (localmap.getIndex(p.head(2), index))
//             {
//                 image.at<cv::Vec3b>(index.x(), index.y()) = cv::Vec3b(255, 255, 255);
//             }
//         }
//     }
//     cv::imshow("image", image);
//     cv::waitKey(0);

//     cv::Mat image_ = plane_image.clone();
//     pcl::PointCloud<pcl::PointXYZ> cloud;
//     for (auto & p : allpoints1)
//     {
//         cloud.emplace_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
//         grid_map::Index index;
//         if (localmap.getIndex(p.head(2), index))
//         {
//             image_.at<cv::Vec3b>(index.x(), index.y()) = cv::Vec3b(0, 0, 255);
//         }
//     }
//     cv::imshow("image", image_);
//     cv::waitKey(0);
//     pcl::io::savePCDFile("/home/bhr/catkin_beijing4th/src/elevation_mapping/AstarHierarchicalFootstepPlanner/data/footcloud.pcd", cloud);
//     // pcl::io::savePCDFile("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/data/footcloud.pcd", cloud);
//     if (getPointsInFootArea(p2, plane_points2, allpoints2))
//     {
//         LOG(INFO)<<plane_points2.size()<<" "<<allpoints2.size();
//     }
//     return true;
// }

// bool AstarHierarchicalFootstepPlanner::testComputeLandInfo()
// {
//     vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> results = fineTransitions(Eigen::Vector3d(0.32, 0.18, 0.1), start_p);
//     int max_size;
//     int above_points;
//     Eigen::Vector3d plane_normal;
//     for (auto & p : results)
//     {
//         if (computeLandInfo(p.second, max_size, above_points, plane_normal))
//         {
//             LOG(INFO)<<"*****land plane";
//             LOG(INFO)<<max_size<<" "<<above_points<<" "<<plane_normal.transpose();
//         }
//         else
//         {
//             LOG(INFO)<<"not land plane";
//         }
//         vector<vector<Eigen::Vector3d>> plane_points1;
//         vector<Eigen::Vector3d> allpoints1;
//         if (getPointsInFootArea(p.second, plane_points1, allpoints1))
//         {
//             LOG(INFO)<<plane_points1.size()<<" "<<allpoints1.size();
//         }
//         cv::Mat image = plane_image.clone();
//         for (auto & points : plane_points1)
//         {
//             for (auto & p : points)
//             {
//                 grid_map::Index index;
//                 if (localmap.getIndex(p.head(2), index))
//                 {
//                     image.at<cv::Vec3b>(index.x(), index.y()) = cv::Vec3b(255, 255, 255);
//                 }
//             }
//         }
//         cv::imshow("image", image);
//         cv::waitKey(0);
//     }
// }

bool AstarHierarchicalFootstepPlanner::testFineTransitions()
{
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> results1 = fineTransitions(Eigen::Vector3d(0.2, 0.18, -0.1), start_p);

    vector<Eigen::Vector3d> results2 = fineTransitionsBasic(Eigen::Vector3d(0.2, 0.18, -0.1));

    for (auto & r : results1)
    {
        LOG(INFO)<<r.first.transpose()<<" "<<r.second.transpose();
    }
    for (auto & r : results2)
    {
        LOG(INFO)<<r.transpose();
    }
    
}

bool AstarHierarchicalFootstepPlanner::testFineTransitionsBasic()
{
    vector<Eigen::Vector3d> fines = fineTransitionsBasic(Eigen::Vector3d(start_p->footstep.x, start_p->footstep.y, start_p->footstep.yaw));
    
    LOG(INFO)<<start_p->footstep.x<<" "<<start_p->footstep.y<<" "<<start_p->footstep.yaw;
    for (auto & p : fines)
    {
        LOG(INFO)<<p.transpose();
    }
    
}

bool AstarHierarchicalFootstepPlanner::testFineLandPoint()
{
    Eigen::Vector3d p(1.1, 0.2, 0.1);
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> result = fineLandPoint(p);
    for (auto & r : result)
    {
        LOG(INFO)<<r.first.transpose()<<" "<<r.second.transpose();
    }
}

// bool AstarHierarchicalFootstepPlanner::testComputeLandPointScore()
// {
//     // computeLandPointScore(std::pair<Eigen::Vector3d, Eigen::Vector3d> land_point, double & score)
//     Eigen::Vector3d p(1.2, 0.11, 0.1);
//     vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> left_cands = fineLandPoint(p);
//     for (auto & cand : left_cands)
//     {
//         double score = 0;
//         if (computeLandPointScore(cand, score))
//         {
//             LOG(INFO)<<"score: "<<score;
//         }
//         else
//         {
//             LOG(INFO)<<"****";
//         }
//     }
    
// }

bool AstarHierarchicalFootstepPlanner::testComputeRollPitch()
{
    // computeRollPitch(Eigen::Vector3d normal, double yaw, double & roll, double & pitch)
    Eigen::AngleAxisd ad(-15/57.3, Eigen::Vector3d::UnitY());
    Eigen::Vector3d n = ad.toRotationMatrix() * Eigen::Vector3d::UnitZ();
    LOG(INFO)<<n.transpose();
    LOG(INFO)<<asin(n.x())*57.3;
    double y = 3.14/2;
    Eigen::Vector3d eular;
    computeRollPitch(n, y, eular);
    LOG(INFO)<<(eular * 57.3).transpose();
}

bool AstarHierarchicalFootstepPlanner::testSwingHeight()
{
    // swingHeight(grid_map::Position start, grid_map::Position end, double & swing_height_change, double & swing_height_max)
    grid_map::Position start(start_p->footstep.x, start_p->footstep.y), end(end_p->footstep.x, end_p->footstep.y);
    LOG(INFO)<<start.transpose();
    LOG(INFO)<<end.transpose();
    double swing_height, swing_height_max;
    swingHeight(start, end, swing_height, swing_height_max);
    LOG(INFO)<<swing_height<<" "<<swing_height_max;
}

// bool AstarHierarchicalFootstepPlanner::testComputeTransitionScore()
// {
//     vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> special_transitions = basicTransitions(start_p);
//     for (auto & trans : special_transitions)
//     {
//         double score = 0;
//         bool dangerous = false;
//         LOG(INFO)<<trans.second.transpose();
//         double height = 0;
//         Eigen::Vector3d normal;
//         if (computeTransitionScore(trans, start_p, prestart_p, dangerous, score, height, normal))
//         {
//             LOG(INFO)<<"score: "<<score;
//         }
//         else
//         {
//             LOG(INFO)<<"ERROR";
//         }   
//     }
// }

// bool AstarHierarchicalFootstepPlanner::testComputeTransitionStrictScore()
// {
//     // computeTransitionStrictScore(std::pair<Eigen::Vector3d, Eigen::Vector3d> transition, FootstepNodePtr current_node, FootstepNodePtr pre_node, double & score)
//     vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> special_transitions = basicTransitions(start_p);
//     for (auto & trans : special_transitions)
//     {
//         double score = 0;
//         double height = 0;
//         LOG(INFO)<<trans.second.transpose();
//         Eigen::Vector3d normal;
//         if (computeTransitionStrictScore(trans, start_p, prestart_p, score, height, normal))
//         {
//             LOG(INFO)<<"score: "<<score;
//         }
//         else
//         {
//             LOG(INFO)<<"ERROR";
//         }   
//     }
// }

bool AstarHierarchicalFootstepPlanner::testPlan()
{
    return plan();
}

bool AstarHierarchicalFootstepPlanner::testComputeAvoidPoints()
{
    vector<Eigen::Vector3d> avoid_points;
    Eigen::Vector2d swing_start(0., 0.02);
    Eigen::Vector2d swing_end(0.8, 0.1);
    grid_map::Index swing_start_index, swing_end_index;
    if (localmap.getIndex(swing_start, swing_start_index) && localmap.getIndex(swing_end, swing_end_index))
    {
        grid_map::Position3 swing_start_p3, swing_end_p3;
        if (localmap.getPosition3("elevation", swing_start_index, swing_start_p3) && localmap.getPosition3("elevation", swing_end_index, swing_end_p3))
        {
            avoid_points.emplace_back(swing_start_p3);
            grid_map::LineIterator iter(localmap, swing_start, swing_end);
            while (!iter.isPastEnd())
            {
                grid_map::Index iter_index(*iter);
                ++iter;
                // 保证下一个需要使用的点不超出范围
                if (iter.isPastEnd())
                {
                    break;
                }
                grid_map::Index iter_index_plus(*iter);
                for (auto & plane_image_8 : plane_images)
                {
                    if (plane_image_8.at<uchar>(iter_index.x(), iter_index.y()) != plane_image_8.at<uchar>(iter_index_plus.x(), iter_index_plus.y()))
                    {
                        if (plane_image_8.at<uchar>(iter_index.x(), iter_index.y()) == 255)
                        {
                            grid_map::Position3 p3;
                            if (localmap.getPosition3("elevation", iter_index, p3))
                            {
                                avoid_points.emplace_back(p3);
                            }
                        }
                        else
                        {
                            grid_map::Position3 p3;
                            if (localmap.getPosition3("elevation", iter_index_plus, p3))
                            {
                                avoid_points.emplace_back(p3);
                            }
                        }
                    }
                }
            }
            avoid_points.emplace_back(swing_end_p3);
        }
        else
        {
            LOG(INFO)<<"can not get elevation";
        }
        
    }
    else
    {
        LOG(INFO)<<"can not get index";
    }
    
    cv::Mat all_points_image = plane_image.clone();
    for (auto & p : avoid_points)
    {
        grid_map::Index index;
        if (localmap.getIndex(p.head(2), index))
        {
            all_points_image.at<cv::Vec3b>(index.x(), index.y()) = cv::Vec3b(0, 0, 255);
        }
    }
    cv::imshow("all point", all_points_image);
    cv::imwrite("/home/bhr/catkin_beijing4th/src/elevation_mapping/AstarHierarchicalFootstepPlanner/data/avoidPoints/1.png", all_points_image);
    // cv::imwrite("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/data/avoidPoints/1.png", all_points_image);
    cv::waitKey(0);

    
    auto tmp_points = avoid_points;
    avoid_points.clear();
    for (int i = 1; i < tmp_points.size() - 1; i++)
    {
        Eigen::Vector3d convex_point1, convex_point2;
        if (getConvexPoint(tmp_points.at(i - 1), tmp_points.at(i), convex_point1))
        {
            avoid_points.emplace_back(convex_point1);
        }
        double dis_two = (tmp_points.at(i - 1) - tmp_points.at(i + 1)).head(2).norm();
        double dis_mid = (tmp_points.at(i - 1) - tmp_points.at(i)).head(2).norm();
        double height_mid = (tmp_points.at(i + 1) - tmp_points.at(i - 1)).z()*(dis_mid / dis_two) + tmp_points.at(i - 1).z();
        if (height_mid + 0.03 < tmp_points.at(i).z())
        {
            avoid_points.emplace_back(tmp_points.at(i));
        }
        if (getConvexPoint(tmp_points.at(i), tmp_points.at(i + 1), convex_point2))
        {
            avoid_points.emplace_back(convex_point2);
        }   
    }

    cv::Mat convex_points_image = plane_image.clone();
    for (auto & p : avoid_points)
    {
        grid_map::Index index;
        if (localmap.getIndex(p.head(2), index))
        {
            convex_points_image.at<cv::Vec3b>(index.x(), index.y()) = cv::Vec3b(0, 0, 255);
        }
    }
    cv::imshow("all point", convex_points_image);
    // cv::imwrite("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/data/avoidPoints/2.png", convex_points_image);
    cv::imwrite("/home/bhr/catkin_beijing4th/src/elevation_mapping/AstarHierarchicalFootstepPlanner/data/avoidPoints/2.png", convex_points_image);
    cv::waitKey(0);
}

AstarHierarchicalFootstepPlanner::~AstarHierarchicalFootstepPlanner()
{
    localmap.clearAll();
    label_localmap.clearAll();
    close_set.clear();
    transitions.clear();
    steps.clear();
#ifdef DEBUG
    outfile.close();
#endif
}