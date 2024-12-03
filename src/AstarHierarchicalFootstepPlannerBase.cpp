#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerBase.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <glog/logging.h>
#include <PEAC_AHFP/plane_fitter_pcl_AHFP.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <pcl/io/pcd_io.h>
// #include <grid_map_core/iterators/LineIterator.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <chrono>
#include <boost/thread.hpp>
// 这个构造函数，有问题，暂时不要使用
// AstarHierarchicalFootstepPlannerBase::AstarHierarchicalFootstepPlannerBase(grid_map::GridMap & lm, FootParam footparam_, double hip_width_)
// {
//     localmap = lm;
//     // 加一个对localmap提取平面的操作
//     resolution = localmap.getResolution();
//     // debug_pub = 
//     mapsize = localmap.getSize().x() * localmap.getSize().y();
//     // LOG(INFO)<<localmap.getSize().transpose();
//     // label_index.resize(localmap.getSize().x(), localmap.getSize().y());
//     // LOG(INFO)<<localmap.getSize().y()<<" "<<localmap.getSize().x();
//     // LOG(INFO)<<label_index.rows()<<" "<<label_index.cols();
//     computePlanarInfor();
//     footparam = footparam_;
//     hip_width = hip_width_;
//     footsize_inmap = (ceil((footparam.x_button + footparam.x_upper)/resolution)) * (ceil((footparam.y_left + footparam.y_right)/resolution));
// #ifdef DEBUG
//     LOG(INFO)<<"resolution: "<<resolution;
//     LOG(INFO)<<"mapsize: "<<mapsize;
//     LOG(INFO)<<"footparam: "<<footparam.x_upper<<" "<<footparam.x_button<<" "<<footparam.y_left<<" "<<footparam.y_right;
//     LOG(INFO)<<"hip_width: "<<hip_width;
//     LOG(INFO)<<"footsize_inmap: "<<footsize_inmap;
//     // outfile = std::ofstream("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/data/out.txt");
//     // outfile = std::ofstream("/home/lichao/TCDS/src/pip_line/data/out.txt");
// #endif
//     // 每个transition x y yaw 右脚往左扩展的步态点
//     initial_transitions();
//     // LOG(INFO)<<"transitions size: "<<transitions.size();
// }

// 请确保不会出现某个栅格点的label为nan，而其周围的栅格点label确实一个值，如果按照自己的平面提取算法，应该不会出现这个情况
AstarHierarchicalFootstepPlannerBase::AstarHierarchicalFootstepPlannerBase(grid_map::GridMap & label_map, cv::Mat & plane_iamage_, vector<cv::Mat> & planes_image_, vector<planeInfo> & planes_info_, FootParam footparam_, double hip_width_):label_localmap(label_map),footparam(footparam_), hip_width(hip_width_), plane_image(plane_iamage_), plane_images(planes_image_),planes_info(planes_info_)
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
    // outfile = std::ofstream("/home/lichao/TCDS/src/pip_line/data/out.txt");
#endif
    initial_transitions();
    LOG(INFO)<<"construct planner over";
}

AstarHierarchicalFootstepPlannerBase::AstarHierarchicalFootstepPlannerBase()
{
    initial_transitions();
}

// plane_image是调试，显示时所用的平面分割结果图
// plane_images 仅仅在计算障碍点，避障点时用到
void AstarHierarchicalFootstepPlannerBase::setBasicInfor(grid_map::GridMap & label_map, cv::Mat & plane_iamage_, vector<cv::Mat> & planes_image_, vector<planeInfo> & planes_info_, FootParam footparam_, double hip_width_)
{
    label_localmap = label_map;
    localmap = label_localmap;
    plane_image = plane_iamage_;
    plane_images = planes_image_;
    planes_info = planes_info_;
    footparam = footparam_;
    hip_width = hip_width_;

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
    // outfile = std::ofstream("/home/lichao/TCDS/src/pip_line/data/out.txt");
#endif
    LOG(INFO)<<"construct planner over";
}

void AstarHierarchicalFootstepPlannerBase::initial_transitions()
{
    for (int i = -1; i < 4; i++)
    {
        for (int j = -2; j < 5; j++)
        {
            for (int k = -1; k < 2; k++)
            {
                if (j == -1 && (k == -3 || k == -2))// 靠的太近时角度不允许内转太多
                {
                    continue;
                }
                if (j == 0 && k ==-3)
                {
                    continue;
                }
                
                Eigen::Vector3d transition = Eigen::Vector3d(i * 0.11,   0.015 * j + 0.23,  k*3/57.3);
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
                
                Eigen::Vector3d transition = Eigen::Vector3d(i * 0.05,   0.015 * j + 0.23,  k*3/57.3);
                // LOG(INFO)<<transition.transpose();
                combine_transitions.emplace_back(transition);
            }
        }
    }
}

// 默认起点和终点位置是对的
bool AstarHierarchicalFootstepPlannerBase::initial(Eigen::Vector3d start, Eigen::Vector3d prestart, int support_side, Eigen::Vector3d goal)
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
#ifdef DEBUG
    LOG(INFO)<<"GOAL FINISH";
#endif
    
#ifdef DEBUG
    // if (!outfile.is_open()) 
    // {
    //     std::cerr << "无法打开文件--" << std::endl;
    //     return false;
    // }
#endif
    return true;
}

bool AstarHierarchicalFootstepPlannerBase::getPointInfoInPlane(Eigen::Vector3d p, double & height, int & plane_index, double & pitch, double & roll)
{
    int max_size = 0, above_points = 0;
    Eigen::Vector3d plane_normal = Eigen::Vector3d::Zero();
    height = -std::numeric_limits<double>::infinity();
    plane_index = -1;
    pitch = std::numeric_limits<double>::infinity();
    roll = std::numeric_limits<double>::infinity();
    if (computeLandInfo(p, max_size, above_points, plane_normal, height, plane_index, pitch, roll))
    {
#ifdef DEBUG
        LOG(INFO)<<"plane_normal = "<<plane_normal.transpose();
#endif
        if (plane_normal.z() > 0.85)
        {
            if (above_points == 0)
            {
                if (max_size > 0.7 * footsize_inmap)
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
bool AstarHierarchicalFootstepPlannerBase::startPoint2Node(Eigen::Vector3d p, FootstepNodePtr node)
{
    double height = 0.0, pitch = 0, roll = 0;
    int plane_index;
    if (getPointInfoInPlane(p, height, plane_index, pitch, roll))
    {
        node->footstep.x = p(0);
        node->footstep.y = p(1);
        node->footstep.z = height;
        node->footstep.yaw = p(2);
        node->footstep.roll = roll;
        node->footstep.pitch = pitch;
        node->plane_index = plane_index;
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


// tested
// 是否是危险的，是分数为0，并需要微调
// 不是危险的，计算得分，得分过低的不再作为扩展节点
bool AstarHierarchicalFootstepPlannerBase::computeTransitionScore(std::pair<Eigen::Vector3d, Eigen::Vector3d> transition, FootstepNodePtr current_node, FootstepNodePtr pre_node, bool & dangerous, double & score, double & height, Eigen::Vector3d & plane_normal, int & plane_index, double & pitch, double & roll)
{
    dangerous = false;
    score = 0;
    // LOG(INFO)<<"function computeTransitionScore";
    // 如果采集的支撑区域位于同一平面，肯定可以，如果采集的支撑区域绝大部分区域位于同一平面，且其他位于平面之下，注意不是高度值小于平面。注意在此处需要考虑分层的打分，比方说缺少的区域或者发生碰撞的区域，如果发生碰撞的区域位于边缘区域，那么可以通过打分再通过后续细化
    int max_size = 0; // 属于同一最大平面的点数
    int above_points = 0; // 超出限制部分的点数
    plane_normal = Eigen::Vector3d::Zero();
    height = -std::numeric_limits<double>::infinity();
    plane_index = -1;
    pitch = std::numeric_limits<double>::infinity();
    roll = std::numeric_limits<double>::infinity();
    if (!computeLandInfo(transition.second, max_size, above_points, plane_normal, height, plane_index, pitch, roll))
    {
#ifdef DEBUG
        LOG(INFO)<<"CANNOT computeLandInfo";
#endif
        return false;
    }
#ifdef DEBUG
    LOG(INFO)<<"XXXX";
#endif
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

// tested 计算落脚点的得分
bool AstarHierarchicalFootstepPlannerBase::computeLandPointScore(std::pair<Eigen::Vector3d, Eigen::Vector3d> land_point, double & score, double & height, int & plane_index, double & pitch, double & roll)
{
    int max_size = 0; // 属于同一最大平面的点数
    int above_points = 0; // 超出限制部分的点数
    Eigen::Vector3d plane_normal = Eigen::Vector3d::Zero();
    height = -std::numeric_limits<double>::infinity();
    plane_index = -1;
    pitch = std::numeric_limits<double>::infinity();
    roll = std::numeric_limits<double>::infinity();
    if (!computeLandInfo(land_point.second, max_size, above_points, plane_normal, height, plane_index, pitch, roll))
    {
#ifdef DEBUG
        LOG(INFO)<<"can not get land info";
#endif
        return false;
    }
#ifdef DEBUG
    LOG(INFO)<<"...";
#endif
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

// 注意这个的原理，图像中的已有数据的点不能与机器人躯干发生碰撞，至于投影地图外的暂不考虑，因为很难把这种情况考虑进去
bool AstarHierarchicalFootstepPlannerBase::traversibilityCheck(ScoreMarkerNodePtr node)
{
    
    grid_map::Position position = node->point.head(2);
    if (!label_localmap.isInside(position))
    {
        return false;
    }

    // grid_map::Position start_position, end_position;
    // label_localmap.getPosition(grid_map::Index(0, 0), start_position);
    // label_localmap.getPosition(grid_map::Index(label_localmap.getSize().x() - 1, label_localmap.getSize().y() - 1), end_position);
    double knee_radius = 0.2;
    double upperbody_radius = 0.5;
    // if (position.x() - upperbody_radius < end_position.x() || position.x() + upperbody_radius > start_position.x())
    // {
    //     return false;
    // }
    // if (position.y() - upperbody_radius < end_position.y() || position.y() - upperbody_radius > start_position.y())
    // {
    //     return false;
    // }
    Eigen::Vector3d normal(planes_info.at(node->plane_index).normal.x(), planes_info.at(node->plane_index).normal.y(), planes_info.at(node->plane_index).normal.z());
    Eigen::Vector3d center(planes_info.at(node->plane_index).center.x(), planes_info.at(node->plane_index).center.y(), planes_info.at(node->plane_index).center.z());
    for (grid_map::CircleIterator iterator(label_localmap, position, knee_radius); !iterator.isPastEnd(); ++iterator)
    {
        grid_map::Position3 p3;
        if (label_localmap.getPosition3("elevation",*iterator, p3))
        {
            double dis = (p3 - center).dot(normal);
            if (dis > 0.3)
            {
                return false;
            }
        }
    }
    for (grid_map::CircleIterator iterator(label_localmap, position, upperbody_radius); !iterator.isPastEnd(); ++iterator)
    {
        grid_map::Position3 p3;
        if (label_localmap.getPosition3("elevation",*iterator, p3))
        {
            double dis = (p3 - center).dot(normal);
            if (dis > 0.6)
            {
                return false;
            }
        }
    }
    return true;
}

// 根据变换前后的向量求变换矩阵 u变换前的向量 v变换后的向量
Eigen::Matrix3d AstarHierarchicalFootstepPlannerBase::computeRotationMatrix(const Eigen::Vector3d& u, const Eigen::Vector3d& v) 
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
Eigen::Vector3d AstarHierarchicalFootstepPlannerBase::adjustEulerAngles(const Eigen::Vector3d& eulerAngles) 
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
Eigen::Vector3d AstarHierarchicalFootstepPlannerBase::Quaterniond2EulerAngles(Eigen::Quaterniond q) 
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
Eigen::Vector3d AstarHierarchicalFootstepPlannerBase::Matrix3d2EulerAngles(Eigen::Matrix3d m)
{
    Eigen::Quaterniond qd(m);
    return Quaterniond2EulerAngles(qd);
}


// 已确定
// tested
// 注意eulerAngles(2, 1, 0) 出来的分别是绕z，y，x的旋转角
// 但是eigen本身的eulerAngles是有问题的，所以使用网上使用的Matrix3d2EulerAngles
void AstarHierarchicalFootstepPlannerBase::computeRollPitch(Eigen::Vector3d normal, double yaw, Eigen::Vector3d & euler)
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
bool AstarHierarchicalFootstepPlannerBase::swingHeight(grid_map::Position start, grid_map::Position end, double & swing_height_change, double & swing_height_max)
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
vector<Eigen::Vector3d> AstarHierarchicalFootstepPlannerBase::fineTransitionsBasic(Eigen::Vector3d parent_transition)
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
vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> AstarHierarchicalFootstepPlannerBase::fineLandPoint(Eigen::Vector3d land_point)
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

bool AstarHierarchicalFootstepPlannerBase::getLandAreaPoints(Eigen::Vector3d ankle, vector<Eigen::Vector3d> & points)
{
    Eigen::AngleAxisd ax(ankle.z(), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d mid(ankle.x(), ankle.y(), 0);

    Eigen::Vector3d mid_top = ax.toRotationMatrix() * Eigen::Vector3d(footparam.x_upper, 0, 0) + mid;
    Eigen::Vector3d mid_down = ax.toRotationMatrix() * Eigen::Vector3d(- footparam.x_button, 0, 0) + mid;

    Eigen::Vector3d mid_left = ax.toRotationMatrix() * Eigen::Vector3d(0, footparam.y_left, 0) + mid;
    Eigen::Vector3d mid_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - footparam.y_right, 0) + mid;

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
                            points.emplace_back(cor_position);
                        }
                        else
                        {
#ifdef DEBUG
                            LOG(INFO)<<"cor nan";
#endif
                        }
                    }

                    else
                    {
#ifdef DEBUG
                        LOG(INFO)<<"can not get cor";
#endif
                    }

                }
                else
                {
#ifdef DEBUG
                    LOG(INFO)<<"can not get cor2";
#endif
                }
            }
        }
        return true;
    }
    else
    {
#ifdef DEBUG
        LOG(INFO)<<"out of map";
#endif
        return false;
    }
    
}

vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> AstarHierarchicalFootstepPlannerBase::fineTransitions(Eigen::Vector3d point, FootstepNodePtr current_node)
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
vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> AstarHierarchicalFootstepPlannerBase::basicTransitions(FootstepNodePtr current_node)
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
            // 当前步和当前步的前一步不在同一平面，就执行并步
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
void AstarHierarchicalFootstepPlannerBase::computePlanarInfor()
{
    pcl::PointCloud<pcl::PointXYZ> pc = gridMap2Pointcloud();
#ifdef DEBUG
    // pcl::io::savePCDFile("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/data/map.pcd", pc);
    pcl::io::savePCDFile("/home/bhr/catkin_beijing4th/src/elevation_mapping/AstarHierarchicalFootstepPlanner/data/map.pcd", pc);
#endif
    // 注意这个peac使用时的对应问题
    AHFP::PlanarContourExtraction pce(pc);
    pce.run();
    vector<cv::Mat> seg_planes = pce.getSegPlanes();
    plane_images = seg_planes;
#ifdef DEBUG
    LOG(INFO)<<"plane size: "<<seg_planes.size();
    for (int i = 0; i < seg_planes.size(); i++)
    {
        cv::imshow("seg_image", seg_planes.at(i));
        // cv::waitKey(0);
        // cv::imwrite("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/data/" + std::to_string(i) + ".jpg", seg_planes.at(i));
        cv::imwrite("/home/bhr/catkin_beijing4th/src/elevation_mapping/AstarHierarchicalFootstepPlanner/data/" + std::to_string(i) + ".jpg", seg_planes.at(i));
    }
    // 测试时，优化起点终点的位置
    // grid_map::Index start_index, start_pre_index, left_goal_index, gight_goal_index;
#endif
    planes = seg_planes.size();
    label_localmap = localmap;
    label_localmap.add("label");
    // LOG(INFO)<<label_localmap.getSize().transpose();
    // LOG(INFO)<<seg_planes.at(0).rows<<" "<<seg_planes.at(0).cols;
    // LOG(INFO)<<seg_planes.at(0).size();
    plane_image = cv::Mat::zeros(label_localmap.getSize().x(), label_localmap.getSize().y(), CV_8UC3);
    for (int i = 0; i < label_localmap.getSize().x(); i++)
    {
        for (int j = 0; j < label_localmap.getSize().y(); j++)
        {
            bool flag = false;
            for (int label = 1; label <= seg_planes.size(); label++)
            {
                if (seg_planes.at(label - 1).at<uchar>(i, j) == 255)
                {
                    flag = true;
                    label_localmap["label"](i ,j) = label;
                    // 后续计算
                    plane_image.at<cv::Vec3b>(i, j) = cv::Vec3b(default_colors[int(label)][0], default_colors[int(label)][1], default_colors[int(label)][2]);
#ifdef DEBUG
                    plane_image.at<cv::Vec3b>(i, j) = cv::Vec3b(default_colors[int(label)][0], default_colors[int(label)][1], default_colors[int(label)][2]);
#endif
                    break;
                }
            }
            if (!flag)
            {
                label_localmap["label"](i,j) = NAN;
            }
        }
    }
#ifdef DEBUG
    cv::imshow("plane image", plane_image);
    // cv::imwrite("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/data/planes.jpg", plane_image);
    cv::imwrite("/home/bhr/catkin_beijing4th/src/elevation_mapping/AstarHierarchicalFootstepPlanner/data/planes.jpg", plane_image);
    pcl::PointCloud<pcl::PointXYZ> planepoints;
    for (int i = 0; i < plane_image.rows; i++)
    {
        for (int j = 0; j < plane_image.cols; j++)
        {
            if (plane_image.at<cv::Vec3b>(i, j) != cv::Vec3b(0, 0, 0))
            {
                grid_map::Position3 p3;
                if (localmap.getPosition3("elevation", grid_map::Index(i, j), p3))
                {
                    planepoints.emplace_back(pcl::PointXYZ(p3.x(), p3.y(), p3.z()));
                }
            }
        }
    }
    pcl::io::savePCDFile("/home/bhr/catkin_beijing4th/src/elevation_mapping/AstarHierarchicalFootstepPlanner/data/planepoints.pcd", planepoints);
    // pcl::io::savePCDFile("/home/lichao/Darwin-op/src/elevation_map_ours/elevation_mapping/AstarHierarchicalFootstepPlanner/data/planepoints.pcd", planepoints);
    // cv::waitKey(0);
#endif
}

// tested
pcl::PointCloud<pcl::PointXYZ> AstarHierarchicalFootstepPlannerBase::gridMap2Pointcloud()
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
bool AstarHierarchicalFootstepPlannerBase::computeHcost(FootstepNodePtr node, double & hcost)
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

bool AstarHierarchicalFootstepPlannerBase::computeGcost(FootstepNodePtr node, double & gcost)
{
    double dis, angle_diff, height_diff;
    dis = (Eigen::Vector2d(node->footstep.x, node->footstep.y) - Eigen::Vector2d(node->PreFootstepNode->footstep.x, node->PreFootstepNode->footstep.y)).norm();
    angle_diff = abs(node->footstep.yaw - node->PreFootstepNode->footstep.yaw);
    height_diff = abs(node->footstep.z - node->PreFootstepNode->footstep.z);
    gcost = dis  /* angle_diff * 0.5 /*+ height_diff */ + node->PreFootstepNode->Gcost;
    return true;
}

// tested
bool AstarHierarchicalFootstepPlannerBase::getNodeString(FootstepNodePtr node, string & s)
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
bool AstarHierarchicalFootstepPlannerBase::arriveGoal(FootstepNodePtr node)
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
cv::Point AstarHierarchicalFootstepPlannerBase::calculateEndPoint(cv::Point start, double length, double angle) 
{
    int end_x = static_cast<int>(start.x + length * cos(angle));
    int end_y = static_cast<int>(start.y + length * sin(angle));
    return cv::Point(end_x, end_y);
}

bool AstarHierarchicalFootstepPlannerBase::plan()
{
    LOG(INFO)<<"ENTER PLAN";
#ifdef DEBUG
    LOG(INFO)<<start_p->footstep.x<<" "<<start_p->footstep.y<<" "<<start_p->footstep.z<<" "<<start_p->footstep.roll<<" "<<start_p->footstep.pitch<<" "<<start_p->footstep.yaw;
    LOG(INFO)<<prestart_p->footstep.x<<" "<<prestart_p->footstep.y<<" "<<prestart_p->footstep.z<<" "<<prestart_p->footstep.roll<<" "<<prestart_p->footstep.pitch<<" "<<prestart_p->footstep.yaw;
    LOG(INFO)<<end_left_p->footstep.x<<" "<<end_left_p->footstep.y<<" "<<end_left_p->footstep.z<<" "<<end_left_p->footstep.roll<<" "<<end_left_p->footstep.pitch<<" "<<end_left_p->footstep.yaw;
    LOG(INFO)<<end_right_p->footstep.x<<" "<<end_right_p->footstep.y<<" "<<end_right_p->footstep.z<<" "<<end_right_p->footstep.roll<<" "<<end_right_p->footstep.pitch<<" "<<end_right_p->footstep.yaw;
#endif
    p_queue.push(start_p);
    while (!p_queue.empty())
    {
        auto current_node = p_queue.top();
#ifdef DEBUG
        // outfile<<current_node->footstep.x<<" "<<current_node->footstep.y<<" "<<current_node->footstep.z<<" "<<current_node->footstep.roll<<" "<<current_node->footstep.pitch<<" "<<current_node->footstep.yaw<<" "<<current_node->footstep.robot_side<<endl;
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
#ifdef DEBUG
                LOG(INFO) << "child_nodes.size() = " << child_nodes.size() << endl;
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
            else
            {
                LOG(INFO) << "No node to expand";
            }
        }
    
        // 插入中断点，用于检查程序是否超时
        boost::this_thread::interruption_point();
    }
    LOG(INFO) << "planning error";
    return false;
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

bool AstarHierarchicalFootstepPlannerBase::getFootsteps(FootstepNodePtr node)
{
    auto iter_P = node;
    while (iter_P)
    {
#ifdef DEBUG
        LOG(INFO)<<iter_P->footstep.x<<" "<<iter_P->footstep.y<<" "<<iter_P->footstep.z<<" "<<iter_P->footstep.roll<<" "<<iter_P->footstep.pitch<<" "<<iter_P->footstep.yaw<<" "<<iter_P->footstep.robot_side;
#endif
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
#ifdef DEBUG
                LOG(INFO)<<step.x<<" "<<step.y<<" "<<step.z<<" "<<step.roll<<" "<<step.pitch<<" "<<step.yaw<<" "<<step.robot_side;
#endif
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
                return false;
            }
        }
        iter_P = iter_P->PreFootstepNode;
    }
    return false;
}

bool AstarHierarchicalFootstepPlannerBase::repairStanceStep(Footstep current_step, Footstep & footstep)
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
    int plane_index = -1;
    double roll = std::numeric_limits<double>::infinity();
    double pitch = std::numeric_limits<double>::infinity();
    if (!computeLandInfo(repair, max_size, above_points, plane_normal, height, plane_index, pitch, roll))
    {
        return false;
    }
#ifdef DEBUG
    LOG(INFO)<<".....";
#endif
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


bool AstarHierarchicalFootstepPlannerBase::checkFootstepsResult()
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
        int plane_index = -1;
        double roll = std::numeric_limits<double>::infinity();
        double pitch = std::numeric_limits<double>::infinity();
        
        if (computeLandInfo(Eigen::Vector3d(step.x, step.y, step.yaw), max_size, above_points, normal, height, plane_index, pitch, roll))
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
            if (!(direct_v.x() >= -0.15 && direct_v.x() <= 0.5 && direct_v.y() > - 0.45 && direct_v.y() <= -0.1))
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


AstarHierarchicalFootstepPlannerBase::~AstarHierarchicalFootstepPlannerBase()
{
    localmap.clearAll();
    label_localmap.clearAll();
    close_set.clear();
    transitions.clear();
    steps.clear();
#ifdef DEBUG
    // outfile.close();
#endif
}