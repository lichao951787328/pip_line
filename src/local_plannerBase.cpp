#include <local_plannerBase.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <grid_map_cv/InpaintFilter.hpp>
#include <ros/package.h>
#include <diy_msgs/footSteps.h>
// #define DEBUG
void initial_package_path(string package_name, string & package_path)
{
  package_path = ros::package::getPath(package_name);
  // 检查是否成功找到包路径
  if (package_path.empty()) {
      std::cerr << "Error: Could not find package " << package_name << std::endl;
  }
  cout<<"package path: "<<package_path<<endl;
}

localPlannerBase::localPlannerBase(std::shared_ptr<AstarHierarchicalFootstepPlannerBase> a):planner_P(std::move(a))
{
    
    initial_package_path("pip_line", package_path);
    LOG(INFO)<<"package path is: "<<package_path;
    pd.initial(package_path + "/config/plane_fitter_pcd.ini");

    // for debug using vscode
    // pd.initial("/home/lichao/TCDS/src/pip_line/config/plane_fitter_pcd.ini");

}
localPlannerBase::localPlannerBase()
{
    initial_package_path("pip_line", package_path);
    LOG(INFO)<<"package path is: "<<package_path;
    pd.initial(package_path + "/config/plane_fitter_pcd.ini");
    // for debug using vscode
    // pd.initial("/home/lichao/TCDS/src/pip_line/config/plane_fitter_pcd.ini");
}

void localPlannerBase::setPlanner(std::shared_ptr<AstarHierarchicalFootstepPlannerBase> a)
{
    planner_P = std::move(a);
}
void localPlannerBase::setFootParam(FootParam & foot_param_)
{
    foot_param = foot_param_;
}

void localPlannerBase::setHipWidth(double hip_width_)
{
    hip_width = hip_width_;
}

// localPlanner::localPlanner(grid_map::GridMap & map_, Eigen::Vector3d start_left_, Eigen::Vector3d start_right_, Eigen::Vector3d goal_, FootParam & foot_param_):map(map_),start_left(start_left_),start_right(start_right_),goal(goal_), foot_param(foot_param_)
// {
    
// }

void localPlannerBase::initial(Eigen::Vector3d start_, Eigen::Vector3d pre_start_, int support_flag_,Eigen::Vector3d goal_)
{
    start = start_;
    pre_start = pre_start_;
    support_flag = support_flag_;
    goal = goal_;
}

void localPlannerBase::Inpaint(int radius)
{
    const float minValue = map.get("elevation").minCoeffOfFinites();
    const float maxValue = map.get("elevation").maxCoeffOfFinites();
    cv::Mat originalImage;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "elevation", CV_8UC1, minValue, maxValue, originalImage);
    // cv::imshow("originalImage", originalImage);
    // cv::waitKey(0);
    // // 对空洞进行补齐
    // // 创建一个标记小区域的掩码
    cv::Mat smallRegionsMask = cv::Mat::zeros(originalImage.size(), CV_8UC1);

    cv::Mat inpainted;
    cv::inpaint(originalImage, smallRegionsMask, inpainted, radius, cv::INPAINT_TELEA);
    // cv::imshow("inpainted", inpainted);
    // cv::waitKey(0);

    // map.erase("elevation");
    map.clear("elevation");
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(inpainted, "elevation", map, minValue, maxValue);
}

void localPlannerBase::detectionPlane()
{
    pcl::PointCloud<pcl::PointXYZ> org_pc = gridMap2PointcloudOrganized();
    pd.detect(org_pc);
    // cv::imwrite("/home/lichao/TCDS/src/pip_line/data/result.png", pd.result);
    // cv::imshow("result", pd.result);
    // cv::waitKey(0);
    
    LOG(INFO)<<"PLANE SIZE: "<< pd.planes.size();
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
    planes_info = pd.planes_info;
    single_results = pd.planes;
}

void localPlannerBase::mergePlanes()
{
    class Graph 
    {
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
#ifdef DEBUG
    LOG(INFO)<<planes_info.size();
#endif
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
    // for (int i = 0; i < components.size(); i++)
    // {
    //     for (int j = 0; j < components.at(i).size(); j++)
    //     {
    //         cout<<components.at(i).at(j)<<" ";
    //     }
    //     cout<<endl;
    // }
    // vector<cv::Mat> merge_results;
    // vector<planeInfo> merge_planes;
    for (int i = 0; i < components.size(); i++)
    {
        // cv::Mat image = cv::Mat::zeros(seg_result_image.size(), seg_result_image.type());
        cv::Mat image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC3);
        for (int j = 0; j < components.at(i).size(); j++)
        {
            cv::Mat mask;
            cv::inRange(single_results.at(components.at(i).at(j)), cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), mask);
            single_results.at(components.at(i).at(j)).copyTo(image, mask);
        }
        merge_results.emplace_back(image);
        merge_planes.emplace_back(planes_info.at(components.at(i).front()));
    }
    // vector<cv::Mat> collision_free_images;

    vector<cv::Vec3b> colors = {
        cv::Vec3b(0, 0, 255),   // Red
        cv::Vec3b(0, 255, 0),   // Green
        cv::Vec3b(255, 0, 0),   // Blue
        cv::Vec3b(0, 255, 255), // Yellow
        cv::Vec3b(255, 0, 255), // Magenta
        cv::Vec3b(255, 255, 0), // Cyan
    };
    seg_image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC3);
    // seg_image for debug in planner
    for (int i = 0; i < merge_results.size(); i++)
    {
        cv::Mat image = merge_results.at(i);
        cv::Mat mask;
        cv::inRange(image, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), mask);
        seg_image.setTo(colors[i % colors.size()], mask);
    }
    // LOG(INFO)<<"merge planes: "<<merge_results.size();
    // for (auto & merge_image : merge_results)
    // {
    //     cv::imshow("merge_image", merge_image);
    //     cv::waitKey(0);
    // }
    
    map.add("label");
    cv::Mat plane_image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC3);
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            bool flag = false;
            for (int label = 0; label < merge_results.size(); label++)
            {
                if (merge_results.at(label).at<uchar>(i, j) == 255)
                {
                    flag = true;
                    map["label"](i ,j) = label;
                    break;
                }
            }
            if (!flag)
            {
                map["label"](i,j) = NAN;
            }
        }
    }
}

// void localPlanner::constructFeasibleMap()
// {
//     double resolution = map.getResolution();
//     double inflation_radius = 0.5;
//     int inflation_pixel = 0.5/resolution;
//     for (int i = 0; i < merge_results.size(); i++)
//     {
//         cv::Mat image = merge_results.at(i);
//         // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/image" + std::to_string(i) + ".png", image);
//         // cv::imshow("image", image);
//         // cv::waitKey(0);
//         int kernel_size = inflation_pixel;
//         cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
//         // 对图像进行膨胀操作
//         cv::Mat dilated_image;
//         cv::dilate(image, dilated_image, kernel);
//         // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/dilated_image" + std::to_string(i) + ".png", dilated_image);
//         // cv::imshow("dilated_image", dilated_image);
//         // cv::waitKey(0);
//         // 计算膨胀后的边缘
//         cv::Mat collision_layer = dilated_image - image;
//         // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/collision_layer" + std::to_string(i) + ".png", collision_layer);
//         // cv::imshow("collision_layer", collision_layer);
//         // cv::waitKey(0);
//         cv::Mat upper_body = cv::Mat::zeros(collision_layer.size(), CV_8UC1);
//         cv::Mat knee = cv::Mat::zeros(collision_layer.size(), CV_8UC1);
//         std::vector<cv::Point> white_points;
//         cv::findNonZero(collision_layer, white_points);
//         for (auto & cv_p : white_points)
//         {
//             grid_map::Position3 p3;
//             if (map.getPosition3("elevation", grid_map::Index(cv_p.y, cv_p.x), p3))
//             {
//                 // cout<<"  mm: "<cv_p<<endl;
//                 if (!std::isnan(p3.z()))
//                 {
//                     // Eigen::Vector3f p3f(p3.x(), p3.y(), p3.z());
//                     double dis = (p3 - merge_planes.at(i).center).dot(merge_planes.at(i).normal);
//                     if (dis > 0.45)// 上半身
//                     {
//                         // cout<<"dis = "<<dis<<endl;
//                         upper_body.at<uchar>(cv_p.y, cv_p.x) = 255;
//                     }
//                     else if (dis > 0.25) // 膝盖
//                     {
//                         // cout<<"dis = "<<dis<<endl;
//                         knee.at<uchar>(cv_p.y, cv_p.x) = 255;
//                     }
//                 }
//             }
//         }
//         // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/knee" + std::to_string(i) + ".png", knee);
//         // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/upper_body" + std::to_string(i) + ".png", upper_body);
//         // cv::imshow("knee", knee);
//         // cv::waitKey(0);
//         // cv::imshow("upper_body", upper_body);
//         // cv::waitKey(0);
//         // // 进行不同半径的膨胀
//         double upper_inflation = 0.4;
//         double knee_inflation = 0.2;
//         int inflation_radius_upper = upper_inflation/resolution;
//         int inflation_radius_knee = knee_inflation/resolution;
//         cv::Mat kernel_upper = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(inflation_radius_upper, inflation_radius_upper));
//         cv::Mat kernel_knee = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(inflation_radius_knee, inflation_radius_knee));
//         // // 对图像进行膨胀操作
//         cv::Mat dilated_image_upper;
//         cv::dilate(upper_body, dilated_image_upper, kernel_upper);
//         upper_body_image = upper_body;
//         upper_body_dilate = dilated_image_upper;
//         cv::Mat dilated_image_knee;
//         cv::dilate(knee, dilated_image_knee, kernel_knee);
//         knee_image = knee;
//         knee_image_dilate = dilated_image_knee;
//         // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/dilated_image_upper" + std::to_string(i) + ".png", dilated_image_upper);
//         // cv::imwrite("/home/bhr/TCDS/src/pip_line/data/dilated_image_knee" + std::to_string(i) + ".png", dilated_image_knee);
//         // cv::imshow("dilated_image_upper", dilated_image_upper);
//         // cv::waitKey(0);
//         // cv::imshow("dilated_image_knee", dilated_image_knee);
//         // cv::waitKey(0);
//         // 计算膨胀后的边缘
//         cv::Mat collision_layer1 = image - dilated_image_upper;
//         cv::Mat free_collision = collision_layer1 - dilated_image_knee;
//         // cv::imwrite("/home/lichao/TCDS/src/pip_line/data/free_collision" + std::to_string(i) + ".png", free_collision);
//         // cv::imshow("free_collision", free_collision);
//         // cv::waitKey(0);
//         collision_free_images.emplace_back(free_collision);
//     }
//     feasible_map = map;
//     map.add("label");
//     cv::Mat plane_image = cv::Mat::zeros(result.size(), CV_8UC3);
//     for (int i = 0; i < map.getSize().x(); i++)
//     {
//         for (int j = 0; j < map.getSize().y(); j++)
//         {
//             bool flag = false;
//             for (int label = 0; label < collision_free_images.size(); label++)
//             {
//                 if (collision_free_images.at(label).at<uchar>(i, j) == 255)
//                 {
//                     flag = true;
//                     map["label"](i ,j) = label;
//                     // 后续计算
//                     plane_image.at<cv::Vec3b>(i, j) = cv::Vec3b(default_colors[(static_cast<int>(label))%12][0], default_colors[(static_cast<int>(label))%10][1], default_colors[(static_cast<int>(label))%12][2]);
//                     break;
//                 }
//             }
//             if (!flag)
//             {
//                 map["label"](i,j) = NAN;
//                 feasible_map["elevation"](i, j) = NAN;
//             }
//         }
//     }
// }

void localPlannerBase::mapPrepare(grid_map::GridMap & map_)
{
    // LOG(INFO)<<"mapPrepare";
    map = map_;
    Inpaint(5);
    detectionPlane();
    mergePlanes();
#ifdef DEBUG
    for (auto & l : map.getLayers())
    {
        LOG(INFO)<<l;
    }
#endif
// #ifdef DEBUG
//     cv::imshow("seg_image", seg_image);
//     cv::waitKey(0);
//     for (auto & single_image : merge_results)
//     {
//         cv::imshow("merge_image", single_image);
//         cv::waitKey(0);
//     }
// #endif
    planner_P->setBasicInfor(map, seg_image, merge_results, merge_planes, foot_param, hip_width);
    // constructFeasibleMap();
}

// 必须在mapPrepare 之后。
bool localPlannerBase::isGoalFeasible(Eigen::Vector3d goal)
{
    if (!merge_results.empty() && !merge_planes.empty())
    {
        return planner_P->checkFeasibleGoal(goal);
    }
    else
    {
        return false;
    }
}

bool localPlannerBase::isStartFeasible(Eigen::Vector3d start, Eigen::Vector3d & left_foot, Eigen::Vector3d & right_foot)
{
    if (!merge_results.empty() && !merge_planes.empty())
    {
        if (planner_P->isStartFeasible(start, left_foot, right_foot))
        {
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
}

bool localPlannerBase::plan()
{
    // LOG(INFO)<<"IN";
    if (planner_P->initial(start, pre_start, support_flag, goal))
    {
#ifdef DEBUG
        LOG(INFO)<<"set start and goal";
#endif
        if (planner_P->plan())
        {
            // 记录结束时间
            // clock_t end_plan = clock();
            cout<<planner_P->time_consume<<endl;
            time_consume = planner_P->time_consume;
            // // 计算消耗的时间（毫秒）
            // double duration_plan = double(end_plan - start_plane) / CLOCKS_PER_SEC * 1000;

            // LOG(INFO)<<"TOTAL TIME: "<<duration_plan;
            steps = planner_P->getResultSteps();
            // LOG(INFO)<<"ERROR";
            // avoid_points = planner.computeAvoidPoints();
            // LOG(INFO)<<"ERROR";

            // for (auto & step : steps)
            // {
            //     cout<<setw(8)<<"step: "<<step.x<<" "<<step.y<<" "<<step.z<<" "<<step.roll<<" "<<step.pitch<<" "<<step.yaw*57.3<<" "<<step.robot_side<<endl;
            //     cout<<setw(8)<<"points: "<<
            // }
            // LOG(INFO)<<steps.size();
            // for (int i = 0; i < steps.size(); i++)
            // {
            //     cout<<setw(8)<<"result step "<<i<<": "<<steps.at(i).x<<" "<<steps.at(i).y<<" "<<steps.at(i).z<<" "<<steps.at(i).roll*57.3<<" "<<steps.at(i).pitch*57.3<<" "<<steps.at(i).yaw*57.3<<" "<<steps.at(i).robot_side<<endl;
            //     // cout<<"points: "<<endl;
            // }
            return true;
        }
        else
        {
            LOG(INFO)<<"planning error";
            return false;
        }
    }
    else
    {
        return false;
    }
}
#ifdef PLANNING_TIMER_CHECK
void localPlannerBase::cancelPlanning() 
{
    planner_P->cancelPlanning();
    // stop_flag.store(true); // 设置取消标志位
}
#endif
pcl::PointCloud<pcl::PointXYZ> localPlannerBase::gridMap2PointcloudOrganized()
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

localPlannerBase::~localPlannerBase()
{

}