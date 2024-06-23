#include <iostream>
#include <memory>
#include <vector>
#include <cassert>
#include <limits>
#include <Eigen/Dense>  // 假设你在使用 Eigen 库

class Node : public std::enable_shared_from_this<Node> {
public:
    // 构造函数，进行基本初始化
    Node(size_t start_rows_, size_t start_cols_, size_t width_, size_t start_rows_2d_, size_t start_cols_2d_, size_t width_2d_, size_t depth_, std::shared_ptr<Node> parent_)
        : start_rows(start_rows_), start_cols(start_cols_), width(width_),
          start_rows_2d(start_rows_2d_), start_cols_2d(start_cols_2d_), width_2d(width_2d_),
          depth(depth_), parent(parent_) 
    {
        assert(start_rows_ >= 0);
        assert(start_rows_ < data_height);
        assert(start_cols_ >= 0);
        assert(start_cols_ < data_width);  // width 计数方式与序号方式不一样
        assert(start_rows_ + width_ <= quatree_width);
        assert(start_cols_ + width_ <= quatree_width);

        neighbors.clear();
        // 将其余逻辑移到 initialize 函数中
    }

    // 初始化函数，调用 shared_from_this 进行进一步初始化
    void initialize() 
    {
        if (depth == param.leafnode_depth) 
        {
            // Leaf node specific initialization
            is_leafnode = true;
            valid_points = raw_points.getRectPoint(start_rows, start_cols, width, width);
            valid_points_size = valid_points.size();
            initial_size = valid_points_size;

            if (valid_points_size >= param.patch_num_th) 
            {
                is_validnode = true;
                stats.push(valid_points);
                stats.compute(center, normal, mse, curvature);

                if (curvature < param.eigen_value_th && mse < param.patch_mse_th) 
                {
                    if (check_normal.dot(normal) > 0.866) 
                    {
                        is_plane = true;
                    }
                } 
                else 
                {
                    resetNode();
                }
            }
            else 
            {
                is_validnode = false;
            }
        } 
        else 
        {
            is_leafnode = false;
            is_plane = false;
            createChildren();
        }
    }

    static std::shared_ptr<Node> create(size_t start_rows_, size_t start_cols_, size_t width_, size_t start_rows_2d_, size_t start_cols_2d_, size_t width_2d_, size_t depth_, std::shared_ptr<Node> parent_) 
    {
        std::shared_ptr<Node> node = std::make_shared<Node>(start_rows_, start_cols_, width_, start_rows_2d_, start_cols_2d_, width_2d_, depth_, parent_);
        node->initialize();  // 创建对象后立即调用初始化函数
        return node;
    }

private:
    void createChildren() 
    {
        size_t child_width = width >> 1;
        size_t child_width_2d = width_2d >> 1;

        for (size_t i = 0; i < 4; i++) 
        {
            size_t child_start_rows = start_rows + child_width * (i >> 1 & 1);
            size_t child_start_cols = start_cols + child_width * (i & 1);
            size_t child_start_rows_2d = start_rows_2d + child_width_2d * (i >> 1 & 1);
            size_t child_start_cols_2d = start_cols_2d + child_width_2d * (i & 1);

            if (child_start_rows >= data_height || child_start_cols >= data_width) 
            {
                children.at(i) = nullptr;
            } 
            else 
            {
                std::shared_ptr<Node> this_ptr = shared_from_this();
                children.at(i) = Node::create(child_start_rows, child_start_cols, child_width, child_start_rows_2d, child_start_cols_2d, child_width_2d, depth + 1, this_ptr);
            }
        }
    }

    void resetNode() 
    {
        center = Eigen::Vector3f::Zero();
        normal = Eigen::Vector3f::Zero();
        mse = FLT_MAX;
        curvature = 0.0;
    }

    // 成员变量声明
    size_t start_rows, start_cols, width, start_rows_2d, start_cols_2d, width_2d, depth;
    std::shared_ptr<Node> parent;
    std::vector<std::shared_ptr<Node>> children;
    std::vector<Eigen::Vector3f> valid_points;
    size_t valid_points_size, initial_size;
    bool is_leafnode, is_validnode, is_plane;
    Eigen::Vector3f center, normal;
    float mse, curvature;

    // 假设有一个名为 `Stats` 的类和 `param` 结构体，以及 `raw_points` 和 `check_normal` 对象
    Stats stats;
    static Param param;
    static RawPoints raw_points;
    static Eigen::Vector3f check_normal;
    static size_t data_height, data_width, quatree_width;
};

// 静态成员变量初始化
Param Node::param;
RawPoints Node::raw_points;
Eigen::Vector3f Node::check_normal;
size_t Node::data_height;
size_t Node::data_width;
size_t Node::quatree_width;

int main() {
    std::shared_ptr<Node> root = Node::create(0, 0, param.quatree_width, 0, 0, param.quatree_width / param.leafnode_width, 0, nullptr);
    return 0;
}
