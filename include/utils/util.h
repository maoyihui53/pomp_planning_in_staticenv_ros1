#pragma once
#include <memory>
#include <Eigen/Dense>
#include <iostream>
#include <chrono>

#include <algorithm>
#include <random>
#include <vector>

#ifndef ANSI_COLOR_RED
#define ANSI_COLOR_RED "\x1b[1;31m"
#endif
/// Set green font in printf funtion
#ifndef ANSI_COLOR_GREEN
#define ANSI_COLOR_GREEN "\x1b[1;32m"
#endif
/// Set yellow font in printf funtion
#ifndef ANSI_COLOR_YELLOW
#define ANSI_COLOR_YELLOW "\x1b[1;33m"
#endif
/// Set blue font in printf funtion
#ifndef ANSI_COLOR_BLUE
#define ANSI_COLOR_BLUE "\x1b[1;34m"
#endif
/// Set magenta font in printf funtion
#ifndef ANSI_COLOR_MAGENTA
#define ANSI_COLOR_MAGENTA "\x1b[1;35m"
#endif
/// Set cyan font in printf funtion
#ifndef ANSI_COLOR_CYAN
#define ANSI_COLOR_CYAN "\x1b[1;36m"
#endif
/// Reset font color in printf funtion
#ifndef ANSI_COLOR_RESET
#define ANSI_COLOR_RESET "\x1b[0m"
#endif

namespace pomp_type
{

    enum State
    {
        Free,
        Uncertain,
        Occupied,
        Unexplored
    };




}

namespace pomp_env
{

    struct env_info
    {

        Eigen::Matrix3Xd points_array; // Using Eigen for better performance, points after adjustment to center
        Eigen::Vector3d min_corner;    /// Minimum corner of the bounding box of the point cloud before adjustment (raw data)
        Eigen::Vector3d max_corner;    /// Maximum corner of the bounding box of the point cloud before adjustment (raw data)
        Eigen::Vector3d center;        // Center of the point cloud before adjustment

        int depth = 0;              // Octree depth
        double octree_size = 0.0;   // Size of the octree at the given depth
        Eigen::Vector3d map_size;   // Size of the map in 3D space
        Eigen::Vector3d cur_center; // Current center
        double resolution = 0.1;    // Resolution of the octree, default is 0.1

        env_info()
            : points_array(Eigen::Matrix3Xd::Zero(3, 0)),
              min_corner(Eigen::Vector3d::Zero()),
              max_corner(Eigen::Vector3d::Zero()), cur_center(Eigen::Vector3d::Zero())
        {
        }

        void adjust_to_center()
        {

            points_array.row(0).array() -= center(0);
            points_array.row(1).array() -= center(1);
            points_array.row(2).array() -= center(2);
            // std::cout <<"min1:"<<  points_array.rowwise().minCoeff()<<std::endl;
            // std::cout <<"max1:"<<  points_array.rowwise().maxCoeff()<<std::endl;
            // std::cout <<"min2:" <<min_corner-center<< std::endl;
            // std::cout <<"max2:" << max_corner-center<< std::endl;
        }

        void shuffle()
        {
            const int group_size = 1;
            const int num_groups = points_array.cols() / group_size;

            // 初始化组索引：0, 1, 2, ..., num_groups-1
            std::vector<int> group_indices(num_groups);
            std::iota(group_indices.begin(), group_indices.end(), 0);

            // 真随机种子初始化
            std::random_device rd;
            std::mt19937 g(rd());

            // 多次洗牌增强随机性（比如洗牌 5 次）
            int shuffle_rounds = 1;
            for (int i = 0; i < shuffle_rounds; ++i)
            {
                std::shuffle(group_indices.begin(), group_indices.end(), g);
            }

            // // 打印洗牌后的组顺序
            // std::cout << "Shuffled group order: ";
            // for (int idx : group_indices)
            //     std::cout << idx << " ";
            // std::cout << "\n\n";

            // 创建新点云矩阵存储打乱后的结果
            Eigen::Matrix3Xd shuffled(3, points_array.cols());
            for (int i = 0; i < num_groups; ++i)
            {
                int src_start = group_indices[i] * group_size;
                int dst_start = i * group_size;
                shuffled.block(0, dst_start, 3, group_size) = points_array.block(0, src_start, 3, group_size);
            }
            points_array.swap(shuffled);
        }

        void computeMinDepth(double res)
        {

            resolution = res;

            Eigen::Vector3d size = max_corner - min_corner;
            double root_size = std::max({size[0], size[1], size[2]});
            double required_ratio = root_size / resolution;
            depth = static_cast<int>(std::ceil(std::log2(required_ratio)));
            octree_size = std::pow(2, depth) * resolution;
            map_size = max_corner - min_corner;

            // std::cout << "points size " << points_array.cols() << std::endl
            //           << "Computed octree depth: " << depth
            //           << ", octree size: " << octree_size
            //           << ", map size: " << map_size.transpose()
            //           << ", resolution: " << resolution << std::endl;
        }
    };

}