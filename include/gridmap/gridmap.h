#pragma once
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <algorithm> // for std::shuffle
#include <display/display.h>
#include <utils/util.h>
#include <random>

#include <optional>

namespace gridMap
{
    using namespace pomp_type;
    using namespace pomp_env;

    class GridMap
    {
    private:
        env_info *env_info_ptr;
        Eigen::Vector3i grid_dim;         // Size of the grid map in 3D space
        Eigen::Vector3d min_corner;       // Minimum corner of the grid map in 3D space
        std::vector<signed char> gridMap; // Points in the grid map
        double resolution;                // Resolution of the grid map

    public:
        GridMap(const Eigen::Vector3d &min_corner, const Eigen::Vector3i &grid_dim, env_info *env, bool verbose = false)
            : min_corner(min_corner), grid_dim(grid_dim), resolution(env->resolution),
              env_info_ptr(env)
        {
            std::cout << "grid dim:" << grid_dim.transpose() << std::endl;

            computeGridSize();

            if (verbose)
            {
                print_info();
            }
        }

        GridMap(env_info *env, bool verbose = false)
            : env_info_ptr(env)
        {
            resolution = env_info_ptr->resolution;
            computeMinCorner(env_info_ptr->cur_center, env_info_ptr->map_size);
            computeGridSize(env_info_ptr->map_size, env_info_ptr->resolution);

            // std::cout << " benchmark GridMap initialized with min_corner: " << min_corner.transpose()
            //           << ", grid_dim: " << grid_dim.transpose() << ", resolution: " << resolution << std::endl;

            if (verbose)
            {
                print_info();
            }
        }

        GridMap(env_info *env, double res, bool verbose = false)
            : env_info_ptr(env)
        {
            resolution = res;
            computeMinCorner(env_info_ptr->cur_center, env_info_ptr->map_size);
            computeGridSize(env_info_ptr->map_size, res);

            if (verbose)
            {
                print_info();
            }
        }

        void computeMinCorner(const Eigen::Vector3d &center, const Eigen::Vector3d &map_size)
        {
            min_corner = center - map_size / 2.0;
        }

        void computeGridSize(const Eigen::Vector3d &map_size, double resolution)
        {

            for (int i = 0; i < 3; ++i)
            {
                grid_dim[i] = static_cast<int>(std::ceil(map_size[i] / resolution));
            }

            // std::cout << "...." << std::endl;
            // std::cout << "GridMap grid dim: " << grid_dim.transpose() << std::endl;
            // std::cout << "GridMap resolution: " << resolution << std::endl;
            // std::cout << "GridMap min corner: " << min_corner.transpose() << std::endl;

            size_t N = grid_dim[0] * grid_dim[1] * grid_dim[2];
            gridMap.resize(N);

            // 2) 计算真实地图的边界（以世界坐标为准）
            //    假设 map_size 是沿 x,y,z 方向的长度向量
            Eigen::Array3d half_size = map_size * 0.5;
            // 中心对称，体素中心需要加上 half resolution
            Eigen::Array3d min_map = -half_size + Eigen::Array3d::Constant(resolution * 0.5);
            Eigen::Array3d max_map = half_size - Eigen::Array3d::Constant(resolution * 0.5);

            // std::cout << "GridMap min map: " << min_map.transpose() << std::endl;
            // std::cout << "GridMap max map: " << max_map.transpose() << std::endl;

            // 3) 对每一个体素进行判断
            for (size_t i = 0; i < N; ++i)
            {
                Eigen::Array3i idx = unflatten3D(i);
                Eigen::Array3d center = gridToWorld(idx);
                // 如果中心在真实地图范围内，就 free(0)，否则 out‐of‐bounds(1)
                bool in =
                    (center.array() >= min_map.array()).all() &&
                    (center.array() <= max_map.array()).all();
                gridMap[i] = in ? 0 : 1;
            }
        }

        void computeGridSize()
        {
            Eigen::Array3d map_size = env_info_ptr->map_size.array();

            Eigen::Array3d min_map = -map_size / 2.0;
            Eigen::Array3d max_map = map_size / 2.0;

            // tbb::parallel_for(size_t(0), total_voxel_count, [&](size_t i)
            //                   {
            //                         Eigen::Array3i idx = unflatten3D(i);
            //                         Eigen::Array3d center = gridToWorld(idx);
            //                         // Check if the center is within the map bounds
            //                         if ((center.array() >= min_map.array()).all() && (center.array() <= max_map.array()).all())
            //                             voxel_node[i].store(0, std::memory_order_relaxed);
            //                         else
            //                             voxel_node[i].store(1, std::memory_order_relaxed); });
            gridMap.resize(grid_dim[0] * grid_dim[1] * grid_dim[2], 0);

            for (size_t i = 0; i < gridMap.size(); ++i)
            {
                Eigen::Vector3i idx = unflatten3D(i);
                Eigen::Vector3d center = gridToWorld(idx);
                // Check if the center is within the map bounds
                if ((center.array() >= min_map.array()).all() && (center.array() <= max_map.array()).all())
                    gridMap[i] = 0; // Mark as free
                else
                    gridMap[i] = 1; // Mark as out of bounds
            }
        }

        inline void print_info()
        {
            std::cout << "GridMap Info:" << std::endl;
            std::cout << "  Resolution: " << resolution << std::endl;
            std::cout << "  Grid dim: " << grid_dim.transpose() << std::endl;
            std::cout << "  Min Corner: " << min_corner.transpose() << std::endl;
        }

        inline Eigen::Vector3i worldToGrid(const Eigen::Vector3d &pt)
        {

            return ((pt - min_corner) / resolution).array().floor().cast<int>();
        }

        inline Eigen::Vector3d gridToWorld(const Eigen::Vector3i &idx)
        {
            return min_corner + (idx.cast<double>() + Eigen::Vector3d::Constant(0.5)) * resolution;
            // return min_corner + (idx.cast<double>() + 0.5) * resolution;
        }

        inline size_t flatten3D(const Eigen::Vector3i &idx)
        {
            // assume 0 ≤ x < NX, 0 ≤ y < NY, 0 ≤ z < NZ
            return idx[0] + idx[1] * grid_dim[0] + idx[2] * (grid_dim[0] * grid_dim[1]);
        }

        inline Eigen::Vector3i unflatten3D(size_t idx)
        {

            size_t nx = grid_dim[0];
            size_t ny = grid_dim[1];

            Eigen::Vector3i index;

            index[2] = idx / (nx * ny);
            size_t rem = idx % (nx * ny);
            index[1] = rem / nx;
            index[0] = rem % nx;

            return index;
        }

        void insertPcd(const Eigen::Matrix3Xd &pts)
        {
            int size = pts.cols();

            for (int i = 0; i < size; ++i)
            {
                Eigen::Vector3d pt(pts(0, i), pts(1, i), pts(2, i));
                Eigen::Vector3i grid_idx = worldToGrid(pt);

                if (grid_idx[0] >= 0 && grid_idx[0] < grid_dim[0] &&
                    grid_idx[1] >= 0 && grid_idx[1] < grid_dim[1] &&
                    grid_idx[2] >= 0 && grid_idx[2] < grid_dim[2])
                {
                    size_t flat_index = flatten3D(grid_idx);
                    if (flat_index < gridMap.size())
                    {
                        gridMap[flat_index] = 1; // Mark the point as occupied
                    }
                }
            }
        }

        inline const std::vector<signed char> &getGridMap() const
        {
            return gridMap;
        }

        inline Eigen::Vector3d getMinCorner() const
        {
            return min_corner;
        }

        inline Eigen::Vector3i getMapDim() const
        {
            return grid_dim;
        }

        inline double getResolution() const
        {
            return resolution;
        }

        std::optional<Eigen::Vector3d> findfree(const std::vector<signed char> &gridMap)
        {
            if (gridMap.empty())
                return std::nullopt;

            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<size_t> dist(0, gridMap.size() - 1);

            const size_t maxAttempts = gridMap.size() * 3; // 最多尝试两倍长度次
            for (size_t attempt = 0; attempt < maxAttempts; ++attempt)
            {
                size_t idx = dist(gen);
                if (gridMap[idx] == 0)
                {
                    Eigen::Vector3i idx_3d = unflatten3D(idx);
                    Eigen::Vector3d pt = gridToWorld(idx_3d);
                    // if (pt[2] < -15 && pt[2] > -25)
                    return pt;
                }
            }
            return std::nullopt; // 超过尝试次数仍未找到
        }

        inline bool isfree(Eigen::Vector3d pt)
        {
            Eigen::Vector3i grid_idx = worldToGrid(pt);

            if (grid_idx[0] < 0 || grid_idx[0] >= grid_dim[0] ||
                grid_idx[1] < 0 || grid_idx[1] >= grid_dim[1] ||
                grid_idx[2] < 0 || grid_idx[2] >= grid_dim[2])
            {
            std:;
                // cout << "Point out of bounds: " << pt.transpose() << std::endl;
                return false; // 超出边界
            }

            size_t flat_index = flatten3D(grid_idx);

            // std::cout << "Checking point: " << pt.transpose() << ", flat index: " << flat_index << std::endl;
            // std::cout << "grid_idx: " << grid_idx.transpose() << std::endl;
            // if (gridMap[flat_index] == 0)
            //     std::cout << "gridMap[flat_index]: 0" << std::endl;
            // else if (gridMap[flat_index] == 1)
            //     std::cout << "gridMap[flat_index]: 1" << std::endl;
            // else
            //     std::cout << "gridMap[flat_index]: " << (int)gridMap[flat_index] << std::endl;

            return (flat_index < gridMap.size() && gridMap[flat_index] == 0);
        }

        bool find_free_voxel(env_info *env, Eigen::Vector3d &result)
        {
            Eigen::Vector3d eps = Eigen::Vector3d::Constant(1e-6);
            Eigen::Vector3d min_corner = env->cur_center - env->map_size / 2.0 + eps;
            Eigen::Vector3d max_corner = env->cur_center + env->map_size / 2.0 - eps;

            // min_corner[2] = -25;
            // max_corner[2] = -15;

            static std::random_device rd;
            static std::mt19937 gen(rd());

            const int maxtrial = 100;
            for (int i = 0; i < maxtrial; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    std::uniform_real_distribution<double> dist(min_corner[j], max_corner[j]);
                    result[j] = dist(gen);
                }

                if (isfree(result))
                    return true;
            }

            return false; // failed after maxtrial attempts
        }

        ~GridMap()
        {
            // env_info_ptr will be automatically deleted
        }
    };
}
