#pragma once
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <algorithm> // for std::shuffle
#include <display/display.h>
#include <utils/util.h>
#include <jps_basis/data_utils.h>

namespace collisionchecker
{
    using namespace pomp_type;
    using namespace pomp_env;

    class CollisionChecker
    {
    private:
        env_info *env_info_ptr;
        Eigen::Vector3i grid_size;        // Size of the grid map in 3D space
        Eigen::Vector3d min_corner;       // Minimum corner of the grid map in 3D space
        std::vector<signed char> gridMap; // Points in the grid map
        double resolution;                // Resolution of the grid map

    public:
        CollisionChecker(env_info *env, double res = 0.1, bool verbose = false)
            : env_info_ptr(env)
        {

            computeGridSize(env_info_ptr->map_size, res);
            computeMinCorner(env_info_ptr->cur_center, env_info_ptr->map_size);
            resolution = res;

            if (verbose)
            {
                print_info();
            }

            // auto start = std::chrono::high_resolution_clock::now();
            // insertPcd(env->points_array);
            // auto end = std::chrono::high_resolution_clock::now();
            // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            // std::cout << "grid map Insertion completed in " << duration.count() << " ms" << std::endl;
        }

        void computeGridSize(const Eigen::Vector3d &map_size, double resolution)
        {

            for (int i = 0; i < 3; ++i)
            {
                grid_size[i] = static_cast<int>(std::ceil(map_size[i] / resolution));
            }

            gridMap.resize(grid_size[0] * grid_size[1] * grid_size[2], 0);
        }

        void computeMinCorner(const Eigen::Vector3d &center, const Eigen::Vector3d &map_size)
        {
            min_corner = center - map_size / 2.0;
        }

        inline void print_info()
        {
            std::cout << "GridMap Info:" << std::endl;
            std::cout << "  Center: " << env_info_ptr->cur_center.transpose() << std::endl;
            std::cout << "  map size " << env_info_ptr->map_size << std::endl;
            std::cout << "  Resolution: " << resolution << std::endl;
            std::cout << "  Grid Size: " << grid_size.transpose() << std::endl;
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
            return idx[0] + idx[1] * grid_size[0] + idx[2] * (grid_size[0] * grid_size[1]);
        }

        inline Eigen::Vector3i unflatten3D(size_t idx)
        {

            size_t nx = grid_size[0];
            size_t ny = grid_size[1];

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

                if (grid_idx[0] >= 0 && grid_idx[0] < grid_size[0] &&
                    grid_idx[1] >= 0 && grid_idx[1] < grid_size[1] &&
                    grid_idx[2] >= 0 && grid_idx[2] < grid_size[2])
                {
                    size_t flat_index = flatten3D(grid_idx);
                    if (flat_index < gridMap.size())
                    {
                        gridMap[flat_index] = 1; // Mark the point as occupied
                    }
                }
            }
        }

        bool CollisionCheck(const vec_Vec3f &path, float segmentResolution = 0.1f)
        {

            if (path.size() < 2)
                return true;

            const float step = segmentResolution;
            for (size_t i = 1; i < path.size(); ++i)
            {
                const Eigen::Vector3d start = {path[i - 1][0], path[i - 1][1], path[i - 1][2]};
                const Eigen::Vector3d end = {path[i][0], path[i][1], path[i][2]};
                Eigen::Vector3d delta = end - start;
                float length = delta.norm();
                if (length < 1e-6f)
                    continue;                         // 跳过几乎重合的点
                Eigen::Vector3d dir = delta / length; // 单位方向

                // std::cout << "start:" << start[0] << " " << start[1] << " " << start[2] << std::endl;
                // std::cout << "end:" << end[0] << " " << end[1] << " " << end[2] << std::endl;

                // 计算需要多少个步进（向上取整）
                int numSteps = static_cast<int>(std::ceil(length / step));
                for (int s = 0; s <= numSteps; ++s)
                {
                    // 采样位置，最后一个点用 end 保证精确

                    float t = std::min(s * step, length);
                    Eigen::Vector3d sample = start + dir * t;

                    Eigen::Vector3i grid_idx = worldToGrid(sample);

                    if (grid_idx[0] >= 0 && grid_idx[0] < grid_size[0] &&
                        grid_idx[1] >= 0 && grid_idx[1] < grid_size[1] &&
                        grid_idx[2] >= 0 && grid_idx[2] < grid_size[2])
                    {
                        size_t id_ = flatten3D(grid_idx);

                        if (gridMap[id_] == 1)
                        {
                            std::cout << "pt" << sample[0] << " " << sample[1] << " " << sample[2] << std::endl;
                            std::cout << "collision" << std::endl;
                            return false;
                        }
                        // else{
                        //     std::cout<<"---------------"<<std::endl;
                        // }
                    }
                }
            }

            std::cout << "safe" << std::endl;
            return true; // 没有碰撞
        }

        inline bool isfree(Eigen::Vector3d pt)
        {
            Eigen::Vector3i grid_idx = worldToGrid(pt);

            if (grid_idx[0] < 0 || grid_idx[0] >= grid_size[0] ||
                grid_idx[1] < 0 || grid_idx[1] >= grid_size[1] ||
                grid_idx[2] < 0 || grid_idx[2] >= grid_size[2])
            {
                return false; // 超出边界
            }

            size_t flat_index = flatten3D(grid_idx);
            return (flat_index < gridMap.size() && gridMap[flat_index] == 0);
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
            return grid_size;
        }

        inline double getResolution() const
        {
            return resolution;
        }

        // ~GridMap()
        // {
        //     // env_info_ptr will be automatically deleted
        // }
    };
}