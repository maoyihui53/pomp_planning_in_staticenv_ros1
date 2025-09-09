#pragma once

#include <iostream>
#include <vector>
#include <random>
#include <memory>
// #include <tbb/atomic.h>
// #include <tbb/tbb.h>
#include <utils/util.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range2d.h>

// #include <set>
// #include <immintrin.h>
// #include <octomap/octomap.h>
// #include <cstddef>
// #include <numeric>

namespace pomp_planning_grid
{
    class grid
    {
    private:
        std::unique_ptr<std::atomic<int16_t>[]> voxel_node;
        Eigen::Array3d map_center; // Center of the scan
        Eigen::Array3i map_dim;    // Size of the scan in each dimension
        Eigen::Array3d min_corner;
        Eigen::Array3d map_size;
        double resolution; // Resolution of the voxel grid

        size_t total_voxel_count; // Total number of voxels in the grid

    public:
        grid(const Eigen::Vector3d &map_center_, const Eigen::Vector3d &map_size_, double res,
             bool raycasting = false, bool verbose = false)
        {
            setup(map_center_, map_size_, res);
            initialize_map();
            // std::cout << "Grid Map initialized with center: " << map_center.transpose()
            //           << ", min_corner: " << min_corner.transpose() << ", resolution: " << resolution << std::endl;
            if (verbose)
            {
                print_info();
            }
        }

        void setup(const Eigen::Vector3d &center, const Eigen::Vector3d &size, double res)
        {
            map_center = center.array();
            map_dim = (map_center / res).ceil().cast<int>();
            map_size = size.array();
            resolution = res;

            Eigen::Array3d half_size = map_size / 2.0;
            Eigen::Array3d up_octree_size = half_size;
            Eigen::Array3d down_octree_size = -half_size;
            Eigen::Array3i min_corner_to_center_idx = (down_octree_size / res).floor().cast<int>();
            Eigen::Array3i max_corner_to_center_idx = (up_octree_size / res).ceil().cast<int>();

            map_dim = max_corner_to_center_idx - min_corner_to_center_idx + 3 * Eigen::Array3i::Ones(3);

            min_corner = map_center - (min_corner_to_center_idx.cast<double>() + 0.5f) * res;

            // map_dim = (map_size / resolution).ceil().cast<int>();

            min_corner = map_center - ((map_dim.cast<double>() / 2.f) * resolution);
            // std::cout << "Map Center: " << map_center.transpose() << std::endl;
            // std::cout << "Map Dimensions: " << map_dim.transpose() << std::endl;
            // std::cout << "Min Corner: " << min_corner.transpose() << std::endl;
        }

        void markBoundaryVoxelsTBB()
        {
            const int NX = map_dim[0];
            const int NY = map_dim[1];
            const int NZ = map_dim[2];

            // 1) z = 0 底面 和 z = NZ-1 顶面
            tbb::parallel_for(
                tbb::blocked_range2d<int, int>(0, NX, 0, NY),
                [&](auto r)
                {
                    for (int x = r.rows().begin(); x < r.rows().end(); ++x)
                    {
                        for (int y = r.cols().begin(); y < r.cols().end(); ++y)
                        {
                            // 底面 z=0
                            {
                                Eigen::Array3i idx(x, y, 0);
                                voxel_node[flatten3D(idx)] = 1;
                            }
                            // 顶面 z=NZ-1
                            {
                                Eigen::Array3i idx(x, y, NZ - 1);
                                voxel_node[flatten3D(idx)] = 1;
                            }
                        }
                    }
                });

            // 2) y = 0 前面 和 y = NY-1 后面
            tbb::parallel_for(
                tbb::blocked_range2d<int, int>(0, NX, 0, NZ),
                [&](auto r)
                {
                    for (int x = r.rows().begin(); x < r.rows().end(); ++x)
                    {
                        for (int z = r.cols().begin(); z < r.cols().end(); ++z)
                        {
                            // 前面 y=0
                            {
                                Eigen::Array3i idx(x, 0, z);
                                voxel_node[flatten3D(idx)] = 1;
                            }
                            // 后面 y=NY-1
                            {
                                Eigen::Array3i idx(x, NY - 1, z);
                                voxel_node[flatten3D(idx)] = 1;
                            }
                        }
                    }
                });

            // 3) x = 0 左面 和 x = NX-1 右面
            tbb::parallel_for(
                tbb::blocked_range2d<int, int>(0, NY, 0, NZ),
                [&](auto r)
                {
                    for (int y = r.rows().begin(); y < r.rows().end(); ++y)
                    {
                        for (int z = r.cols().begin(); z < r.cols().end(); ++z)
                        {
                            // 左面 x=0
                            {
                                Eigen::Array3i idx(0, y, z);
                                voxel_node[flatten3D(idx)] = 1;
                            }
                            // 右面 x=NX-1
                            {
                                Eigen::Array3i idx(NX - 1, y, z);
                                voxel_node[flatten3D(idx)] = 1;
                            }
                        }
                    }
                });
        }

        // void markBoundaryVoxelsSequential()
        // {
        //     const int NX = map_dim[0];
        //     const int NY = map_dim[1];
        //     const int NZ = map_dim[2];

        //     // 1) 底面 z = 0 和 顶面 z = NZ-1
        //     for (int x = 0; x < NX; ++x)
        //     {
        //         for (int y = 0; y < NY; ++y)
        //         {
        //             // 底面
        //             {
        //                 Eigen::Array3i idx(x, y, 0);
        //                 voxel_node[flatten3D(idx)] = 1;
        //             }
        //             // 顶面
        //             {
        //                 Eigen::Array3i idx(x, y, NZ - 1);
        //                 voxel_node[flatten3D(idx)] = 1;
        //             }
        //         }
        //     }

        //     // 2) 前面 y = 0 和 后面 y = NY-1
        //     for (int x = 0; x < NX; ++x)
        //     {
        //         for (int z = 0; z < NZ; ++z)
        //         {
        //             // 前面
        //             {
        //                 Eigen::Array3i idx(x, 0, z);
        //                 voxel_node[flatten3D(idx)] = 1;
        //             }
        //             // 后面
        //             {
        //                 Eigen::Array3i idx(x, NY - 1, z);
        //                 voxel_node[flatten3D(idx)] = 1;
        //             }
        //         }
        //     }

        //     // 3) 左面 x = 0 和 右面 x = NX-1
        //     for (int y = 0; y < NY; ++y)
        //     {
        //         for (int z = 0; z < NZ; ++z)
        //         {
        //             // 左面
        //             {
        //                 Eigen::Array3i idx(0, y, z);
        //                 voxel_node[flatten3D(idx)] = 1;
        //             }
        //             // 右面
        //             {
        //                 Eigen::Array3i idx(NX - 1, y, z);
        //                 voxel_node[flatten3D(idx)] = 1;
        //             }
        //         }
        //     }
        // }

        void initialize_map()
        {
            total_voxel_count = map_dim[0] * map_dim[1] * map_dim[2];
            // auto start_time = std::chrono::high_resolution_clock::now();
            voxel_node.reset(new std::atomic<int16_t>[total_voxel_count]);

            Eigen::Array3d min_map = -map_size / 2.0;

            Eigen::Array3d max_map = map_size / 2.0;

            tbb::parallel_for(size_t(0), total_voxel_count, [&](size_t i)
                              {
                Eigen::Array3i idx = unflatten3D(i);
                Eigen::Array3d center = gridToWorld(idx);
                // voxel_node[i].store(0, std::memory_order_relaxed); });
            // Check if the center is within the map bounds
            if ((center.array() >= min_map.array()).all() && (center.array() <= max_map.array()).all())
                voxel_node[i].store(0, std::memory_order_relaxed);
            else
                voxel_node[i].store(1, std::memory_order_relaxed); });

            markBoundaryVoxelsTBB();
            // markBoundaryVoxelsSequential();
        }

        void print_info()
        {
            std::cout << "Grid Map Info:" << std::endl;
            std::cout << " Map Center: " << map_center.transpose() << std::endl;
            std::cout << " Map Dimensions: " << map_dim.transpose() << std::endl;
            std::cout << " Min Corner: " << min_corner.transpose() << std::endl;
            std::cout << " Resolution: " << resolution << std::endl;
            std::cout << " Total Voxel Count: " << total_voxel_count << std::endl;
        }

        inline Eigen::Array3i worldToGrid(const Eigen::Array3d &pt)
        {
            return ((pt - min_corner) / resolution).floor().cast<int>();
        }

        inline Eigen::Array3d gridToWorld(const Eigen::Array3i &idx)
        {
            return min_corner + (idx.cast<double>() + 0.5f) * resolution;
        }

        inline size_t flatten3D(const Eigen::Array3i &idx)
        {
            // assume 0 ≤ x < NX, 0 ≤ y < NY, 0 ≤ z < NZ
            return idx[0] + idx[1] * map_dim[0] + idx[2] * (map_dim[0] * map_dim[1]);
        }

        inline Eigen::Array3i unflatten3D(size_t idx)
        {
            size_t nx = map_dim[0];
            size_t ny = map_dim[1];

            Eigen::Array3i index;

            index[2] = idx / (nx * ny);
            size_t rem = idx % (nx * ny);
            index[1] = rem / nx;
            index[0] = rem % nx;

            return index;
        }

        // inline const std::vector<std::atomic<int>> &getVoxelNode() const
        // {
        //     return voxel_node;
        // }

        inline double getResolution() const
        {
            return resolution;
        }
        inline std::atomic<int16_t> *getVoxelNode() const
        {
            return voxel_node.get();
        }

        inline Eigen::Array3d getMapCenter() const
        {
            return map_center;
        }

        inline Eigen::Array3i getMapDim() const
        {
            return map_dim;
        }

        inline Eigen::Array3d getMinCorner() const
        {
            return min_corner;
        }
        inline Eigen::Array3d getMaxCorner() const
        {
            return min_corner + map_dim.cast<double>() * resolution;
        }

        inline size_t getTotalVoxelCount() const
        {
            return total_voxel_count;
        }

        void updateOccupiedVoxelNodeCount(const Eigen::Array3d &pt)
        {
            size_t idx = flatten3D(worldToGrid(pt));
            voxel_node[idx].fetch_add(1);
        }

        void set_buffer(const Eigen::Array3d &pt)
        {
            size_t idx = flatten3D(worldToGrid(pt));
            voxel_node[idx] = 1;
        }

        bool isfree(const Eigen::Array3d &pt)
        {
            size_t idx = flatten3D(worldToGrid(pt));
            return voxel_node[idx].load(std::memory_order_relaxed) == 0;
        }
    };
}