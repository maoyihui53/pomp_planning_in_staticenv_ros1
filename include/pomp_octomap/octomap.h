#pragma once
#include <tbb/enumerable_thread_specific.h>
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <algorithm> // for std::shuffle
#include <tbb/parallel_for.h>
#include <tbb/parallel_invoke.h>
// #include <tbb/task_scheduler_init.h>
// #include <tbb/tbb.h>
#include <atomic>
#include <pomp_octomap/grid.h>
#include <display/display.h>
#include <utils/util.h>

namespace pomp_planning_octomap
{
    using namespace pomp_type;
    using namespace pomp_env;
    using namespace pomp_planning_display;

    std::chrono::milliseconds duration;

    class OctreeNode
    {
    public:
        tbb::concurrent_vector<std::array<double, 3>> points;
        std::array<std::atomic<OctreeNode *>, 8> children{nullptr};
        Eigen::Array3d center;
        double size;
        State state = Unexplored;                 // State of the node
        tbb::atomic<bool> current_treeNode{true}; // 是否是当前树节点

        std::atomic<uint16_t> safe_state{0}; // 16位状态：高8位表示 visited，低8位表示 unsafe

        OctreeNode(double size_) : center(Eigen::Array3d::Zero()), size(size_)
        {
        }

        OctreeNode(const Eigen::Array3d &center_, double size_) : size(size_), center(center_)

        {
        }

        void setcurrent_treeNode(bool flag)
        {
            current_treeNode = flag;
        }

        ~OctreeNode()
        {
            // 递归 delete 子节点
            for (auto &p : children)
            {
                OctreeNode *ch = p.load(std::memory_order_relaxed);
                if (ch)
                    delete ch;
            }
        }

        bool isLeaf() const
        {
            for (const auto &child : children)
                if (child)
                    return false;
            return true;
        }

        void setCenter(const Eigen::Array3d &new_center)
        {
            center = new_center;
        }

        inline const Eigen::Array3d &getCenter() const
        {
            return center;
        }
        inline double getSize() const
        {
            return size;
        }
    };

    class OcTree
    {
    private:
        std::unique_ptr<OctreeNode> root;
        double resolution;
        env_info *env_info_ptr;
        std::unique_ptr<pomp_planning_grid::grid> grid_map;

        // std::unique_ptr<pomp_planning_grid::grid> grid_map_original;

        // std::unique_ptr<raycasting3D> raycaster;

        // std::unique_ptr<GlobalGrid> global_grid;
    public:
        OcTree(env_info *env, bool verbose = false)
            : resolution(env->resolution),
              env_info_ptr(env)
        {
            // Initialize the octree with the environment info
            root = std::make_unique<OctreeNode>(env->cur_center, env->octree_size);
            if (verbose)
            {
                print_info();
            }
            // std::cout << "resolutiion<< " << env->resolution << std::endl;
            grid_map = std::make_unique<pomp_planning_grid::grid>(env->cur_center, env->map_size, env->resolution);
            // grid_map_original = std::make_unique<pomp_planning_grid::grid>(env->cur_center, env->map_size, env->resolution);

            // if (to_grid)
            // {
            //     this->to_grid = to_grid;
            //     // Initialize the grid map
            //     // grid_map = std::make_unique<pomp_grid::grid>(env->cur_center, env->map_size, env->resolution);
            // }
        }

        inline void get_bias(int index, Eigen::Array3d &bias, double quat)
        {
            bias = {(index & 4) ? quat : -quat,
                    (index & 2) ? quat : -quat,
                    (index & 1) ? quat : -quat};
        }

        void insertIter(OctreeNode *node, const Eigen::Array3d &pt, const Eigen::Array3d &center, double size)
        {
            // OctreeNode *node = root;
            // double curX = x, curY = y, curZ = z;

            Eigen::Array3d curCenter = center;
            double curSize = size;
            Eigen::Array3d bias;

            while (curSize > this->resolution)
            {
                int idx = (pt[0] >= curCenter[0] ? 4 : 0) | (pt[1] >= curCenter[1] ? 2 : 0) | (pt[2] >= curCenter[2] ? 1 : 0);
                double half = curSize * 0.5;
                get_bias(idx, bias, 0.5 * half);

                // 2) 第一次原子读取
                OctreeNode *child = node->children[idx]
                                        .load(std::memory_order_acquire);
                if (!child)
                {
                    // 3) 构造自己的节点
                    // State state_;
                    // if (occupied)
                    //     state_ = Occupied;
                    // else
                    //     state_ = Free;

                    OctreeNode *newNode = new OctreeNode(curCenter + bias, half);

                    // 4) 尝试把 newNode 放进去
                    //    expected = nullptr，如果当前还是 nullptr，就交换成 newNode
                    if (!node->children[idx].compare_exchange_strong(
                            /* expected */ child,
                            /* desired  */ newNode,
                            std::memory_order_release,
                            std::memory_order_relaxed))
                    {
                        // 失败：说明 child 被别的线程先设置了，丢弃自己
                        delete newNode;
                    }
                    else
                    {
                        // 成功：child 现在就是 newNode
                        child = newNode;
                        // child->setcurrent_treeNode(true);
                    }
                }

                // 5) 继续钻下去
                node = child;
                // node->setcurrent_treeNode(true);
                curCenter += bias;
                curSize = half;
            }

            set_safe(node, pt);
            // grid_map_original->updateOccupiedVoxelNodeCount(pt);
            // if (!to_grid)
            // {
            //     node->points.push_back({pt[0], pt[1], pt[2]});
            // }
            // else
            // {
            //     // grid_map->updateOccupiedVoxelNodeCount(pt);
            //     set_safe(node, pt);
            // }
        }

        inline void set_safe(OctreeNode *node, const Eigen::Array3d &pt, double ratio = 0.75)
        {

            // std::cout << "size: " << node->getSize() << std::endl;
            Eigen::Array3d curCenter = node->getCenter();
            double half_size = node->getSize() * 0.5;

            int idx = (pt[0] >= curCenter[0] ? 4 : 0) | (pt[1] >= curCenter[1] ? 2 : 0) | (pt[2] >= curCenter[2] ? 1 : 0);

            // 高8位：visited
            uint16_t mask = (1 << (idx + 8));

            // 如果你允许延迟写，可以先聚合后再 fetch_or
            node->safe_state.fetch_or(mask, std::memory_order_relaxed);

            // // 判断 unsafe，用平方距离比较
            if (!((abs(pt[0] - curCenter[0]) < half_size * ratio) && (abs(pt[1] - curCenter[1]) < half_size * ratio) && (abs(pt[2] - curCenter[2]) < half_size * ratio)))
            // 至少有一维超出 halfSiz
            {
                uint16_t unsafe_mask = (1 << idx);
                node->safe_state.fetch_or(unsafe_mask, std::memory_order_relaxed);
            }
        }

        /*

         / 0 / 4 /
        / 2 / 6 /
                            ___ x
         / 1 / 5 /        /|
        / 3 / 7 /        y  z

        */
        void set_map(OctreeNode *node, const Eigen::Array3d &pos, const Eigen::Array3d bias, int cur_idx, int ops_idx)
        {

            if ((node->safe_state & (1 << (cur_idx + 8))) == 0) // without obs
            {

                if ((node->safe_state & (1 << (ops_idx + 8))) != 0) // opps with obs, set opps occupied
                {
                    // std::cout << "set_buffer: " << pos + bias << std::endl;
                    grid_map->set_buffer(pos + bias);
                }
            }
            else // with obs
            {
                if ((node->safe_state & (1 << cur_idx)) == 0) // with obs but safe
                {

                    if ((node->safe_state & (1 << (ops_idx + 8))) == 0) // if opps without obs, set cur occupied
                    {
                        // std::cout << "set_buffer: " << pos - bias << std::endl;
                        grid_map->set_buffer(pos - bias);
                    }
                    else // if opps with obs, set opps  occupied
                    {
                        // std::cout << "set_buffer: " << pos + bias << std::endl;
                        grid_map->set_buffer(pos + bias);
                    }
                }
                else // with obs but unsafe
                {
                    if ((node->safe_state & (1 << (ops_idx))) != 0) // if opps with obs and unsafe, set both occupied
                        grid_map->set_buffer(pos + bias);
                    grid_map->set_buffer(pos - bias);
                }
            }

            //  std::atomic<uint8_t> bits(0);

            // if (node->sub_occupied[cur_idx] == 0) // x: - y: -  without obs
            // {

            //     if (node->sub_occupied[ops_idx] > 0) // x: + y: + with obs
            //     {

            //         gridMap->set_buffer(pos + bias);
            //     }
            // }
            // else if (node->sub_occupied[cur_idx] == 1) // x: - y: -  with obs but safe
            // {
            //     // std::cout << "++" << std::endl;

            //     if (node->sub_occupied[ops_idx] == 0) // x: + y: + without obs
            //     {
            //         // std::cout << "-------" << std::endl;
            //         // Eigen::Vector2f pos = {node->center_x - node->size, node->center_y - node->size};
            //         gridMap->set_buffer(pos - bias);
            //     }
            //     else
            //     {
            //         // std::cout << "-------" << std::endl;
            //         // Eigen::Vector2f pos = {node->center_x + node->size, node->center_y + node->size};
            //         gridMap->set_buffer(pos + bias);
            //     }
            // }
            // else if (node->sub_occupied[cur_idx] == 2)
            // {
            //     // std::cout << "-------" << std::endl;
            //     // Eigen::Vector2f pos = {node->center_x - node->size, node->center_y - node->size};
            //     if (node->sub_occupied[ops_idx] == 2)
            //         gridMap->set_buffer(pos + bias);
            //     gridMap->set_buffer(pos - bias);
            // }
            // else
            // {
            //     std::cout << "----++---" << std::endl;
            // }
        }
        /*

   / 0 / 4 /
  / 2 / 6 /
                      ___ x
   / 1 / 5 /        /|
  / 3 / 7 /        y  z

  */

        void process_node(OctreeNode *node)
        {
            Eigen::Array3d pos = node->center;
            // Eigen::Array3d size = Eigen::Array3d::Constant(node->size * 0.5f);

            double half_size = node->size * 0.5;

            set_map(node, pos, {half_size, half_size, half_size}, 0, 7);
            set_map(node, pos, {-half_size, half_size, half_size}, 4, 3);
            set_map(node, pos, {half_size, -half_size, half_size}, 2, 5);
            set_map(node, pos, {-half_size, -half_size, half_size}, 6, 1);

            // pos = node->center;
            // pos[0] = node->center[0] - half_size;
            // set_map(node, pos, {0, half_size, half_size}, 0, 3);
            // set_map(node, pos, {0, -half_size, half_size}, 2, 1);

            // pos = node->center;
            // pos[0] = node->center[0] + half_size;
            // set_map(node, pos, {0, half_size, half_size}, 4, 7);
            // set_map(node, pos, {0, -half_size, half_size}, 6, 5);

            // pos = node->center;
            // pos[1] = node->center[1] - half_size;
            // set_map(node, pos, {half_size, 0, half_size}, 2, 7);
            // set_map(node, pos, {-half_size, 0, half_size}, 6, 3);

            // pos = node->center;
            // pos[1] = node->center[1] + half_size;
            // set_map(node, pos, {half_size, 0, half_size}, 0, 5);
            // set_map(node, pos, {-half_size, 0, half_size}, 4, 1);

            // pos = node->center;
            // pos[2] = node->center[2] - half_size;
            // set_map(node, pos, {half_size, half_size, 0}, 0, 6);
            // set_map(node, pos, {-half_size, half_size, 0}, 4, 2);

            // pos = node->center;
            // pos[2] = node->center[2] + half_size;
            // set_map(node, pos, {half_size, half_size, 0}, 1, 7);
            // set_map(node, pos, {-half_size, half_size, 0}, 5, 3);

            // pos[0] = node->center[0] + node->size * 0.5;
        }

        // void traverse(OctreeNode *node)
        // {
        //     if (node->isLeaf())
        //     {

        //         process_node(node);
        //     }
        //     else
        //     {
        //         // process_node(node);

        //         for (const auto &child : node->children)
        //         {
        //             if (child)
        //             {
        //                 traverse(child);
        //             }
        //         }
        //     }

        // }
        void traverse(OctreeNode *node)
        {
            if (node->isLeaf())
            {
                process_node(node);
            }
            else
            {
                // 并行调用最多 8 个子节点
                tbb::parallel_invoke(
                    [&]
                    { if (node->children[0]) traverse(node->children[0]); },
                    [&]
                    { if (node->children[1]) traverse(node->children[1]); },
                    [&]
                    { if (node->children[2]) traverse(node->children[2]); },
                    [&]
                    { if (node->children[3]) traverse(node->children[3]); },
                    [&]
                    { if (node->children[4]) traverse(node->children[4]); },
                    [&]
                    { if (node->children[5]) traverse(node->children[5]); },
                    [&]
                    { if (node->children[6]) traverse(node->children[6]); },
                    [&]
                    { if (node->children[7]) traverse(node->children[7]); });
            }
        }

        void mapping()
        {
            // 遍历整个树，处理每个节点
            traverse(root.get());

            // grid_map->updateOccupiedVoxelNodeCount();
            // grid_map->updateFreeVoxelNodeCount();
            // grid_map->updateUnexploredVoxelNodeCount();
        }

        void insertPcd(const Eigen::Matrix3Xd &pts)
        {

            int size = pts.cols();

            // size_t total_work = size;
            // tbb::task_scheduler_init init;
            // size_t num_threads = init.default_num_threads();
            // // std::cout << "Default TBB threads: " << num_threads << std::endl;

            // size_t factor = 4;
            // size_t min_grain = 512;

            // size_t grain_size = std::max(total_work / (num_threads * factor), min_grain);

            tbb::parallel_for(
                tbb::blocked_range<size_t>(0, size),
                // tbb::blocked_range<size_t>(0, size, 2072 * 16),
                // tbb::blocked_range<size_t>(0, size, 2072 * 40),
                [&](const tbb::blocked_range<size_t> &r)
                {
                    for (size_t i = r.begin(); i < r.end(); ++i)
                    {

                        Eigen::Array3d pt(pts(0, i), pts(1, i), pts(2, i));

                        insertIter(root.get(), pt, root->getCenter(), root->getSize());
                    }
                },
                // tbb::simple_partitioner()
                tbb::auto_partitioner() // 简单、固定划分
            );
        }

        void print_info()
        {
            std::cout << "Octree Info:" << std::endl;
            std::cout << "  Center: " << root->getCenter().transpose() << std::endl;
            std::cout << "  Size: " << root->getSize() << std::endl;
            std::cout << "  map size: " << env_info_ptr->map_size.transpose() << std::endl;
            std::cout << "  Resolution: " << resolution << std::endl;
        }
        const std::unique_ptr<OctreeNode> &getRoot() const
        {
            return root;
        }

        void collect_nodes(OctreeNode *node, std::vector<Eigen::Array3d> &center_vec, std::vector<double> &size)
        {
            if (node->isLeaf())
            {

                center_vec.push_back(node->getCenter());
                size.push_back(node->getSize());
            }
            else
            {
                for (const auto &child : node->children)
                {
                    if (child)
                    {
                        collect_nodes(child, center_vec, size);
                    }
                }
            }
        }

        void collect_voxel(std::vector<Eigen::Array3d> &center_vec, bool original)
        {
            // if (original)
            // {
            //     const std::atomic<int16_t> *grid_map_node = grid_map_original->getVoxelNode();
            //     size_t total_voxel_count = grid_map_original->getTotalVoxelCount();
            //     for (size_t i = 0; i < total_voxel_count; ++i)
            //     {
            //         if (grid_map_node[i].load(std::memory_order_relaxed) > 0)
            //         {
            //             Eigen::Array3i idx = grid_map_original->unflatten3D(i);
            //             Eigen::Array3d center = grid_map_original->gridToWorld(idx);
            //             center_vec.push_back(center);

            //             // std::cout << ", Center: " << center.transpose()
            //             //           << ", Count: " << grid_map_node[i].load(std::memory_order_relaxed) << std::endl;

            //             // Do something with center and size
            //             // std::cout << "Voxel Center: " << center.transpose() << ", Size: " << size << std::endl;
            //         }
            //     }
            // }
            // else
            // {
                const std::atomic<int16_t> *grid_map_node = grid_map->getVoxelNode();
                size_t total_voxel_count = grid_map->getTotalVoxelCount();
                for (size_t i = 0; i < total_voxel_count; ++i)
                {
                    if (grid_map_node[i].load(std::memory_order_relaxed) > 0)
                    {
                        Eigen::Array3i idx = grid_map->unflatten3D(i);
                        Eigen::Array3d center = grid_map->gridToWorld(idx);
                        center_vec.push_back(center);

                        // std::cout << ", Center: " << center.transpose()
                        //           << ", Count: " << grid_map_node[i].load(std::memory_order_relaxed) << std::endl;

                        // Do something with center and size
                        // std::cout << "Voxel Center: " << center.transpose() << ", Size: " << size << std::endl;
                    }
                }
            // }
        }

        void collect_info(OctreeDisplay3d &displayOctree)
        {
            std::vector<Eigen::Array3d> octree_node_center_vec, voxel_node_center_vec;
            std::vector<double> octree_node_size_vec;

            collect_nodes(root.get(), octree_node_center_vec, octree_node_size_vec);
            displayOctree.display_octree(octree_node_center_vec, octree_node_size_vec);

            // raycaster->collect_info(displayGridMap);
            // Do something with center_vec and size_vec
        }

        void collect_info(OctreeDisplay3d &displayOctree, voxelMapDisplay3d &displayGridMap_new)
        {
            std::vector<Eigen::Array3d> octree_node_center_vec, voxel_node_center_vec_new;
            std::vector<double> octree_node_size_vec;

            collect_nodes(root.get(), octree_node_center_vec, octree_node_size_vec);
            displayOctree.display_octree(octree_node_center_vec, octree_node_size_vec);

            collect_voxel(voxel_node_center_vec_new, false);
            displayGridMap_new.display_voxel(voxel_node_center_vec_new, grid_map->getResolution());

            // collect_voxel(voxel_node_center_vec_original, true);
            // displayGridMap_original.display_voxel(voxel_node_center_vec_original, grid_map->getResolution());

            Eigen::Array3d voxelmap_bd_min = grid_map->getMinCorner();
            Eigen::Array3d voxelmap_bd_max = grid_map->getMaxCorner();
            displayGridMap_new.display_boundline(voxelmap_bd_min, voxelmap_bd_max);

            // raycaster->collect_info(displayGridMap);
            // Do something with center_vec and size_vec
        }

        inline std::atomic<int16_t> *getVoxelmap() const
        {
            return grid_map->getVoxelNode();
        }
        inline Eigen::Array3d getMinCorner() const
        {
            return grid_map->getMinCorner();
        }
        inline Eigen::Array3d getMaxCorner() const
        {
            return grid_map->getMaxCorner();
        }

        inline Eigen::Array3i getMapDim() const
        {
            return grid_map->getMapDim();
        }

        inline double get_resolution() const
        {
            return resolution;
        }

        inline   bool isfree(const Eigen::Array3d &pt) {
            return grid_map->isfree(pt);
        }
        ~OcTree()
        {
            // root will be automatically deleted
        }
    };
} // namespace pomp_octree
