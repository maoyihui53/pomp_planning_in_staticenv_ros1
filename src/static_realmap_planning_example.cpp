#include <ros/ros.h>
#include <param/paramReader.h>
#include <pomp_octomap/octomap.h>
#include <generate_pd/generatePd.h>

#include <gridmap/gridmap.h>
#include <gridmap/collisioncheck.h>
#include <chrono>
#include <read_pd/readPd.h>
#include <memory>

// #include <jps_tbb/jps_basis/data_utils.h>
#include <jps_basis/data_utils.h>
#include <jps_tbb/jps_planner/jps_planner/jps_planner.h>
#include <jps_tbb/jps_planner/distance_map_planner/distance_map_planner.h>

#include <jps/jps_planner/jps_planner/jps_planner.h>
#include <jps/jps_planner/distance_map_planner/distance_map_planner.h>
#include <fstream>
#include <type_traits>
using namespace JPS;
using namespace JPS_TBB;

using namespace pomp_planning_generatePd;
using namespace pomp_planning_octomap;
using namespace pomp_planning_readPd;
template <typename T>
struct is_duration : std::false_type
{
};

template <typename Rep, typename Period>
struct is_duration<std::chrono::duration<Rep, Period>> : std::true_type
{
};

template <typename T>
inline constexpr bool is_duration_v = is_duration<std::decay_t<T>>::value;
template <typename T>
void log_into(const std::string &txt_file, const T &value)
{
    std::ofstream file(txt_file, std::ios::app);
    if (!file)
    {
        std::cerr << "Unable to open file: " << txt_file << "\n";
        return;
    }

    if constexpr (is_duration_v<T>)
    {
        // exact match for a std::chrono::duration
        file << value.count() << '\n';
    }
    else
    {
        // anything else that supports operator<<
        file << value << '\n';
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "static_realmap_planning_example");

    ros::NodeHandle nh;
    readParem(nh);

    std::unique_ptr<pomp_planning_display::OctreeDisplay3d> octree_display = std::make_unique<pomp_planning_display::OctreeDisplay3d>(nh);
    std::unique_ptr<pomp_planning_display::voxelMapDisplay3d> voxelmap_display = std::make_unique<pomp_planning_display::voxelMapDisplay3d>(nh, false);
    std::unique_ptr<pomp_planning_display::pathDisplay3d> pomp_path_display = std::make_unique<pomp_planning_display::pathDisplay3d>(nh, "pomp_path_marker");
    std::unique_ptr<pomp_planning_display::pathDisplay3d> grid_path_display = std::make_unique<pomp_planning_display::pathDisplay3d>(nh, "grid_path_marker");
    std::unique_ptr<pomp_planning_display::pathDisplay3d> collision_path_display = std::make_unique<pomp_planning_display::pathDisplay3d>(nh, "collision_path_marker");

    std::unique_ptr<readMapPd> reader = std::make_unique<readMapPd>(nh, static_pd_path);
    env_info *env = reader->getEnvInfo(resolution);
    reader->print_info();
    std::unique_ptr<gridMap::GridMap> gridmap_small_res = std::make_unique<gridMap::GridMap>(env, 0.5f);
    gridmap_small_res->insertPcd(env->points_array);

    auto start_time = std::chrono::high_resolution_clock::now();
    std::unique_ptr<OcTree> pomp_octomap = std::make_unique<OcTree>(env);
    pomp_octomap->insertPcd(env->points_array);
    pomp_octomap->mapping();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_0 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "pomp octree construction time: " << duration_0.count() << " ms" << std::endl;

    start_time = std::chrono::high_resolution_clock::now();
    std::unique_ptr<gridMap::GridMap> grid_map = std::make_unique<gridMap::GridMap>(env);
    grid_map->insertPcd(env->points_array);
    end_time = std::chrono::high_resolution_clock::now();
    auto duration_2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "grid map construction time: " << duration_2.count() << " ms" << std::endl;

    /*----------------------get map data----------------------*/
    std::vector<signed char> gridMap = grid_map->getGridMap();
    std::atomic<int16_t> *pomp_map_ptr = pomp_octomap->getVoxelmap();

    std::chrono::microseconds duration_grid_astar, duration_grid_jps, duration_pomp_astar, duration_pomp_jps;

    decimal_t grid_astar_dist, grid_jps_dist, pomp_astar_dist, pomp_jps_dist, gt_dist;

    bool valid_grid_astar_path = false;
    bool valid_grid_jps_path = false;
    bool valid_pomp_astar_path = false;
    bool valid_pomp_jps_path = false;
    bool valid_gt_path = false;

    vec_Vec3f path_grid_astar, path_grid_jps, path_pomp_astar, path_pomp_jps, path_gt;

    for (int iter = 0; iter < 1000; iter++)
    {

        std::cout << "===============================" << iter << "======================================" << std::endl;

        // std::optional<Eigen::Vector3d> maybe_start;
        // std::optional<Eigen::Vector3d> maybe_goal;
        // int time_count = 0;
        // do
        // {
        //     maybe_start = grid_map->findfree(gridMap);
        //     maybe_goal = grid_map->findfree(gridMap);
        //     time_count++;
        //     if (time_count > 5000)
        //     {
        //         std::cout << "Warning: Could not find free start and goal points after 5000 attempts." << std::endl;
        //         break;
        //     }
        // } while (
        //     !maybe_start.has_value() ||
        //     !maybe_goal.has_value() ||
        //     !pomp_octomap->isfree(*maybe_start) ||
        //     !pomp_octomap->isfree(*maybe_goal));

        // Vec3f start = maybe_start.value();
        // Vec3f goal = maybe_goal.value();

        // if (time_count > 5000)
        // {
        //     std::cout << "start or goal is not free, please set again" << std::endl;
        //     break;
        // }

        Eigen::Vector3d start_v;
        Eigen::Vector3d goal_v;
        Vec3f start, goal;

        int maxtrial = 500;
        {
            for (int i_trial = 0; i_trial < maxtrial; i_trial++)
            {
                bool start_find = gridmap_small_res->find_free_voxel(env, start_v);
                bool goal_find = gridmap_small_res->find_free_voxel(env, goal_v);
                if (start_find && goal_find)
                {
                    start = start_v;
                    goal = goal_v;
                }
                else
                {
                    std::cout << "can not find the start point or goal point" << std::endl;
                }
                // Eigen::Array3d min_corner = gridmap_small_res->getMinCorner();
                // Eigen::Array3i map_dim = gridmap_small_res->getMapDim();

                // Vecf<3> origin(min_corner[0], min_corner[1], min_corner[2]);
                // Veci<3> dim(map_dim[0], map_dim[1], map_dim[2]);
                // decimal_t resolution = gridmap_small_res->getResolution();

                // std::shared_ptr<JPS::VoxelMapUtil> jps_map_util = std::make_shared<JPS::VoxelMapUtil>();
                // jps_map_util->setMap(origin, dim, gridmap_small_res->getGridMap(), resolution);
                // std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(false)); // Declare a planner
                // planner_ptr->setMapUtil(jps_map_util);

                // // Set collision checking function
                // planner_ptr->updateMap();

                // valid_gt_path = planner_ptr->plan(start, goal, 1, true);

                if (start_find && goal_find)
                {

                    std::cout << "start and goal find" << std::endl;
                    // if (valid_gt_path)
                    // {
                    //     path_gt = planner_ptr->getRawPath();
                    // }

                    // collision_path = planner_ptr->getRawPath();
                    // collision_path_dist = total_distance3f(collision_path);
                    // std::cout << "collision path distance: " << collision_path_dist << std::endl;

                    break;
                }
            }
        }

        /*** original grid a star */
        {

            // std::cout << "-----------------------------grid a star-------------------" << std::endl;
            /*---------------------planning setting up-------------------*/
            Eigen::Array3d min_corner = grid_map->getMinCorner();
            Eigen::Array3i map_dim = grid_map->getMapDim();

            Vecf<3> origin(min_corner[0], min_corner[1], min_corner[2]);
            Veci<3> dim(map_dim[0], map_dim[1], map_dim[2]);
            decimal_t resolution = grid_map->getResolution();

            std::shared_ptr<JPS::VoxelMapUtil> jps_map_util = std::make_shared<JPS::VoxelMapUtil>();
            jps_map_util->setMap(origin, dim, grid_map->getGridMap(), resolution);
            std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(false)); // Declare a planner
            planner_ptr->setMapUtil(jps_map_util);

            // Set collision checking function
            planner_ptr->updateMap();

            auto start_plan_time = std::chrono::high_resolution_clock::now();
            valid_grid_astar_path = planner_ptr->plan(start, goal, 1, false);
            auto end_plan_time = std::chrono::high_resolution_clock::now();
            duration_grid_astar = std::chrono::duration_cast<std::chrono::microseconds>(end_plan_time - start_plan_time);
            if (valid_grid_astar_path)
            {
                path_grid_astar = planner_ptr->getRawPath();
                // grid_astar_dist = total_distance3f(path_grid_astar);
                // std::cout << "grid a star plan time: " << duration_grid_astar.count() << " ms" << std::endl;
                // std::cout << "grid a star path distance: " << grid_astar_dist << std::endl;
            }
            // else
            // {
            //     std::cout << "grid a star plan failed!" << std::endl;
            // }
        }

        /*** original grid jps*/
        {

            /*---------------------planning setting up-------------------*/

            // std::cout << "-----------------------------grid jps---------------------" << std::endl;
            Eigen::Array3d min_corner = grid_map->getMinCorner();
            Eigen::Array3i map_dim = grid_map->getMapDim();

            Vecf<3> origin(min_corner[0], min_corner[1], min_corner[2]);
            Veci<3> dim(map_dim[0], map_dim[1], map_dim[2]);
            decimal_t resolution = grid_map->getResolution();

            std::shared_ptr<JPS::VoxelMapUtil> jps_map_util = std::make_shared<JPS::VoxelMapUtil>();
            jps_map_util->setMap(origin, dim, grid_map->getGridMap(), resolution);
            std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(false)); // Declare a planner
            planner_ptr->setMapUtil(jps_map_util);

            // Set collision checking function
            planner_ptr->updateMap();
            auto start_plan_time = std::chrono::high_resolution_clock::now();
            valid_grid_jps_path = planner_ptr->plan(start, goal, 1, true);
            auto end_plan_time = std::chrono::high_resolution_clock::now();
            duration_grid_jps = std::chrono::duration_cast<std::chrono::microseconds>(end_plan_time - start_plan_time);
            if (valid_grid_jps_path)
            {
                path_grid_jps = planner_ptr->getRawPath();
                // grid_jps_dist = total_distance3f(path_grid_jps);
                // std::cout << "grid jps plan time: " << duration_grid_jps.count() << " ms" << std::endl;
                // std::cout << "grid jps path distance: " << grid_jps_dist << std::endl;
            }
            // else
            // {
            //     std::cout << "grid jps plan failed!" << std::endl;
            // }
        }
        /**** pomp a star*/
        {
            // std::cout << "-----------------------------pomp a star-------------------" << std::endl;

            Eigen::Array3d min_corner = pomp_octomap->getMinCorner();
            Eigen::Array3i map_dim = pomp_octomap->getMapDim();
            Vecf<3> origin(min_corner[0], min_corner[1], min_corner[2]);
            Veci<3> dim(map_dim[0], map_dim[1], map_dim[2]);
            decimal_t resolution = pomp_octomap->get_resolution();

            std::shared_ptr<JPS_TBB::VoxelMapUtil> jps_tbb_map_util = std::make_shared<JPS_TBB::VoxelMapUtil>();
            jps_tbb_map_util->setMap(origin, dim, pomp_map_ptr, resolution);
            std::unique_ptr<JPSPlanner_TBB3D> planner_ptr(new JPSPlanner_TBB3D(false)); // Declare a planner
            planner_ptr->setMapUtil(jps_tbb_map_util);

            // Set collision checking function
            planner_ptr->updateMap();
            auto start_plan_time = std::chrono::high_resolution_clock::now();
            valid_pomp_astar_path = planner_ptr->plan(start, goal, 1, false);
            auto end_plan_time = std::chrono::high_resolution_clock::now();
            duration_pomp_astar = std::chrono::duration_cast<std::chrono::microseconds>(end_plan_time - start_plan_time);

            if (valid_pomp_astar_path)
            {
                path_pomp_astar = planner_ptr->getRawPath();
                // pomp_astar_dist = total_distance3f(path_pomp_astar);
                // std::cout << "pomp a star plan time: " << duration_pomp_astar.count() << " ms" << std::endl;
                // std::cout << "pomp a star path distance: " << pomp_astar_dist << std::endl;
            }
            // else
            // {
            //     std::cout << "pomp a star plan failed!" << std::endl;
            // }

            // pomp_path_display->display_path(path_jps);
        }

        /**** pomp jps*/
        {
            // std::cout << "-----------------------------pomp jps---------------------" << std::endl;

            Eigen::Array3d min_corner = pomp_octomap->getMinCorner();
            Eigen::Array3i map_dim = pomp_octomap->getMapDim();
            Vecf<3> origin(min_corner[0], min_corner[1], min_corner[2]);
            Veci<3> dim(map_dim[0], map_dim[1], map_dim[2]);
            decimal_t resolution = pomp_octomap->get_resolution();

            std::shared_ptr<JPS_TBB::VoxelMapUtil> jps_tbb_map_util = std::make_shared<JPS_TBB::VoxelMapUtil>();
            jps_tbb_map_util->setMap(origin, dim, pomp_map_ptr, resolution);
            std::unique_ptr<JPSPlanner_TBB3D> planner_ptr(new JPSPlanner_TBB3D(false)); // Declare a planner
            planner_ptr->setMapUtil(jps_tbb_map_util);

            // Set collision checking function
            planner_ptr->updateMap();

            auto start_plan_time = std::chrono::high_resolution_clock::now();
            valid_pomp_jps_path = planner_ptr->plan(start, goal, 1, true);
            auto end_plan_time = std::chrono::high_resolution_clock::now();
            duration_pomp_jps = std::chrono::duration_cast<std::chrono::microseconds>(end_plan_time - start_plan_time);
            if (valid_pomp_jps_path)
            {
                path_pomp_jps = planner_ptr->getRawPath();
                // pomp_jps_dist = total_distance3f(path_pomp_jps);
                // std::cout << "pomp jps plan time: " << duration_pomp_jps.count() << " ms" << std::endl;
                // std::cout << "pomp jps path distance: " << pomp_jps_dist << std::endl;
            }
            // else
            // {
            //     std::cout << "pomp jps plan failed!" << std::endl;
            // }
        }

        // if ((!grid_map->isfree(start_v)) || (!grid_map->isfree(goal_v)))
        // {
        //     valid_grid_astar_path = false;
        //     valid_grid_jps_path = false;
        // }
        // if ((!pomp_octomap->isfree(start_v)) || (!pomp_octomap->isfree(goal_v)))
        // {
        //     valid_pomp_astar_path = false;
        //     valid_pomp_jps_path = false;
        // }

        {
            // if (iter > 4)
            // {
                // log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/pomp_octree_construction_mapping.txt", std::chrono::duration_cast<std::chrono::milliseconds>(duration_0 + duration_1));
                // log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/gridmap_construction.txt", duration_2);
                // std::cout << "pomp octree construction and mapping time: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration_0 + duration_1).count() << " ms" << std::endl;
                // std::cout << "grid map construction time: " << duration_2.count() << " ms" << std::endl;
                if (valid_grid_astar_path && valid_grid_jps_path && valid_pomp_astar_path && valid_pomp_jps_path)
                {

                    log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/both_valid_gridmap_a_star_plan_time.txt", duration_grid_astar);
                    log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/both_valid_gridmap_jps_plan_time.txt", duration_grid_jps);
                    log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/both_valid_pomp_a_star_plan_time.txt", duration_pomp_astar);
                    log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/both_valid_pomp_jps_plan_time.txt", duration_pomp_jps);
                    std::cout << "grid a star plan time: " << duration_grid_astar.count() << " \u03BCs" << std::endl;
                    std::cout << "grid jps plan time: " << duration_grid_jps.count() << " \u03BCs" << std::endl;
                    std::cout << "pomp a star plan time: " << duration_pomp_astar.count() << " \u03BCs" << std::endl;
                    std::cout << "pomp jps plan time: " << duration_pomp_jps.count() << " \u03BCs" << std::endl
                              << std::endl;
                }

                log_into<bool>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/gridmap_a_star_find.txt", valid_grid_astar_path ? 1 : 0);
                log_into<bool>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/gridmap_jps_find.txt", valid_grid_jps_path ? 1 : 0);
                log_into<bool>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/pomp_a_star_find.txt", valid_pomp_astar_path ? 1 : 0);
                log_into<bool>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/pomp_jps_find.txt", valid_pomp_jps_path ? 1 : 0);

                std::cout << "grid a star path find: " << valid_grid_astar_path << std::endl;
                std::cout << "grid jps path find: " << valid_grid_jps_path << std::endl;
                std::cout << "pomp a star path find: " << valid_pomp_astar_path << std::endl;
                std::cout << "pomp jps path find: " << valid_pomp_jps_path << std::endl
                          << std::endl;

                log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/gridmap_a_star_plan_time.txt", duration_grid_astar);
                log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/gridmap_jps_plan_time.txt", duration_grid_jps);
                log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/pomp_a_star_plan_time.txt", duration_pomp_astar);
                log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/pomp_jps_plan_time.txt", duration_pomp_jps);
                // std::cout << "grid a star plan time: " << duration_grid_astar.count() << " ms" << std::endl;
                // std::cout << "grid jps plan time: " << duration_grid_jps.count() << " ms" << std::endl;
                // std::cout << "pomp a star plan time: " << duration_pomp_astar.count() << " ms" << std::endl;
                // std::cout << "pomp jps plan time: " << duration_pomp_jps.count() << " ms" << std::endl
                //           << std::endl;

                if (valid_grid_astar_path)
                {
                    grid_astar_dist = total_distance3f(path_grid_astar);
                    log_into<decimal_t>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/gridmap_a_star_dist.txt", grid_astar_dist);
                    std::cout << "grid a star path distance: " << grid_astar_dist << std::endl;
                }
                if (valid_grid_jps_path)
                {
                    grid_jps_dist = total_distance3f(path_grid_jps);
                    log_into<decimal_t>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/gridmap_jps_dist.txt", grid_jps_dist);
                    std::cout << "grid jps path distance: " << grid_jps_dist << std::endl;
                }
                if (valid_pomp_astar_path)
                {
                    pomp_astar_dist = total_distance3f(path_pomp_astar);
                    log_into<decimal_t>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/pomp_a_star_dist.txt", pomp_astar_dist);
                    std::cout << "pomp a star path distance: " << pomp_astar_dist << std::endl;
                }
                if (valid_pomp_jps_path)
                {
                    pomp_jps_dist = total_distance3f(path_pomp_jps);
                    log_into<decimal_t>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/pomp_jps_dist.txt", pomp_jps_dist);
                    std::cout << "pomp jps path distance: " << pomp_jps_dist << std::endl;
                }

                if (valid_grid_astar_path && valid_pomp_astar_path)
                {
                    grid_astar_dist = total_distance3f(path_grid_astar);
                    log_into<decimal_t>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/both_valid_grid_a_star_dist.txt", grid_astar_dist);
                    pomp_astar_dist = total_distance3f(path_pomp_astar);
                    log_into<decimal_t>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/both_valid_pomp_a_star_dist.txt", pomp_astar_dist);
                }

                if (valid_grid_jps_path && valid_pomp_jps_path)
                {
                    grid_jps_dist = total_distance3f(path_grid_jps);
                    log_into<decimal_t>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/both_valid_grid_jps_dist.txt", grid_jps_dist);
                    pomp_jps_dist = total_distance3f(path_pomp_jps);
                    log_into<decimal_t>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/both_valid_pomp_jps_dist.txt", pomp_jps_dist);
                }

                // if (valid_pomp_astar_path && valid_pomp_jps_path)
                // {
                //     bool safe_pomp_astar = collision_checker->CollisionCheck(path_pomp_astar);
                //     bool safe_pomp_jps = collision_checker->CollisionCheck(path_pomp_jps);

                //     log_into<bool>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/pomp_a_star_safe.txt", safe_pomp_astar ? 1 : 0);
                //     log_into<bool>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_new_cambridage_16/res_6m/pomp_jps_safe.txt", safe_pomp_jps ? 1 : 0);
                //     std::cout << "pomp a star path safe: " << safe_pomp_astar << std::endl;
                //     std::cout << "pomp jps path safe: " << safe_pomp_jps << std::endl
                //               << std::endl;
                // }
            // }
        }
        // grid_path_display->display_path(path_grid_jps);
        // pomp_path_display->display_path(path_pomp_jps);
        // // collision_path_display->display_path(collision_path);

        // grid_path_display->publish_path();
        // pomp_path_display->publish_path();

        // collision_path_display->publish_path();

        // pomp_octomap->collect_info(*octree_display.get(), *voxelmap_display.get());
        // octree_display->publish_octree();
        // voxelmap_display->publish_voxel();

        // reader->publishCloud();

        // ros::spin();
    }
    //

    // octree_display->publish_octree();
    // voxelmap_display->publish_voxel();

    // reader->publishCloud();

    return 0;
}