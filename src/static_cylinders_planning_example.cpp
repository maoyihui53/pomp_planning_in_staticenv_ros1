#include <ros/ros.h>
#include <param/paramReader.h>
#include <pomp_octomap/octomap.h>
#include <generate_pd/generatePd.h>

#include <gridmap/gridmap.h>
#include <gridmap/collisioncheck.h>
#include <chrono>

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

// 2) Single log_into template
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

bool isDistanceGreaterThan(
    const std::optional<Eigen::Vector3d> &maybe_start,
    const std::optional<Eigen::Vector3d> &maybe_goal,
    double threshold = 3.f)
{
    if (maybe_start && maybe_goal)
    {
        return ((*maybe_start - *maybe_goal).norm() > threshold);
    }
    return false;
}

bool hasAtLeast200Lines(const std::string &filename)
{
    std::ifstream in(filename);
    if (!in)
    {
        // could throw or handle error as you prefer
        return false;
    }

    std::string line;
    int count = 0;
    while (std::getline(in, line))
    {
        if (++count >= 200)
        {
            return true;
        }
    }
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "static_cylinders_planning_example");
    ros::NodeHandle nh;
    readParem(nh);
    int num_line = 0;

    for (int iter = 0; iter < 500; iter++)
    {
        // pd generator
        std::cout << "===============================" << iter << "======================================" << std::endl;
        std::unique_ptr<generateCylinderPd> pd_generator = std::make_unique<generateCylinderPd>(
            nh, num_cylinders, points_per_circle, radius_min_max, generate_cylinder_size, resolution, z_step);
        // std::unique_ptr<pomp_planning_display::OctreeDisplay3d> octree_display = std::make_unique<pomp_planning_display::OctreeDisplay3d>(nh);
        // std::unique_ptr<pomp_planning_display::voxelMapDisplay3d> voxelmap_display = std::make_unique<pomp_planning_display::voxelMapDisplay3d>(nh, false);
        // std::unique_ptr<pomp_planning_display::pathDisplay3d> pomp_path_display = std::make_unique<pomp_planning_display::pathDisplay3d>(nh, "pomp_path_marker");
        // std::unique_ptr<pomp_planning_display::pathDisplay3d> grid_path_display = std::make_unique<pomp_planning_display::pathDisplay3d>(nh, "grid_path_marker");

        env_info *env = pd_generator->getEnvInfo();
        std::cout << "pd size: " << env->points_array.cols() << std::endl;
        // if (iter > 4)
        //     log_into<int>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/pd_amount.txt", env->points_array.cols());

        std::unique_ptr<collisionchecker::CollisionChecker> collision_checker = std::make_unique<collisionchecker::CollisionChecker>(env);
        collision_checker->insertPcd(env->points_array);

        auto start_time = std::chrono::high_resolution_clock::now();
        std::unique_ptr<OcTree> pomp_octomap = std::make_unique<OcTree>(env);
        pomp_octomap->insertPcd(env->points_array);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration_0 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        // log_into()

        // mapping to the grid map

        start_time = std::chrono::high_resolution_clock::now();
        pomp_octomap->mapping();
        end_time = std::chrono::high_resolution_clock::now();
        auto duration_1 = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        // pomp_octomap->collect_info(*octree_display.get(), *voxelmap_display.get());

        start_time = std::chrono::high_resolution_clock::now();
        std::unique_ptr<gridMap::GridMap> grid_map = std::make_unique<gridMap::GridMap>(env);
        grid_map->insertPcd(env->points_array);
        end_time = std::chrono::high_resolution_clock::now();
        auto duration_2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        /*----------------------get map data----------------------*/
        std::vector<signed char> gridMap = grid_map->getGridMap();
        std::atomic<int16_t> *pomp_map_ptr = pomp_octomap->getVoxelmap();

        /*------------------set up start and goal--------------------*/

        Vec3f start = {-7.0f, -7.0f, -2.f};
        Vec3f goal = {7.0f, 7.0f, 2.f};

        // Vec3f start = {-4.f, -4.f, -4.f};
        // Vec3f goal = {4.f, 4.f, 4.f};

        if (!pomp_octomap->isfree({start[0], start[1], start[2]}) ||
            !pomp_octomap->isfree({goal[0], goal[1], goal[2]}) ||
            !grid_map->isfree({start[0], start[1], start[2]}) ||
            !grid_map->isfree({goal[0], goal[1], goal[2]}))
        {
            std::cout << "start or goal is not free, please set again" << std::endl;
            break;
        }

        // auto maybe_start;
        // auto maybe_goal;
        // do
        // {
        //     maybe_start = grid_map->findfree(gridMap);
        //     maybe_goal = grid_map->findfree(gridMap); // or findGoal()

        //     // handle the empty‑case
        // } while (!maybe_start || !maybe_goal || !pomp_octomap->isfree(maybe_start.value()) || !pomp_octomap->isfree(maybe_goal.value()))

        // std::optional<Eigen::Vector3d> maybe_start;
        // std::optional<Eigen::Vector3d> maybe_goal;
        // int time_count = 0;
        // do
        // {
        //     maybe_start = grid_map->findfree(gridMap);
        //     maybe_goal = grid_map->findfree(gridMap);
        //     time_count++;
        //     if (time_count > 50000)
        //     {
        //         std::cout << "Warning: Could not find free start and goal points after 50000 attempts." << std::endl;
        //         break;
        //     }
        // } while (
        //     !maybe_start.has_value() ||
        //     !maybe_goal.has_value() ||
        //     !pomp_octomap->isfree(*maybe_start) ||
        //     !pomp_octomap->isfree(*maybe_goal) );

        // // std::cout << "run time " << iter << std::endl;

        // Eigen::Vector3d start_vec = maybe_start.value();
        // Eigen::Vector3d goal_vec = maybe_goal.value();
        // start = start_vec;
        // goal = goal_vec;

        std::chrono::microseconds duration_grid_astar, duration_grid_jps, duration_pomp_astar, duration_pomp_jps;

        decimal_t grid_astar_dist, grid_jps_dist, pomp_astar_dist, pomp_jps_dist;

        bool valid_grid_astar_path = false;
        bool valid_grid_jps_path = false;
        bool valid_pomp_astar_path = false;
        bool valid_pomp_jps_path = false;

        vec_Vec3f path_grid_astar, path_grid_jps, path_pomp_astar, path_pomp_jps;
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
            }
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

                Vec3f min_(1000, 1000, 1000);
                Vec3f max_(-1000, -1000, -1000);
                for (int i = 0; i < path_grid_jps.size(); i++)
                {
                    // std::cout << "pomp jps path: " << path_pomp_jps[i].transpose() << std::endl;
                    min_ = min_.cwiseMin(path_grid_jps[i]);
                    max_ = max_.cwiseMax(path_grid_jps[i]);
                }
                if (min_[0] < -10 || min_[1] < -10 || min_[2] < -5)
                {
                    std::cout << "grid jps path min: " << min_.transpose() << std::endl;
                }
                if (max_[0] > 10 || max_[1] > 10 || max_[2] > 5)
                {
                    std::cout << "grid jps path max: " << max_.transpose() << std::endl;
                }

                // std::cout << "grid jps path min: " << min_.transpose() << std::endl;
                // std::cout << "grid jps path max: " << max_.transpose() << std::endl;
            }
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
            }

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

                Vec3f min_(1000, 1000, 1000);
                Vec3f max_(-1000, -1000, -1000);
                for (int i = 0; i < path_pomp_jps.size(); i++)
                {
                    // std::cout << "pomp jps path: " << path_pomp_jps[i].transpose() << std::endl;
                    min_ = min_.cwiseMin(path_pomp_jps[i]);
                    max_ = max_.cwiseMax(path_pomp_jps[i]);
                }
                if (min_[0] < -10 || min_[1] < -10 || min_[2] < -5)
                {
                    std::cout << "-------------pomp jps path min: " << min_.transpose() << std::endl;
                }
                if (max_[0] > 10 || max_[1] > 10 || max_[2] > 5)
                {
                    std::cout << "--------------pomp jps path max: " << max_.transpose() << std::endl;
                }
            }
        }

        {
            if (iter > 4)
            {
                if (valid_grid_astar_path && valid_grid_jps_path && valid_pomp_astar_path && valid_pomp_jps_path)
                {

                    log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/both_valid_gridmap_a_star_plan_time.txt", duration_grid_astar);
                    log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/both_valid_gridmap_jps_plan_time.txt", duration_grid_jps);
                    log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/both_valid_pomp_a_star_plan_time.txt", duration_pomp_astar);
                    log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/both_valid_pomp_jps_plan_time.txt", duration_pomp_jps);
                    std::cout << "grid a star plan time: " << duration_grid_astar.count() << " \u03BCs" << std::endl;
                    std::cout << "grid jps plan time: " << duration_grid_jps.count() << " \u03BCs" << std::endl;
                    std::cout << "pomp a star plan time: " << duration_pomp_astar.count() << " \u03BCs" << std::endl;
                    std::cout << "pomp jps plan time: " << duration_pomp_jps.count() << " \u03BCs" << std::endl
                              << std::endl;
                }

                log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/pomp_octree_construction_mapping.txt", std::chrono::duration_cast<std::chrono::milliseconds>(duration_0 + duration_1));
                log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/gridmap_construction.txt", duration_2);
                std::cout << "pomp octree construction and mapping time: " << std::chrono::duration_cast<std::chrono::milliseconds>(duration_0 + duration_1).count() << " ms" << std::endl;
                std::cout << "grid map construction time: " << duration_2.count() << " ms" << std::endl;
                log_into<bool>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/gridmap_a_star_find.txt", valid_grid_astar_path ? 1 : 0);
                log_into<bool>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/gridmap_jps_find.txt", valid_grid_jps_path ? 1 : 0);
                log_into<bool>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/pomp_a_star_find.txt", valid_pomp_astar_path ? 1 : 0);
                log_into<bool>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/pomp_jps_find.txt", valid_pomp_jps_path ? 1 : 0);
                std::cout << "grid a star path find: " << valid_grid_astar_path << std::endl;
                std::cout << "grid jps path find: " << valid_grid_jps_path << std::endl;
                std::cout << "pomp a star path find: " << valid_pomp_astar_path << std::endl;
                std::cout << "pomp jps path find: " << valid_pomp_jps_path << std::endl
                          << std::endl;

                log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/gridmap_a_star_plan_time.txt", duration_grid_astar);
                log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/gridmap_jps_plan_time.txt", duration_grid_jps);
                log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/pomp_a_star_plan_time.txt", duration_pomp_astar);
                log_into("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/pomp_jps_plan_time.txt", duration_pomp_jps);
                std::cout << "grid a star plan time: " << duration_grid_astar.count() << " \u03BCs" << std::endl;
                std::cout << "grid jps plan time: " << duration_grid_jps.count() << " \u03BCs" << std::endl;
                std::cout << "pomp a star plan time: " << duration_pomp_astar.count() << " \u03BCs" << std::endl;
                std::cout << "pomp jps plan time: " << duration_pomp_jps.count() << " \u03BCs" << std::endl
                          << std::endl;

                if (valid_grid_astar_path)
                {
                    grid_astar_dist = total_distance3f(path_grid_astar);
                    log_into<decimal_t>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/gridmap_a_star_dist.txt", grid_astar_dist);
                    std::cout << "grid a star path distance: " << grid_astar_dist << std::endl;
                }
                if (valid_grid_jps_path)
                {
                    grid_jps_dist = total_distance3f(path_grid_jps);
                    log_into<decimal_t>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/gridmap_jps_dist.txt", grid_jps_dist);
                    std::cout << "grid jps path distance: " << grid_jps_dist << std::endl;
                }
                if (valid_pomp_astar_path)
                {
                    pomp_astar_dist = total_distance3f(path_pomp_astar);
                    log_into<decimal_t>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/pomp_a_star_dist.txt", pomp_astar_dist);
                    std::cout << "pomp a star path distance: " << pomp_astar_dist << std::endl;
                }
                if (valid_pomp_jps_path)
                {
                    pomp_jps_dist = total_distance3f(path_pomp_jps);
                    log_into<decimal_t>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/pomp_jps_dist.txt", pomp_jps_dist);
                    std::cout << "pomp jps path distance: " << pomp_jps_dist << std::endl;
                }

                if (valid_grid_astar_path && valid_pomp_astar_path)
                {
                    grid_astar_dist = total_distance3f(path_grid_astar);
                    log_into<decimal_t>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/both_valid_grid_a_star_dist.txt", grid_astar_dist);
                    pomp_astar_dist = total_distance3f(path_pomp_astar);
                    log_into<decimal_t>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/both_valid_pomp_a_star_dist.txt", pomp_astar_dist);
                }

                if (valid_grid_jps_path && valid_pomp_jps_path)
                {
                    grid_jps_dist = total_distance3f(path_grid_jps);
                    log_into<decimal_t>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/both_valid_grid_jps_dist.txt", grid_jps_dist);
                    pomp_jps_dist = total_distance3f(path_pomp_jps);
                    log_into<decimal_t>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/both_valid_pomp_jps_dist.txt", pomp_jps_dist);
                }

                if (valid_pomp_astar_path && valid_pomp_jps_path)
                {
                    bool safe_pomp_astar = collision_checker->CollisionCheck(path_pomp_astar);
                    bool safe_pomp_jps = collision_checker->CollisionCheck(path_pomp_jps);

                    log_into<bool>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/pomp_a_star_safe.txt", safe_pomp_astar ? 1 : 0);
                    log_into<bool>("/home/yihuimao/experiment_data/pomp_planning/static_octomap_planning/doublecheck_Cylinder_dense_size/res_100cm/pomp_jps_safe.txt", safe_pomp_jps ? 1 : 0);
                    std::cout << "pomp a star path safe: " << safe_pomp_astar << std::endl;
                    std::cout << "pomp jps path safe: " << safe_pomp_jps << std::endl
                              << std::endl;
                }
            }
        }

        // grid_path_display->display_path(path_grid_jps);
        // pomp_path_display->display_path(path_pomp_jps);

        // octree_display->publish_octree();
        // voxelmap_display->publish_voxel();

        // grid_path_display->publish_path();
        // pomp_path_display->publish_path();

        // pd_generator->publishCloud();
        // ros::spin();
        // if (num_line >= 200)
        // {
        //     std::cout << "stop iterating, num_line: " << num_line << std::endl;
        //     break;
        // }
    }

    // grid_path_display->publish_path();
    // pomp_path_display->publish_path();

    // pd_generator->publishCloud();
    // ros::spin();
    // Initialize the octree
}
