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

using namespace JPS;
using namespace JPS_TBB;

using namespace pomp_planning_generatePd;
using namespace pomp_planning_octomap;

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "static_planning_example");
    ros::NodeHandle nh;
    readParem(nh);
    std::unique_ptr<generateCylinderPd> pd_generator = std::make_unique<generateCylinderPd>(
        nh, num_cylinders, points_per_circle, radius_min_max, generate_cylinder_size, z_step);
    std::unique_ptr<pomp_planning_display::OctreeDisplay3d> octree_display = std::make_unique<pomp_planning_display::OctreeDisplay3d>(nh);
    std::unique_ptr<pomp_planning_display::voxelMapDisplay3d> voxelmap_display_original = std::make_unique<pomp_planning_display::voxelMapDisplay3d>(nh, true);
    std::unique_ptr<pomp_planning_display::voxelMapDisplay3d> voxelmap_display_new = std::make_unique<pomp_planning_display::voxelMapDisplay3d>(nh, false);
    std::unique_ptr<pomp_planning_display::pathDisplay3d> pomp_path_display = std::make_unique<pomp_planning_display::pathDisplay3d>(nh, "pomp_path_marker");
    std::unique_ptr<pomp_planning_display::pathDisplay3d> grid_path_display = std::make_unique<pomp_planning_display::pathDisplay3d>(nh, "grid_path_marker");

    env_info *env = pd_generator->getEnvInfo(resolution);

    std::cout << "Map size: " << env->map_size.transpose() << std::endl;
    std::cout << "Resolution: " << env->resolution << std::endl;
    std::cout << "pd size: " << env->points_array.cols() << std::endl;
    
    // for (int i = 0; i < 10; i++)
    // {

    auto t_0 = std::chrono::high_resolution_clock::now();
    std::unique_ptr<OcTree> pomp_octomap = std::make_unique<OcTree>(env, true);
    auto t_1 = std::chrono::high_resolution_clock::now();
    auto m_s = std::chrono::duration_cast<std::chrono::milliseconds>(t_1 - t_0).count();
    std::cout << "OcTree initialization time (high_res): " << m_s << " ms\n";

    auto t0_init = std::chrono::high_resolution_clock::now();
    pomp_octomap->insertPcd(env->points_array);
    auto t1_init = std::chrono::high_resolution_clock::now();

    auto ms_init = std::chrono::duration_cast<std::chrono::milliseconds>(t1_init - t0_init).count();
    std::cout << "Initialization time (high_res): " << ms_init << " ms\n";
    auto t0 = std::chrono::high_resolution_clock::now();
    pomp_octomap->mapping();
    // … 你的代码 …
    auto t1 = std::chrono::high_resolution_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout << "Elapsed (high_res): " << ms << " ms\n";
    pomp_octomap->collect_info(*octree_display.get(), *voxelmap_display_original.get(), *voxelmap_display_new.get());

    std::atomic<int16_t> *pomp_map_ptr = pomp_octomap->getVoxelmap();

    Eigen::Array3d min_corner = pomp_octomap->getMinCorner();
    Eigen::Array3d max_corner = pomp_octomap->getMaxCorner();
    Eigen::Array3i map_dim = pomp_octomap->getMapDim();

    Vecf<3> origin(min_corner[0], min_corner[1], min_corner[2]);
    Veci<3> dim(map_dim[0], map_dim[1], map_dim[2]);
    decimal_t resolution = pomp_octomap->get_resolution();

    Vec3f start;
    Vec3f goal;

    std::unique_ptr<gridMap::GridMap> grid_map = std::make_unique<gridMap::GridMap>(min_corner, map_dim, env, true);

    std::unique_ptr<collisionchecker::CollisionChecker> collision_checker = std::make_unique<collisionchecker::CollisionChecker>(env);

    collision_checker->insertPcd(env->points_array);

    grid_map->insertPcd(env->points_array);

    // std::vector<unsigned char> grid_map_data = grid_map->getGridMap();

    std::shared_ptr<JPS::VoxelMapUtil> jps_map_util = std::make_shared<JPS::VoxelMapUtil>();

    jps_map_util->setMap(origin, dim, grid_map->getGridMap(), resolution);

    auto maybe_start = grid_map->findfree(grid_map->getGridMap());
    auto maybe_goal = grid_map->findfree(grid_map->getGridMap()); // or findGoal()

    // 2) handle the empty‑case
    if (!maybe_start || !maybe_goal)
    {
        throw std::runtime_error("Could not pick both start and goal!");
    }

    // 3) unwrap
    Eigen::Vector3d start_vec = maybe_start.value();
    Eigen::Vector3d goal_vec = maybe_goal.value();

    // 4a) simplest: direct assign (if 'start' and 'goal' are also Vector3d)
    start = start_vec;
    goal = goal_vec;

    {
        std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(true)); // Declare a planner
        planner_ptr->setMapUtil(jps_map_util);

        // Set collision checking function
        planner_ptr->updateMap();

        bool valid_jps = planner_ptr->plan(start, goal, 1, true);
        vec_Vec3f path_jps = planner_ptr->getRawPath();

        std::cout << " grid jps distant: " << total_distance3f(path_jps) << std::endl;

        std::cout << "grid jps path:" << std::endl;
        for (const auto &it : path_jps)
            std::cout << it.transpose() << std::endl;
        grid_path_display->display_path(path_jps);


    }

    // }

    // size_t total_voxels = 100 * 100 * 100;
    // std::unique_ptr<std::atomic<int16_t>[]> map_ptr(new std::atomic<int16_t>[total_voxels]);

    // // 初始化为 0（可并行）
    // for (size_t i = 0; i < total_voxels; ++i)
    // {
    //     map_ptr[i].store(0, std::memory_order_relaxed);
    // }

    {
        std::shared_ptr<JPS_TBB::VoxelMapUtil> jps_tbb_map_util = std::make_shared<JPS_TBB::VoxelMapUtil>();

        jps_tbb_map_util->setMap(origin, dim, pomp_map_ptr, resolution);

        std::unique_ptr<JPSPlanner_TBB3D> planner_ptr(new JPSPlanner_TBB3D(true)); // Declare a planner
        planner_ptr->setMapUtil(jps_tbb_map_util);

        // Set collision checking function
        planner_ptr->updateMap();

        bool valid_jps = planner_ptr->plan(start, goal, 1, true);
        vec_Vec3f path_jps = planner_ptr->getRawPath();
        std::cout << "pomp jps path:" << std::endl;
        for (const auto &it : path_jps)
            std::cout << it.transpose() << std::endl;


        collision_checker->CollisionCheck(path_jps);

        
        // DMPlanner_TBB3D dmp(false);

        // dmp.setPotentialRadius(Vec3f(0.25, 0.25, 0.25)); // Set 2D potential field radius
        // dmp.setSearchRadius(Vec3f(0.25, 0.25, 0.25));    // Set the valid search region around given path
        // dmp.setMap(jps_tbb_map_util, start);             // Set map util for collision checking, must be called before planning

        // // Run DMP planner

        // bool valid_dist = dmp.computePath(start, goal, path_jps); // Compute the path given the jps path

        // const auto path_dist = dmp.getPath();

        // std::cout << "  pomp jps distant: " << total_distance3f(path_jps) << std::endl;

        // std::cout << "DMP Path:" << std::endl;
        // for (const auto &it : path_dist)
        //     std::cout << it.transpose() << std::endl;
        pomp_path_display->display_path(path_jps);
    }

    // Example of a static planning task
    ROS_INFO("Static planning example started.");

    octree_display->publish_octree();
    voxelmap_display_original->publish_voxel();
    voxelmap_display_new->publish_voxel();
    pomp_path_display->publish_path();
    grid_path_display->publish_path();

    // Here you would typically set up your planning environment,
    // load a map, configure the planner, etc.

    // Spin to keep the node alive
    pd_generator->publishCloud();
    ros::spin();

    return 0;
}