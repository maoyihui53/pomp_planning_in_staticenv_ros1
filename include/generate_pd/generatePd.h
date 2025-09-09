#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/Dense>
#include <random>
#include <utils/util.h>

namespace pomp_planning_generatePd
{
    using namespace pomp_type;
    using namespace pomp_env;
    class generateCylinderPd
    {
    private:
        ros::Publisher pub;
        sensor_msgs::PointCloud2 output;

        std::unique_ptr<env_info> env;

    public:
        generateCylinderPd(ros::NodeHandle nh, int num_cyliner, int points_per_circle,
                           const std::vector<double> &radius_min_max,
                           const std::vector<double> &size, double res,
                           double z_step) : env(new env_info())

        {
            pub = nh.advertise<sensor_msgs::PointCloud2>("generated_cylinder_pd_cloud", 1, true);
            Eigen::Vector3d size_vec(size[0], size[1], size[2]);
            // env = std::make_unique<env_info>();
            env->map_size = size_vec;
            // std::cout << "Map size: " << env->map_size.transpose() << std::endl;

            //     env->points_array.resize(3, num_points);
            env->center = Eigen::Vector3d::Zero();
            env->min_corner = env->center - 0.5 * size_vec;
            env->max_corner = env->center + 0.5 * size_vec;

            int num_points = 0;
            const double eps = 1e-6; // 防止除零错误
            double z_min = env->min_corner[2] + eps;
            double z_max = env->max_corner[2] - eps;

            env->computeMinDepth(res); // Example resolution, adjust as needed

            // std::cout << "num_cyliner: " << num_cyliner << std::endl;
            // std::cout << "z min: " << z_min << ", z max: " << z_max << std::endl;
            // std::cout << "z step: " << z_step << std::endl;
            // std::cout << "Points per circle: " << points_per_circle << std::endl;

            // for (int i = 0; i < num_cyliner; ++i)
            // {
            //     for (double z = z_min; z <= z_max; z += z_step)
            //     {
            //         for (int j = 0; j < points_per_circle; ++j)
            //         {

            //             num_points++; // 每执行一次循环体就计数
            //         }
            //     }
            // }

            // std::cout << "Total number of points to generate: " << num_points << std::endl;
            // env->points_array.resize(3, num_points);
            random_generate_oriented_cylinders(
                num_cyliner, points_per_circle, radius_min_max, z_step);
            // random_generate_pcd(num_cyliner, points_per_circle, radius_min_max, z_step);
            // random_generate_pcd(num_points);
        }

        // generateCylinderPd(ros::NodeHandle nh, const std::vector<double> &size, int num_points) : env(new env_info())
        // {
        //     pub = nh.advertise<sensor_msgs::PointCloud2>("generated_cylinder_pd_cloud", 1, true);

        //     Eigen::Vector3d size_vec(size[0], size[1], size[2]);
        //     env->map_size = size_vec;
        //     env->points_array.resize(3, num_points);
        //     env->center = Eigen::Vector3d::Zero();
        //     env->min_corner = env->center - 0.5 * size_vec;
        //     env->max_corner = env->center + 0.5 * size_vec;
        //     random_generate_pcd(num_points);
        // }

        // void random_generate_pcd(int n, int points_per_circle, const std::vector<double> &radius_min_max, double z_step)
        // {
        //     // std::vector<Eigen::Vector3d> cloud;
        //     std::random_device rd;
        //     std::mt19937 gen(rd());

        //     std::cout << "Generating point cloud with parameters:" << std::endl;

        //     std::cout << env->min_corner.transpose() << std::endl;
        //     std::cout << env->max_corner.transpose() << std::endl;
        //     std::cout << "Radius min max: " << radius_min_max[0] << ", " << radius_min_max[1] << std::endl;

        //     const double eps = 1e-6; // 防止除零错误
        //     // 对称范围
        //     std::uniform_real_distribution<> x_dist(env->min_corner[0] + radius_min_max[1] + eps, env->max_corner[0] - radius_min_max[1] - eps);
        //     std::uniform_real_distribution<> y_dist(env->min_corner[1] + radius_min_max[1] + eps, env->max_corner[1] - radius_min_max[1] - eps);
        //     std::uniform_real_distribution<> radius_dist(radius_min_max[0], radius_min_max[1]);

        //     double z_min = env->min_corner[2] + eps;
        //     double z_max = env->max_corner[2] - eps;

        //     int count = 0;
        //     for (int i = 0; i < n; ++i)
        //     {
        //         double cx = x_dist(gen);
        //         double cy = y_dist(gen);
        //         double radius = radius_dist(gen);

        //         for (double z = z_min; z <= z_max; z += z_step)
        //         {
        //             for (int j = 0; j < points_per_circle; ++j)
        //             {
        //                 double angle = 2.0 * M_PI * j / points_per_circle;

        //                 double x = cx + radius * cos(angle);
        //                 double y = cy + radius * sin(angle);
        //                 env->points_array(0, count) = x;
        //                 env->points_array(1, count) = y;
        //                 env->points_array(2, count) = z;

        //                 count++;
        //             }
        //         }
        //     }

        //     env->adjust_to_center();

        //     std::cout << "count: " << count << std::endl;
        //     std::cout << "env->points_array " << env->points_array.cols() << std::endl;
        //     eigenToPointCloud2(env->points_array);
        // }

        // void random_generate_oriented_cylinders(
        //     int n,
        //     int points_per_circle,
        //     const std::vector<double> &radius_min_max,
        //     double z_step)
        // {
        //     std::random_device rd;
        //     std::mt19937 gen(rd());
        //     std::uniform_real_distribution<> uni01(0.0, 1.0);
        //     const double eps = 1e-6;

        //     // 地图 Z 范围
        //     auto &minC = env->min_corner;
        //     auto &maxC = env->max_corner;
        //     double Z_range = maxC[2] - minC[2];

        //     // 1) 准备一个高度分布：最矮一个层（z_step），最高不超过地图高度
        //     double max_cyl_height = (Z_range - eps) * 0.2;
        //     std::uniform_real_distribution<> height_dist(z_step, max_cyl_height);

        //     // 定义一个结构体存放每根圆柱的参数
        //     struct Cylinder
        //     {
        //         double r;               // 半径
        //         double length;          // 轴向总高度
        //         Eigen::Vector3d a;      // 轴方向单位向量
        //         Eigen::Vector3d c;      // 圆柱中心
        //         Eigen::Vector3d e1, e2; // 局部正交基
        //         int layers;             // 层数
        //     };

        //     std::vector<Cylinder> cyls;
        //     cyls.reserve(n);

        //     // 2) 第一遍：为每根圆柱采样 r, a, length, 计算 extents、中心 c、基 e1/e2、层数 layers
        //     int total_points = 0;
        //     for (int i = 0; i < n; ++i)
        //     {
        //         Cylinder cy;
        //         // 半径
        //         std::uniform_real_distribution<> radius_dist(
        //             radius_min_max[0], radius_min_max[1]);
        //         cy.r = radius_dist(gen);

        //         // 轴向方向 a ：球面均匀采样
        //         double u = uni01(gen);
        //         double cos_t = 2 * u - 1;
        //         double sin_t = std::sqrt(1 - cos_t * cos_t);
        //         double phi = 2.0 * M_PI * uni01(gen);
        //         cy.a = Eigen::Vector3d(
        //             sin_t * std::cos(phi),
        //             sin_t * std::sin(phi),
        //             cos_t);

        //         // 随机高度
        //         cy.length = height_dist(gen);
        //         double hi = cy.length * 0.5;

        //         // 计算 extents，确保整个圆柱都在地图里
        //         Eigen::Vector3d extents;
        //         for (int k = 0; k < 3; ++k)
        //         {
        //             extents[k] = hi * std::abs(cy.a[k]) + cy.r * std::sqrt(1 - cy.a[k] * cy.a[k]);
        //         }

        //         // 在安全范围内采样中心 c
        //         for (int k = 0; k < 3; ++k)
        //         {
        //             std::uniform_real_distribution<> dist(
        //                 minC[k] + extents[k] + eps,
        //                 maxC[k] - extents[k] - eps);
        //             cy.c[k] = dist(gen);
        //         }

        //         // 构造局部正交基 (e1, e2, a)
        //         Eigen::Vector3d arb = (std::abs(cy.a.x()) < 0.9
        //                                    ? Eigen::Vector3d(1, 0, 0)
        //                                    : Eigen::Vector3d(0, 1, 0));
        //         cy.e1 = cy.a.cross(arb).normalized();
        //         cy.e2 = cy.a.cross(cy.e1).normalized();

        //         // 计算层数
        //         cy.layers = static_cast<int>(std::floor(cy.length / z_step)) + 1;

        //         // 累加这根圆柱的点数
        //         total_points += cy.layers * points_per_circle;
        //         cyls.push_back(cy);
        //     }

        //     // 3) 预分配所有点
        //     env->points_array.resize(3, total_points);
        //     std::cout << "总点数 (total_points): " << total_points << std::endl;

        //     // 4) 第二遍：根据采样参数生成点
        //     int count = 0;
        //     for (auto &cy : cyls)
        //     {
        //         double hi = cy.length * 0.5;
        //         for (int layer = 0; layer < cy.layers; ++layer)
        //         {
        //             double t = -hi + layer * z_step;
        //             for (int j = 0; j < points_per_circle; ++j)
        //             {
        //                 double ang = 2.0 * M_PI * j / points_per_circle;
        //                 Eigen::Vector3d p =
        //                     cy.c + cy.a * t + cy.e1 * (cy.r * std::cos(ang)) + cy.e2 * (cy.r * std::sin(ang));
        //                 env->points_array.col(count++) = p;
        //             }
        //         }
        //     }

        //     // 5) （可选）居中并发布
        //     env->adjust_to_center();
        //     std::cout << "实际生成点数: " << count << std::endl;
        //     eigenToPointCloud2(env->points_array);
        // }
        inline bool isInCube(
            const Eigen::Vector3d &p,
            const Eigen::Vector3d &center,
            double halfExtent = 0.5)
        {
            // Compute absolute offset along each axis
            Eigen::Vector3d d = (p - center).cwiseAbs();
            // Check if all coordinates lie within [-halfExtent, +halfExtent]
            return (d.array() <= halfExtent).all();
        }

        void random_generate_oriented_cylinders(
            int n,
            int points_per_circle,
            const std::vector<double> &radius_min_max,
            double z_step)
        {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> uni01(0.0, 1.0);
            const double eps = 1e-6;

            // 地图范围
            auto &minC = env->min_corner;
            auto &maxC = env->max_corner;
            double Z_range = maxC[2] - minC[2];

            // 高度分布示例（不超过总高的 30%）
            double max_cyl_height = (Z_range - eps) * 0.3;
            std::uniform_real_distribution<> height_dist(z_step, max_cyl_height);

            struct Cylinder
            {
                double r;
                double length;
                Eigen::Vector3d a;
                Eigen::Vector3d c;
                Eigen::Vector3d e1, e2;
                int layers;
            };

            // 1) 采样所有圆柱参数（除了 center）
            std::vector<Cylinder> cyls;
            cyls.reserve(n);
            for (int i = 0; i < n; ++i)
            {
                Cylinder cy;
                // 半径
                std::uniform_real_distribution<> radius_dist(
                    radius_min_max[0], radius_min_max[1]);
                cy.r = radius_dist(gen);

                // 方向 a（球面均匀）
                double u = uni01(gen);
                double cos_t = 2 * u - 1;
                double sin_t = std::sqrt(1 - cos_t * cos_t);
                double phi = 2.0 * M_PI * uni01(gen);
                cy.a = {sin_t * std::cos(phi),
                        sin_t * std::sin(phi),
                        cos_t};

                // 高度
                cy.length = height_dist(gen);
                double hi = cy.length * 0.5;

                // 局部基
                Eigen::Vector3d arb = (std::abs(cy.a.x()) < 0.9
                                           ? Eigen::Vector3d(1, 0, 0)
                                           : Eigen::Vector3d(0, 1, 0));
                cy.e1 = cy.a.cross(arb).normalized();
                cy.e2 = cy.a.cross(cy.e1).normalized();

                // 层数
                cy.layers = static_cast<int>(std::floor(cy.length / z_step)) + 1;

                cyls.push_back(cy);
            }

            // 2) 为每根圆柱均匀采样 center（全域，无 extents 约束）
            for (auto &cy : cyls)
            {
                std::uniform_real_distribution<> dx(minC[0], maxC[0]);
                std::uniform_real_distribution<> dy(minC[1], maxC[1]);
                std::uniform_real_distribution<> dz(minC[2], maxC[2]);
                cy.c = {dx(gen), dy(gen), dz(gen)};
            }

            // 3) 生成所有点，丢弃超出 [minC,maxC] 的点
            std::vector<Eigen::Vector3d> pts;
            pts.reserve(n * cyls[0].layers * points_per_circle); // 粗略预留

            const size_t MAX_PTS = 20000000;
            bool done = false;

            std::cout << "env->resolution: " << env->resolution << std::endl;

            for (auto &cy : cyls)
            {
                double hi = cy.length * 0.5;
                for (int layer = 0; layer < cy.layers && !done; ++layer)
                {
                    double t = -hi + layer * z_step;
                    for (int j = 0; j < points_per_circle; ++j)
                    {
                        double ang = 2.0 * M_PI * j / points_per_circle;
                        Eigen::Vector3d p =
                            cy.c + cy.a * t + cy.e1 * (cy.r * std::cos(ang)) + cy.e2 * (cy.r * std::sin(ang));

                        // 越界检测
                        if ((p.array() < minC.array()).any() ||
                            (p.array() > maxC.array()).any() || isInCube(p, {-7.f, -7.f, -2.f}, env->resolution) || isInCube(p,  {7.f, 7.f, 2.f}, env->resolution))
                            //  (p.array() > maxC.array()).any() || isInCube(p, {-22.5f, -22.5f, -10.f}, env->resolution) || isInCube(p,  {22.5f, 22.5f, 10.f}, env->resolution))
                        {
                            continue;
                        }

                        pts.push_back(p);
                        if (pts.size() >= MAX_PTS)
                        {
                            done = true;
                            break; // 退出 j 循环
                        }
                    }
                }
                if (done)
                    break; // 退出 cy 循环
            }

            // 4) 填入 env->points_array 并发布
            int M = (int)pts.size();
            env->points_array.resize(3, M);
            for (int i = 0; i < M; ++i)
            {
                env->points_array.col(i) = pts[i];
            }

            // std::cout << "实际保留点数: " << M << std::endl;
            env->adjust_to_center();
            eigenToPointCloud2(env->points_array);
        }

        void random_generate_pcd(int num_points)
        {
            std::random_device rd;
            std::mt19937 gen(rd());

            std::uniform_real_distribution<double> dist_x(-0.55, 0.55);
            std::uniform_real_distribution<double> dist_y(-0.55, 0.55);
            std::uniform_real_distribution<double> dist_z(-0.55, 0.55);

            for (int i = 0; i < num_points; ++i)
            {
                env->points_array(0, i) = dist_x(gen);
                env->points_array(1, i) = dist_y(gen);
                env->points_array(2, i) = dist_z(gen);
            }
            env->adjust_to_center();

            // Convert to PointCloud2 message
            eigenToPointCloud2(env->points_array);
        };

        void eigenToPointCloud2(const Eigen::Matrix3Xd &points)
        {

            output.height = 1;
            output.width = points.cols();
            output.is_dense = false;
            output.is_bigendian = false;

            sensor_msgs::PointCloud2Modifier modifier(output);
            modifier.setPointCloud2FieldsByString(1, "xyz");
            modifier.resize(points.cols());

            sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(output, "z");

            for (int i = 0; i < points.cols(); ++i, ++iter_x, ++iter_y, ++iter_z)
            {
                *iter_x = static_cast<float>(points(0, i));
                *iter_y = static_cast<float>(points(1, i));
                *iter_z = static_cast<float>(points(2, i));
            }
        }
        env_info *getEnvInfo()
        {

            return env.get();
        }

        void publishCloud()
        {
            output.header.frame_id = "map";
            output.header.stamp = ros::Time::now();
            pub.publish(output);

            ROS_INFO("Published generated point cloud to topic: %s", pub.getTopic().c_str());
            ROS_INFO("Point cloud size: %zu points", env->points_array.cols());
        }
    };

}