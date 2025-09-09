#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <utils/util.h>
namespace pomp_planning_readPd
{

    using namespace pomp_type;
    using namespace pomp_env;

    class readMapPd
    {
    private:
        std::string ply_file_path;
        ros::Publisher pub;
        sensor_msgs::PointCloud2 output;

        std::unique_ptr<env_info> env;
        // env_info env;

        // // std::vector<Eigen::Vector3d> points_array;
        // Eigen::Matrix3Xd points_array; // Using Eigen for better performance
        // Eigen::Vector3d min_corner;
        // Eigen::Vector3d max_corner;

    public:
        readMapPd(ros::NodeHandle nh, const std::string &file_path)
            : ply_file_path(file_path), env(new env_info())
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            // latched = true -> publish once, stay in topic buffer
            pub = nh.advertise<sensor_msgs::PointCloud2>("static_ply_cloud", 1, true);

            // Load .ply file
            if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(ply_file_path, *cloud_ptr) == -1)
            {
                PCL_ERROR("Couldn't read PLY file: %s\n", ply_file_path.c_str());
                return;
            }
            ROS_INFO("Loaded %lu points from %s", cloud_ptr->points.size(), ply_file_path.c_str());
           

            // Optional: Downsample if point count is too large
            // if (cloud_ptr->points.size() > 10'000'000)
            // {
            //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
            //     pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
            //     voxel.setInputCloud(cloud_ptr);
            //     voxel.setLeafSize(0.01f, 0.01f, 0.01f); // Adjust as needed
            //     voxel.filter(*filtered);

            //     ROS_INFO("Downsampled to %lu points", filtered->points.size());
            //     cloud_ptr = filtered; // Replace
            // }
            if (!cloud_ptr || cloud_ptr->empty())
            {
                ROS_WARN("No point cloud to publish");
                return;
            }

            // sensor_msgs::PointCloud2 output;
            pclToEigenXYZ(cloud_ptr);
            pclRGBToPointCloud2WithOffset(cloud_ptr, output);
        }

        void print_info()
        {

            ROS_INFO("Point cloud info:");
            ROS_INFO("  Number of points: %lu", env->points_array.cols());
            Eigen::Vector3d mapSize = env->map_size; // Size of the map in 3D space

            ROS_INFO("  Map size: [%f, %f, %f]", mapSize(0), mapSize(1), mapSize(2));
            ROS_INFO("  Resolution: %f", env->resolution);
            // Current center
        }
        // ~readPd()
        // {
        //     if (cloud_ptr)
        //     {
        //         cloud_ptr->clear();
        //     }
        // }

        // void pointCloud2ToXYZArray(const sensor_msgs::PointCloud2 &cloud_msg)

        // {
        //     // 每个点为 x, y, z => 一个点三维
        //     size_t num_points = cloud_msg.width * cloud_msg.height;
        //     env->points_array.resize(3, num_points); // 3 行, n 列（每列是一个点）

        //     // 使用迭代器遍历 x, y, z 字段
        //     sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x");
        //     sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_msg, "y");
        //     sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_msg, "z");

        //     for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z)
        //     {
        //         env->points_array(0, i) = *iter_x; // x
        //         env->points_array(1, i) = *iter_y; // y
        //         env->points_array(2, i) = *iter_z; // z
        //     }
        //     env->min_corner = env->points_array.rowwise().minCoeff();
        //     env->max_corner = env->points_array.rowwise().maxCoeff();
        // }

        void pclToEigenXYZ(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr)
        {
            size_t num_points = cloud_ptr->size();
            env->points_array.resize(3, num_points); // 3 rows (x, y, z), num_points columns

            for (size_t i = 0; i < num_points; ++i)
            {
                const auto &pt = cloud_ptr->points[i];
                env->points_array(0, i) = pt.x;
                env->points_array(1, i) = pt.y;
                env->points_array(2, i) = pt.z;
            }
            env->min_corner = env->points_array.rowwise().minCoeff();
            env->max_corner = env->points_array.rowwise().maxCoeff();
            env->center = (env->min_corner + env->max_corner) / 2.0;

            env->adjust_to_center();
        }

        void pclRGBToPointCloud2WithOffset(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr,
                                           sensor_msgs::PointCloud2 &output)
        {
            // Step 1: Convert PCL to sensor_msgs::PointCloud2
            pcl::toROSMsg(*cloud_ptr, output);

            // Step 2: Subtract center from each point using iterators
            sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(output, "z");

            size_t num_points = cloud_ptr->size();
            for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z)
            {
                *iter_x -= static_cast<float>(env->center(0));
                *iter_y -= static_cast<float>(env->center(1));
                *iter_z -= static_cast<float>(env->center(2));
            }
        }

        env_info *getEnvInfo(double resolution = 0.1)
        {
            env->computeMinDepth(resolution); // Example resolution, adjust as needed
            return env.get();
        }

        void publishCloud()
        {

            output.header.frame_id = "map";
            output.header.stamp = ros::Time::now();
            pub.publish(output);

            ROS_INFO("Published static PLY cloud to topic: %s", pub.getTopic().c_str());
        }
    };

}
