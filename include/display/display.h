#pragma once
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <utils/util.h>
#include <jps_basis/data_type.h>

#include <string>
#include <vector>
#include <stdexcept>
#include <type_traits>
namespace pomp_planning_display
{
    enum MarkerType
    {
        CubeMarker,
        SphereMarker,
        LineStripMarker,
        LineListMarker,
        PointsMarker,
        TextMarker
    };
    template <typename T, typename = void>
    struct is_eigen_vector : std::false_type
    {
    };

    template <typename T>
    struct is_eigen_vector<T, std::void_t<decltype(T::SizeAtCompileTime)>> : std::true_type
    {
    };
    template <typename Vec3, typename VecScale, typename VecColor>

    // Function to set marker properties and add it to the marker array
    // This function is templated to accept different vector types for position, scale, and color
    // It uses static_assert to ensure the correct size of the vectors at compile time
    // If the vector size is not correct, it will throw an assertion error

    void set_marker_array(const std::string &ns,
                          const Vec3 &position,
                          const VecScale &scale,
                          const VecColor &color,
                          MarkerType type,
                          visualization_msgs::MarkerArray &marker_array)
    {
        if constexpr (is_eigen_vector<Vec3>::value)
            static_assert(Vec3::SizeAtCompileTime == 3, "Position must have 3 elements");
        else
            assert(position.size() == 3 && "Position must have 3 elements");

        if constexpr (is_eigen_vector<VecScale>::value)
            static_assert(VecScale::SizeAtCompileTime == 3, "Scale must have 3 elements");
        else
            assert(scale.size() == 3 && "Scale must have 3 elements");

        if constexpr (is_eigen_vector<VecColor>::value)
            static_assert(VecColor::SizeAtCompileTime == 4, "Color must have 4 elements");
        else
            assert(color.size() == 4 && "Color must have 4 elements");

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = marker_array.markers.size();

        switch (type)
        {
        case CubeMarker:
            marker.type = visualization_msgs::Marker::CUBE;
            break;
        case SphereMarker:
            marker.type = visualization_msgs::Marker::SPHERE;
            break;
        case LineStripMarker:
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            break;
        case LineListMarker:
            marker.type = visualization_msgs::Marker::LINE_LIST;
            break;
        case PointsMarker:
            marker.type = visualization_msgs::Marker::POINTS;
            break;
        case TextMarker:
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            break;
        default:
            throw std::invalid_argument("Unsupported marker type");
        }

        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = position[0];
        marker.pose.position.y = position[1];
        marker.pose.position.z = position[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = scale[0];
        marker.scale.y = scale[1];
        marker.scale.z = scale[2];

        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = color[3]; // Alpha channel

        // marker.lifetime = ros::Duration(0);  // 🔥 添加这一行！

        marker_array.markers.push_back(marker);
    }

    // std_msgs::ColorRGBA heightMapColor(double h)
    // {
    //     std_msgs::ColorRGBA color;
    //     h = std::min(1.0, std::max(0.0, h)); // clamp to [0,1]

    //     double s = 1.0;
    //     double v = 1.0;

    //     double i = floor(h * 6.0);
    //     double f = h * 6.0 - i;
    //     double p = v * (1.0 - s);
    //     double q = v * (1.0 - f * s);
    //     double t = v * (1.0 - (1.0 - f) * s);

    //     switch (static_cast<int>(i) % 6)
    //     {
    //     case 0:
    //         color.r = v;
    //         color.g = t;
    //         color.b = p;
    //         break;
    //     case 1:
    //         color.r = q;
    //         color.g = v;
    //         color.b = p;
    //         break;
    //     case 2:
    //         color.r = p;
    //         color.g = v;
    //         color.b = t;
    //         break;
    //     case 3:
    //         color.r = p;
    //         color.g = q;
    //         color.b = v;
    //         break;
    //     case 4:
    //         color.r = t;
    //         color.g = p;
    //         color.b = v;
    //         break;
    //     case 5:
    //         color.r = v;
    //         color.g = p;
    //         color.b = q;
    //         break;
    //     }

    //     color.a = 1.0; // full opacity
    //     return color;
    // }

    std_msgs::ColorRGBA yellowToBlueColor(double h)
    {
        std_msgs::ColorRGBA color;
        h = std::clamp(h, 0.0, 1.0); // 保证 h ∈ [0,1]

        // 插值：黄色(1,1,0) → 蓝色(0,0,1)
        color.r = (0.8 - h); // 从 1 → 0
        color.g = (1.0 - h); // 从 1 → 0
        color.b = h;         // 从 0 → 1
        color.a = 1.0;       // 不透明

        return color;
    }

    class OctreeDisplay3d
    {
    private:
        ros::Publisher octree_pub;
        ros::Publisher sensor_marker_pub;
        visualization_msgs::MarkerArray octree_marker_array;
        visualization_msgs::MarkerArray sensor_marker_array;

    public:
        OctreeDisplay3d(ros::NodeHandle nh) : octree_pub(nh.advertise<visualization_msgs::MarkerArray>("octree_marker_array", 1, true)),
                                              sensor_marker_pub(nh.advertise<visualization_msgs::MarkerArray>("sensor_marker_array", 1, true))

        {
        }

        void display_octree(const std::vector<Eigen::Array3d> center_vec, const std::vector<double> &size)
        {
            octree_marker_array.markers.clear();
            if (center_vec.size() != size.size())
            {
                throw std::invalid_argument("center_vec and size must have the same length");
            }

            for (size_t i = 0; i < center_vec.size(); ++i)
            {
                std::vector<double> position = {center_vec[i][0], center_vec[i][1], center_vec[i][2]};
                double node_size = (size[i] > 0 ? size[i] : 0.05);

                std::vector<double> scale = {node_size - 0.003, node_size - 0.003, node_size - 0.003};
                std::vector<double> color;
                std::string ns;
                double z_min = -30.0;                     // 最小高度
                double z_max = 30.0;                      // 最大高度
                double z = center_vec[i][2];              // 当前 voxel 的高度
                double h = (z - z_min) / (z_max - z_min); // 归一化到 [0,1]
                auto c = yellowToBlueColor(h);
                // color = {c.r, c.g, c.b, 0.2};
                color = {0.0, 0.3, 0.0, 0.5};
                ns = "occupied_node";

                set_marker_array(ns, position, scale, color, CubeMarker, octree_marker_array);
            }
        }

        void
        display_sensor_origin(const Eigen::Array3d &sensor_origin)
        {
            std::vector<double> position = {sensor_origin[0], sensor_origin[1], sensor_origin[2]};
            std::vector<double> scale = {0.5, 0.5, 0.5f};
            std::vector<double> color = {0.0, 1.0, 0.0, 1.0};
            set_marker_array("sensor_origin", position, scale, color, SphereMarker, sensor_marker_array);
        }

        void publish_octree()
        {
            octree_pub.publish(octree_marker_array);
            sensor_marker_pub.publish(sensor_marker_array);

            // bd_marker_pub.publish(bd_marker_array);
            // leaf_center_pub.publish(leafcenter_marker_array);
        }
    };

    class voxelMapDisplay3d
    {
    private:
        ros::Publisher voxel_pub;
        visualization_msgs::MarkerArray voxel_marker_array;
        ros::Publisher bd_marker_pub;
        visualization_msgs::MarkerArray voxel_bd_marker_array; // 用于边界线的 MarkerArray
        bool original_;

    public:
        voxelMapDisplay3d(ros::NodeHandle nh, bool original) : bd_marker_pub(nh.advertise<visualization_msgs::MarkerArray>("bd_marker_array", 1, true))
        {
            if (original)
            {
                voxel_pub = nh.advertise<visualization_msgs::MarkerArray>("voxel_marker_array_original", 1, true);
            }
            else
            {
                voxel_pub = nh.advertise<visualization_msgs::MarkerArray>("voxel_marker_array_new", 1, true);
            }
            original_ = original;
        }

        void display_voxel(const std::vector<Eigen::Array3d> &center_vec, const double &size)
        {
            voxel_marker_array.markers.clear();
            // if (center_vec.size() != size.size())
            // {
            //     throw std::invalid_argument("center_vec and size must have the same length");
            // }

            for (size_t i = 0; i < center_vec.size(); ++i)
            {
                std::vector<double> position = {center_vec[i][0], center_vec[i][1], center_vec[i][2]};
                double node_size = (size > 0 ? size : 0.05);

                std::vector<double> scale = {node_size - 0.002, node_size - 0.002, node_size - 0.002};
                std::vector<double> color;
                if (!original_)
                {
                    color = {0.8, 0.8, 0.0, 0.6};
                    std::string ns = "voxel_node_new";

                    set_marker_array(ns, position, scale, color, CubeMarker, voxel_marker_array);
                }
                else
                {
                    color = {1.0, 1.0, 1.0, 0.6};
                    std::string ns = "voxel_node_original";
                    set_marker_array(ns, position, scale, color, CubeMarker, voxel_marker_array);
                }
            }
        }

        void display_boundline(const Eigen::Array3d &min_corner, const Eigen::Array3d &max_corner)
        {
            std::vector<double> pos = {(min_corner[0] + max_corner[0]) / 2.f,
                                       (min_corner[1] + max_corner[1]) / 2.f,
                                       (min_corner[2] + max_corner[2]) / 2.f};

            std::vector<double> scale = {max_corner[0] - min_corner[0],
                                         max_corner[1] - min_corner[1],
                                         max_corner[2] - min_corner[2]};
            std::vector<double> color = {0.0, 0.0, 1.0, 0.1}; // Blue color
            set_marker_array("bound_line", pos, scale, color, CubeMarker, voxel_bd_marker_array);
        }

        void publish_voxel()
        {
            voxel_pub.publish(voxel_marker_array);
            bd_marker_pub.publish(voxel_bd_marker_array); // 发布边界线
        }
    };

    class pathDisplay3d
    {
    private:
        ros::Publisher path_pub;
        visualization_msgs::Marker path_marker;
        std::string ns; // 用于区分不同路径的命名空间

    public:
        pathDisplay3d(ros::NodeHandle nh, const std::string &ns) : path_pub(nh.advertise<visualization_msgs::Marker>(ns, 1, true))
        {
            this->ns = ns;
        }

        void display_path(const vec_Vec3f &path)
        {
            path_marker.header.stamp = ros::Time::now();
            path_marker.header.frame_id = "map";
            path_marker.ns = "path";
            path_marker.id = 0;
            path_marker.type = visualization_msgs::Marker::LINE_STRIP;
            path_marker.action = visualization_msgs::Marker::ADD;

            path_marker.pose.orientation.x = 0.0;
            path_marker.pose.orientation.y = 0.0;
            path_marker.pose.orientation.z = 0.0;
            path_marker.pose.orientation.w = 1.0;
            path_marker.scale.x = 1.0; // 设置线条宽度
            path_marker.scale.y = 1.0;

            if (ns == "pomp_path_marker")
            {
                path_marker.color.r = 0.0f;
                path_marker.color.g = 0.0f;
                path_marker.color.b = 1.0f;
            }
            else if (ns == "grid_path_marker")
            {
                path_marker.color.r = 1.0f;
                path_marker.color.g = 0.0f;
                path_marker.color.b = 0.0f;
            }
            else if (ns == "collision_path_marker")
            {
                path_marker.color.r = 1.0f;
                path_marker.color.g = 1.0f;
                path_marker.color.b = 0.0f;
            }
            else
            {
                throw std::invalid_argument("Unknown namespace for path marker");
            }
            path_marker.color.a = 1.0; // 设置透明度

            for (auto &pt : path)
            {
                geometry_msgs::Point p;
                p.x = pt[0];
                p.y = pt[1];
                p.z = pt[2];
                path_marker.points.push_back(p);
            }
        }

        void
        publish_path()
        {

            path_pub.publish(path_marker);
        }
    };
}