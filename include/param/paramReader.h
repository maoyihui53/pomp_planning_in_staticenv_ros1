#pragma once
#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <utils/util.h>
using namespace std;
using namespace Eigen;
static std::vector<std::string> param_names;
// static double resolution;
// static string graphFilename;
// static string pcdFilename;
// static bool tree_dynamic_grow;

static string static_pd_path;
static double resolution = 0.1; // Default resolution, can be overridden by parameter
static string choice_env;
static int num_cylinders;
static int points_per_circle;
static std::vector<double> radius_min_max;
static std::vector<double> generate_cylinder_size;
static double z_step;

template <typename>
struct is_std_vector : std::false_type
{
};

template <typename U>
struct is_std_vector<std::vector<U>> : std::true_type
{
};

void iter_param(const ros::NodeHandle nh)
{
    std::vector<std::string> tmp_names;
    nh.getParamNames(tmp_names);
    int i = 0;
    for (const auto &param_name : tmp_names)
    {
        if (nh.hasParam(param_name))
        {
            if (param_name.find("/param_") == 0)
            {
                param_names.push_back(param_name);
                cout << param_name << endl;
            }
        }
        // printf(ANSI_COLOR_RED "----------------" ANSI_COLOR_RESET);
    }
}

template <typename T>
bool param_validation(T &input, string namespace_str, ros::NodeHandle nh, bool iter_)
{

    for (const auto &param_name : param_names)
    {
        // cout << param_name << endl;

        if (nh.hasParam(param_name))
        {
            if (param_name.find(namespace_str) == 0)
            {
                // printf(ANSI_COLOR_RED "here" ANSI_COLOR_RESET);

                if (iter_)
                {
                    if constexpr (is_std_vector<T>::value)
                    {
                        // Extract the type contained in the vector
                        using ValueType = typename T::value_type;

                        // Create a temporary value (using default constructor)
                        ValueType tmp{};
                        if (!nh.getParam(param_name, tmp))
                        {
                            printf(ANSI_COLOR_RED "Failed to get param %s" ANSI_COLOR_RESET, param_name.c_str());
                        }
                        else
                        {
                            input.push_back(tmp);
                        }
                    }
                }
                else
                {

                    if (!nh.getParam(param_name, input))
                    {
                        printf(ANSI_COLOR_RED "Failed to get param %s" ANSI_COLOR_RESET, param_name.c_str());
                    }

                    return true;
                }
            }
        }
        else
        {

            printf(ANSI_COLOR_RED "Has no %s " ANSI_COLOR_RESET, param_name.c_str());
            return false;
        }
    }

    return true;
}

bool read_param(const ros::NodeHandle &nh)
{
    if (!param_validation<double>(resolution, "/param_/resolution", nh, false))
        return false;
    if (!param_validation<string>(static_pd_path, "/param_/readPcd/staticGTFilename", nh, false))
        return false;
    if (!param_validation<string>(choice_env, "/param_/choice_env", nh, false))
        return false;
    if (!param_validation<int>(num_cylinders, "/param_/generateCylinerPcd/num_cylinders", nh, false))
        return false;
    if (!param_validation<int>(points_per_circle, "/param_/generateCylinerPcd/points_per_circle", nh, false))
        return false;
    if (!param_validation<std::vector<double>>(radius_min_max, "/param_/generateCylinerPcd/radius", nh, true))
        return false;
    if (!param_validation<std::vector<double>>(generate_cylinder_size, "/param_/generateCylinerPcd/size", nh, true))
        return false;
    if (!param_validation<double>(z_step, "/param_/generateCylinerPcd/z_step", nh, false))
        return false;
    // if (!param_validation<string>(pcdFilename, "/param_/pcdFilename", nh, false))
    //     return false;
    // if (!param_validation<bool>(tree_dynamic_grow, "/param_/tree_dynamic_grow", nh, false))
    //     return false;
    printf(ANSI_COLOR_YELLOW "---------------------------------------------\n" ANSI_COLOR_RESET);
    printf(ANSI_COLOR_GREEN "Parameters read successfully:\n" ANSI_COLOR_RESET);

    if (choice_env == "generateCylinerPcd")
    {
        printf(ANSI_COLOR_CYAN "    choice_env: %s\n" ANSI_COLOR_RESET, choice_env.c_str());
        printf(ANSI_COLOR_CYAN "        num_cyliners: %d\n" ANSI_COLOR_RESET, num_cylinders);
        printf(ANSI_COLOR_CYAN "        points_per_circle: %d\n" ANSI_COLOR_RESET, points_per_circle);
        printf(ANSI_COLOR_CYAN "        radius_min_max: [%f, %f]\n" ANSI_COLOR_RESET, radius_min_max[0], radius_min_max[1]);
        printf(ANSI_COLOR_CYAN "        size: [%f, %f, %f]\n" ANSI_COLOR_RESET, generate_cylinder_size[0], generate_cylinder_size[1], generate_cylinder_size[2]);
        printf(ANSI_COLOR_CYAN "        z_step: %f\n" ANSI_COLOR_RESET, z_step);
    }

    printf(ANSI_COLOR_YELLOW "----------------------------------------------\n" ANSI_COLOR_RESET);

    return true;
}

void readParem(const ros::NodeHandle &nh)
{

    iter_param(nh);

    if (!read_param(nh))
    {
        printf(ANSI_COLOR_RED "Failed to read parameters\n" ANSI_COLOR_RESET);
    }
}