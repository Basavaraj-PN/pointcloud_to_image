#include <Eigen/Core>
#include <iostream>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "pointcloud_to_image.hpp"
#include "yaml_parser.hpp"
#include <Eigen/Core>
#include <string>
#include <yaml-cpp/yaml.h>

int main()
{
    spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] [thread %t] %v");
    auto logger = spdlog::stdout_logger_mt("logger");
    // Log a message
    logger->info("Logging with spdlog!");

    YAMLConfigParser configParser;
    configParser.parseConfig("/home/omnipotent/Desktop/Desktop/pointcloud_to_image/config/config.yaml");
    configParser.printConfig();

    std::string pointCloudPath = "/home/omnipotent/Desktop/Desktop/VIO-Tutorials/resources/output.pcd";
    std::string imagePath = "/media/omnipotent/HDD/Dataset/data_odometry_gray/dataset/sequences/00/image_0/000000.png";

    PointCloudToImage p(pointCloudPath, imagePath);
    p.processPointCloud();

    return 0;
}
