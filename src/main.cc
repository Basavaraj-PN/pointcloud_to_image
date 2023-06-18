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

    PointCloudToImage p(configParser.pointcloud_, configParser.image_, configParser.Tr, configParser.P0);
    p.processPointCloud();

    return 0;
}
