
#include "pointcloud_to_image.hpp"
#include "yaml_parser.hpp"
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

int main()
{

    // Load thhe yaml file
    YAMLConfigParser configParser;
    configParser.parseConfig("/home/omnipotent/Desktop/Desktop/pointcloud_to_image/config/config.yaml");

    // Create PointCloudToImage object
    PointCloudToImage project(configParser.Tr, configParser.P0, configParser.write_image_, configParser.output_path_);

    // project pointcloud on image
    project.projectPointCloud(configParser.pointcloud_, configParser.image_);

    return 0;
}
