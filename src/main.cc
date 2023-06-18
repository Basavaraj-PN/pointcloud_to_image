
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

    // Create PointCloudToImage object with TR and P0
    PointCloudToImage pointcloudtoimage(configParser.Tr, configParser.P0);

    // project pointcloud on image
    pointcloudtoimage.projectPointCloud(configParser.pointcloud_, configParser.image_);

    // save projection
    pointcloudtoimage.saveProjection(configParser.output_path_);

    // Display output
    pointcloudtoimage.projectionShow();
    
    return 0;
}
