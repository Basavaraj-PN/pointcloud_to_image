#include <Eigen/Core>
#include <iostream>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <string>
#include <yaml-cpp/yaml.h>

class YAMLConfigParser
{
public:
    std::string pointcloud_;
    std::string image_;
    std::string output_path_;
    Eigen::Matrix<float, 3, 4> Tr;
    Eigen::Matrix<float, 3, 4> P0;

    void parseConfig(const std::string &filename);
    void printConfig() const;
    void parseMatrix(const YAML::Node &node, Eigen::Matrix<float, 3, 4> &matrix);
};

void YAMLConfigParser::parseConfig(const std::string &filename)
{
    YAML::Node config = YAML::LoadFile(filename);

    pointcloud_ = config["pointcloud"].as<std::string>();
    image_ = config["image"].as<std::string>();
    output_path_ = config["output_path"].as<std::string>();

    parseMatrix(config["extrinsicTranslation"]["data"], Tr);
    parseMatrix(config["projectionMatrix"]["data"], P0);
}

void YAMLConfigParser::printConfig() const
{
    std::cout << "pointcloud: " << pointcloud_ << std::endl;
    std::cout << "image: " << image_ << std::endl;
    std::cout << "output_path: " << output_path_ << std::endl;

    std::cout << "extrinsicTranslation: " << std::endl;
    std::cout << Tr << std::endl;

    std::cout << "projectionMatrix: " << std::endl;
    std::cout << P0 << std::endl;

}

void YAMLConfigParser::parseMatrix(const YAML::Node &node, Eigen::Matrix<float, 3, 4> &matrix)
{
    std::vector<double> matrixData;
    for (const auto &value : node)
    {
        matrixData.push_back(static_cast<float>(value.as<double>()));
    }

    // Ensure the size of the matrixData vector is correct
    if (matrixData.size() != 12)
    {
        // Handle the error appropriately
    }

    // Copy the matrixData to the Eigen matrix
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            matrix(i, j) = matrixData[i * 4 + j];
        }
    }
}
