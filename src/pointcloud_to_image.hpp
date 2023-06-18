#ifndef POINTCLOUD_IMAGE_H
#define POINTCLOUD_IMAGE_H
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class PointCloudToImage
{
public:
    PointCloudToImage(const std::string &pointCloudPath, const std::string &imagePath);

    void processPointCloud();

private:
    std::string point_cloud;
    std::string image_path;

    Eigen::Matrix<float, 3, 4> Tr;
    Eigen::Matrix<float, 3, 4> P0;

    enum class Axis
    {
        X,
        Y,
        Z
    };

    void filterPointCloud(PointCloudT::Ptr cloud, Axis axis, float threshold);
    Eigen::MatrixXf pointCloudToMatrix(const PointCloudT::Ptr cloud);
    Eigen::MatrixXf filterPointsBehindCamera(const Eigen::MatrixXf &cam_xyz);
};

#endif // POINTCLOUD_IMAGE_H