#ifndef POINTCLOUD_IMAGE_H
#define POINTCLOUD_IMAGE_H
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class PointCloudToImage
{
public:
    PointCloudToImage();
    void projectPointCloud(const std::string &point_cloud, const std::string &image_path,
                           const Eigen::Matrix<float, 3, 4> &Tr_, const Eigen::Matrix<float, 3, 4> &P0_,
                           bool write_image, const std::string output_path);

private:
    std::string point_cloud;
    std::string image_path;
    cv::Mat image;
    int imwidth;
    int imheight;

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
    std::string getImageName(const std::string &imagePath);
};

#endif // POINTCLOUD_IMAGE_H