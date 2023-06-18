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
    // Constructor
    PointCloudToImage(const Eigen::Matrix<float, 3, 4> &Tr_, const Eigen::Matrix<float, 3, 4> &P0_,
                      bool write_image_, const std::string output_path_);

    // Method to project point cloud onto the image
    void projectPointCloud(const std::string &point_cloud, const std::string &image_path);

private:
    Eigen::Matrix<float, 3, 4> Tr;     // Transformation matrix for the camera
    Eigen::Matrix<float, 3, 4> P0;     // Projection matrix for the camera
    bool write_image;                  // Flag to determine whether to save the image
    const std::string output_path;     // Output path for saving the image

    std::string point_cloud;           // Path to the point cloud file
    std::string image_path;            // Path to the input image file
    cv::Mat image;                     // Loaded image
    int imwidth;                       // Image width
    int imheight;                      // Image height

    enum class Axis
    {
        X,
        Y,
        Z
    };

    // Filter the point cloud based on a given axis and threshold
    void filterPointCloud(PointCloudT::Ptr cloud, Axis axis, float threshold);

    // Convert the point cloud to a matrix representation
    Eigen::MatrixXf pointCloudToMatrix(const PointCloudT::Ptr cloud);

    // Filter out points behind the camera
    Eigen::MatrixXf filterPointsBehindCamera(const Eigen::MatrixXf &cam_xyz);

    // Get the image name from the image path
    std::string getImageName(const std::string &imagePath);
};

#endif // POINTCLOUD_IMAGE_H
