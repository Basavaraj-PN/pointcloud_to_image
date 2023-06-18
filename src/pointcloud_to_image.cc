#include "pointcloud_to_image.hpp"
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

PointCloudToImage::PointCloudToImage(const Eigen::Matrix<float, 3, 4> &Tr_, const Eigen::Matrix<float, 3, 4> &P0_,
                                     bool write_image_, const std::string output_path_)
    : Tr(Tr_), P0(P0_), write_image(write_image_), output_path(output_path_)
{
}
void PointCloudToImage::projectPointCloud(const std::string &point_cloud, const std::string &image_path)
{
    // Load the image
    image = cv::imread(image_path);
    imwidth = image.cols;
    imheight = image.rows;

    // Load the point cloud from PCD file
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(point_cloud, *cloud) == -1)
    {
        std::cerr << "Failed to load PCD file." << std::endl;
        return;
    }

    // Filter the point cloud along the X axis
    filterPointCloud(cloud, Axis::X, 0.0);

    // Convert point cloud to matrix
    Eigen::MatrixXf cloudMatrix = pointCloudToMatrix(cloud);

    // Project the point cloud into camera coordinates
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> cam_xyz = Tr * (cloudMatrix.transpose());

    // Filter points behind the camera
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> cam_xyz_filter = filterPointsBehindCamera(cam_xyz);

    // Extract depth values
    Eigen::VectorXf depth = cam_xyz.row(2).transpose();

    // Normalize the coordinates by dividing by the depth
    for (int i = 0; i < cam_xyz_filter.cols(); ++i)
    {
        cam_xyz_filter.col(i) /= cam_xyz_filter(2, i);
    }

    // Convert coordinates to homogeneous form
    Eigen::MatrixXf cam_xyz_homogeneous(cam_xyz_filter.rows() + 1, cam_xyz_filter.cols());
    cam_xyz_homogeneous.block(0, 0, cam_xyz_filter.rows(), cam_xyz_filter.cols()) = cam_xyz_filter;
    cam_xyz_homogeneous.row(cam_xyz_filter.rows()).setOnes();

    // Perform projection using the projection matrix
    auto projection = P0 * cam_xyz_homogeneous;

    // Convert projected coordinates to pixel coordinates
    Eigen::MatrixXi pixel_coordinates = projection.cast<int>().topLeftCorner(2, projection.cols()).array().round();

    // Check if pixel coordinates are within the image boundaries
    Eigen::Array<bool, Eigen::Dynamic, 1> indices = ((pixel_coordinates.row(0).array() < imwidth) &&
                                                     (pixel_coordinates.row(0).array() >= 0) &&
                                                     (pixel_coordinates.row(1).array() < imheight) &&
                                                     (pixel_coordinates.row(1).array() >= 0));

    // Draw circles at valid pixel coordinates on the image
    for (int i = 0; i < indices.size(); ++i)
    {
        if (indices(i))
        {
            int x = pixel_coordinates(0, i);
            int y = pixel_coordinates(1, i);
            cv::circle(image, cv::Point(x, y), 1, cv::Scalar(227, 97, 255), -1);
        }
    }
    // Save the image with projected points

    if (write_image)
    {
        cv::imwrite(output_path + "/" + getImageName(image_path), image);
        std::cout << "Saved image with projected points: " << output_path + getImageName(image_path) << std::endl;
    }

    // Display the image with projected points
    cv::imshow("PointCloud In Image Plane ", image);
    cv::waitKey(0);
}

std::string PointCloudToImage::getImageName(const std::string &imagePath)
{
    std::filesystem::path path(imagePath);
    return path.filename().string();
}

void PointCloudToImage::filterPointCloud(PointCloudT::Ptr cloud, Axis axis, float threshold)
{
    PointCloudT::Ptr filteredCloud(new PointCloudT);
    for (const auto &point : cloud->points)
    {
        float coordinate;
        switch (axis)
        {
        case Axis::X:
            coordinate = point.x;
            break;
        case Axis::Y:
            coordinate = point.y;
            break;
        case Axis::Z:
            coordinate = point.z;
            break;
        default:
            return; // Invalid axis
        }

        if (coordinate > threshold)
        {
            filteredCloud->points.push_back(point);
        }
    }

    filteredCloud->width = filteredCloud->points.size();
    filteredCloud->height = 1;
    filteredCloud->is_dense = true;

    pcl::copyPointCloud(*filteredCloud, *cloud);
}

Eigen::MatrixXf PointCloudToImage::pointCloudToMatrix(const PointCloudT::Ptr cloud)
{
    Eigen::MatrixXf matrix(cloud->size(), 4);
    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        const auto &point = cloud->points[i];
        matrix(i, 0) = point.x;
        matrix(i, 1) = point.y;
        matrix(i, 2) = point.z;
        matrix(i, 3) = 1.0f;
    }
    return matrix;
}

Eigen::MatrixXf PointCloudToImage::filterPointsBehindCamera(const Eigen::MatrixXf &cam_xyz)
{
    Eigen::MatrixXf filtered_cam_xyz;

    std::vector<int> indices;
    for (int i = 0; i < cam_xyz.cols(); ++i)
    {
        if (cam_xyz(2, i) > 0)
        {
            indices.push_back(i);
        }
    }
    int num_points = indices.size();
    filtered_cam_xyz.resize(cam_xyz.rows(), num_points);
    for (int i = 0; i < num_points; i++)
    {
        filtered_cam_xyz.col(i) = cam_xyz.col(indices[i]);
    }

    return filtered_cam_xyz;
}
