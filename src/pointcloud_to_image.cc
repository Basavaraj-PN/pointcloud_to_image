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

PointCloudToImage::PointCloudToImage(const std::string &pointCloudPath, const std::string &imagePath,
                                     const Eigen::Matrix<float, 3, 4> &Tr_, const Eigen::Matrix<float, 3, 4> &P0_)
    : point_cloud(pointCloudPath), image_path(imagePath), Tr(Tr_), P0(P0_)
{
    // Constructor implementation...
}

void PointCloudToImage::processPointCloud()
{
    cv::Mat image = cv::imread(image_path);
    int imwidth = image.cols;
    int imheight = image.rows;

    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(point_cloud, *cloud) == -1)
    {
        std::cerr << "Failed to load PCD file." << std::endl;
        return;
    }

    filterPointCloud(cloud, Axis::X, 0.0);
    Eigen::MatrixXf cloudMatrix = pointCloudToMatrix(cloud);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> cam_xyz = Tr * (cloudMatrix.transpose());
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> cam_xyz_filter = filterPointsBehindCamera(cam_xyz);
    Eigen::VectorXf depth = cam_xyz.row(2).transpose();

    for (int i = 0; i < cam_xyz_filter.cols(); ++i)
    {
        cam_xyz_filter.col(i) /= cam_xyz_filter(2, i);
    }

    Eigen::MatrixXf cam_xyz_homogeneous(cam_xyz_filter.rows() + 1, cam_xyz_filter.cols());

    cam_xyz_homogeneous.block(0, 0, cam_xyz_filter.rows(), cam_xyz_filter.cols()) = cam_xyz_filter;
    cam_xyz_homogeneous.row(cam_xyz_filter.rows()).setOnes();

    auto projection = P0 * cam_xyz_homogeneous;
    Eigen::MatrixXi pixel_coordinates = projection.cast<int>().topLeftCorner(2, projection.cols()).array().round();

    Eigen::Array<bool, Eigen::Dynamic, 1> indices = ((pixel_coordinates.row(0).array() < imwidth) &&
                                                     (pixel_coordinates.row(0).array() >= 0) &&
                                                     (pixel_coordinates.row(1).array() < imheight) &&
                                                     (pixel_coordinates.row(1).array() >= 0));

    for (int i = 0; i < indices.size(); ++i)
    {
        if (indices(i))
        {
            int x = pixel_coordinates(0, i);
            int y = pixel_coordinates(1, i);
            cv::circle(image, cv::Point(x, y), 1, cv::Scalar(227, 97, 255), -1);
        }
    }

    cv::imshow("Image with Pixel Coordinates", image);
    cv::waitKey(0);
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
