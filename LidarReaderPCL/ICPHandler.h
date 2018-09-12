#pragma once

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

class ICPHandler
{
public:
    ICPHandler();
    ~ICPHandler();

    void setupICP();
    void downsample(double leafSize);
    void startVisualisation();

    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);

private:
    void loadPointCloud(std::vector<std::vector<double>> &data, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const;
    std::vector<std::vector<double>> parse2DCsvFile(std::string inputFileName) const;
    void print4x4Matrix(const Eigen::Matrix4d& matrix) const;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> m_icp;

    int m_iterations;
    bool m_nextIteration;
    Eigen::Matrix4d m_transformationMatrix;
    pcl::console::TicToc m_timer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_in, m_cloud_in_downsampled; // Original point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_tr, m_cloud_tr_downsampled; // Transformed point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_icp, m_cloud_icp_downsampled; // ICP output point cloud
};

