#pragma once

#include "ThreadSafeQueue.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/io/vlp_grabber.h>

class ICPHandler
{
public:
    ICPHandler();
    ~ICPHandler();

    void setupNewCycle();
    void downsample(double leafSize);
    void startVisualisation();

    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);
    void newCloudCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& newCloud);

private:
    void loadPointCloud(std::vector<std::vector<double>> &data, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) const;
    std::vector<std::vector<double>> parse2DCsvFile(std::string inputFileName) const;
    void print4x4Matrix(const Eigen::Matrix4d& matrix) const;
    void setupVisualisation();

    pcl::VLPGrabber m_grabber;
    boost::signals2::connection m_cloudConnection;
    boost::mutex m_cloudMutex;

    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> m_icp;

    int m_iterations;
    bool m_nextIteration;
    bool m_newCloud;
    Eigen::Matrix4d m_transformationMatrix;
    pcl::console::TicToc m_timer;
    float m_downsampleLeafSize;

    std::stringstream m_viewerString;
    pcl::visualization::PCLVisualizer m_viewer;
    float m_backgroundColor;
    float m_textColor;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> m_cloudInColor, m_cloudIcpColor;

    ThreadSafeQueue<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_queue;

    pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud_in; // Original point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud_new; // Transformed point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud_icp; // ICP output point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud_recent_transformed; // ICP output point cloud
};

