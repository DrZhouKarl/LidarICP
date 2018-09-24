#pragma once

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <mutex>

class IterativeClosestPointWrapper
{
public:
    IterativeClosestPointWrapper();
    ~IterativeClosestPointWrapper() = default;

    pcl::PointCloud<pcl::PointXYZI> apply(pcl::PointCloud<pcl::PointXYZI>::Ptr finalCloud, std::mutex& finalCloudMutex, pcl::PointCloud<pcl::PointXYZI>::Ptr newCloud);

private:
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> m_icp;
};

