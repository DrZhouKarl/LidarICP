#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h> 
#include <pcl/point_types.h>

class DownSampler
{
public:
    DownSampler();
    ~DownSampler() = default;

    void setLeafSize(float ls);
    double apply(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud);
    

private:
    pcl::VoxelGrid<pcl::PointXYZI> m_vox;
    pcl::console::TicToc m_timer;
    float m_downsampleLeafSize;
};

