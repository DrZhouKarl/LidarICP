#include "stdafx.h"
#include "DownSampler.h"


DownSampler::DownSampler()
    : m_downsampleLeafSize(0.1)
{
}

void DownSampler::setLeafSize(float ls)
{
    m_downsampleLeafSize = ls;
}

double DownSampler::apply(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud)
{
    m_timer.tic();

    m_vox.setInputCloud(pointCloud);
    m_vox.setLeafSize(m_downsampleLeafSize, m_downsampleLeafSize, m_downsampleLeafSize);
    m_vox.filter(*pointCloud);

    return m_timer.toc();
}