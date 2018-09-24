#include "stdafx.h"
#include "IterativeClosestPointWrapper.h"


IterativeClosestPointWrapper::IterativeClosestPointWrapper()
{
}

pcl::PointCloud<pcl::PointXYZI> IterativeClosestPointWrapper::apply(pcl::PointCloud<pcl::PointXYZI>::Ptr finalCloud, std::mutex& finalCloudMutex, pcl::PointCloud<pcl::PointXYZI>::Ptr newCloud)
{
    m_icp.setMaximumIterations(50);
    m_icp.setTransformationEpsilon(0.0000001);
    m_icp.setMaxCorrespondenceDistance(0.5);
    //m_icp.setUseReciprocalCorrespondences(true);

    pcl::PointCloud<pcl::PointXYZI>::Ptr finalCloudCopy;
    {
        std::unique_lock<std::mutex> lock(finalCloudMutex);
        finalCloudCopy = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*finalCloud);
    }

    m_icp.setInputSource(newCloud);
    m_icp.setInputTarget(finalCloudCopy);
    pcl::PointCloud<pcl::PointXYZI> returnCloud;

    for (int i = 0; i < 15; i++)
    {
        //std::unique_lock<std::mutex> lock(finalCloudMutex);
        m_icp.align(returnCloud);
        std::cout << "Doing transformation n" << i << std::endl;
        //m_viewer.spinOnce();
        //m_cloudIcpColor = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(m_cloud_icp, 180, 20, 20);
        //m_viewer.updatePointCloud(m_cloud_icp, m_cloudIcpColor, "cloud_icp_v2");
    }

    return returnCloud;
}