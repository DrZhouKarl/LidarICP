#include "stdafx.h"
#include "IterativeClosestPointWrapper.h"


IterativeClosestPointWrapper::IterativeClosestPointWrapper()
{
}

pcl::PointCloud<pcl::PointXYZI> IterativeClosestPointWrapper::apply(pcl::PointCloud<pcl::PointXYZI>::Ptr finalCloud, std::mutex& finalCloudMutex, pcl::PointCloud<pcl::PointXYZI>::Ptr newCloud)
{
    //m_icp.setMaximumIterations(50);
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

    for (int i = 0; i < 20; i++)
    {
        m_icp.align(returnCloud);
    }

    std::cout << "ICP Thread" << std::this_thread::get_id() << " done." << std::endl;


    return returnCloud;
}