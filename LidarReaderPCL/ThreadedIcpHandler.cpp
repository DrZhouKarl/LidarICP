#include "stdafx.h"
#include "ThreadedIcpHandler.h"

#include <pcl/common/common.h>

#include <thread>
#include <future>


ThreadedIcpHandler::ThreadedIcpHandler()
    : m_finalCloud(new pcl::PointCloud<pcl::PointXYZI>)
      , m_firstCloud(true)
      , m_counter(0)
      , m_paused(false)
      , m_minLimit(-100)
      , m_maxLimit(100)
{
    m_downSampler.setLeafSize(0.13);

    int threadCount = 6;
    for (int i = 0; i < threadCount; i++)
    {
        std::thread t1([=]()
            {
                KeyPointRegistrationWrapper kpr;
                IterativeClosestPointWrapper icp;
                DownSampler downSampler;
                downSampler.setLeafSize(0.2);
                while (true)
                {
                    if (!m_paused)
                    {
                        threadedFunction(icp, kpr, downSampler);
                    } else
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                }
            });
        t1.detach();
    }
}

void ThreadedIcpHandler::threadedFunction(IterativeClosestPointWrapper& icp, KeyPointRegistrationWrapper& kpr, DownSampler& downsampler)
{
    int count = mapVal(m_toProcessQueue.size(), 0, 100, 0, 10, false);
    std::cout << "Skipping " << count << " times." << std::endl;
    for (int i = 0; i < count; i++)
    {
        m_toProcessQueue.waitAndPop();
    }

    auto toProcess = m_toProcessQueue.waitAndPop();

    {
        /*
        m_passThroughFilter.setInputCloud(toProcess);
        m_passThroughFilter.setFilterFieldName("x");
        m_passThroughFilter.setFilterLimits(0, 0.1);
        //m_passThroughFilter.setFilterLimitsNegative (true);
        m_passThroughFilter.filter(*toProcess);
        pcl::PointXYZI minPt, maxPt;
        pcl::getMinMax3D(*toProcess, minPt, maxPt);
        std::cout << "Max x: " << maxPt.x << std::endl;
        std::cout << "Max y: " << maxPt.y << std::endl;
        std::cout << "Max z: " << maxPt.z << std::endl;
        std::cout << "Min x: " << minPt.x << std::endl;
        std::cout << "Min y: " << minPt.y << std::endl;
        std::cout << "Min z: " << minPt.z << std::endl;
        float min = -1;
        float max = 1;
        */

        /*
        boxFilter.setMin(Eigen::Vector4f(m_minLimit, m_minLimit, m_minLimit, 1.0));
        boxFilter.setMax(Eigen::Vector4f(m_maxLimit, m_maxLimit, m_maxLimit, 1.0));
        boxFilter.setInputCloud(toProcess);
        boxFilter.filter(*toProcess);
        */
    }

    {
        downsampler.apply(toProcess);
    }
    /*
    {
        auto finished = kpr.apply(m_finalCloud, m_finalCloudMutex, toProcess);
        m_lastFinishedCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(finished);
        std::unique_lock<std::mutex> lock(m_finalCloudMutex);
        *m_finalCloud += finished;
    }
    */
    {
        auto finished = icp.apply(m_finalCloud, m_finalCloudMutex, toProcess);
        m_lastFinishedCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(finished);
        std::unique_lock<std::mutex> lock(m_finalCloudMutex);
        *m_finalCloud += finished;
    }
    
    m_counter += 1;

    if (m_counter % 3 == 0)
    {
        std::unique_lock<std::mutex> lock(m_finalCloudMutex);
        downsampler.apply(m_finalCloud);
        m_finishedClouds.push(*m_finalCloud);
        m_firstCloud = false;
    }
    //m_finishedProcessQueue.push(boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(finished));
}


void ThreadedIcpHandler::add(pcl::PointCloud<pcl::PointXYZI>::Ptr newCloud)
{
    if (m_firstCloud)
    {
        /*
        boxFilter.setMin(Eigen::Vector4f(m_minLimit, m_minLimit, m_minLimit, 1.0));
        boxFilter.setMax(Eigen::Vector4f(m_maxLimit, m_maxLimit, m_maxLimit, 1.0));
        boxFilter.setInputCloud(newCloud);
        boxFilter.filter(*newCloud);
        */
        m_downSampler.apply(newCloud);

        std::unique_lock<std::mutex> lock(m_finalCloudMutex);
        m_finalCloud = newCloud;
        m_lastFinishedCloud = newCloud;
        m_firstCloud = false;
    }

    if (!m_paused)
    {
        auto cloudCopy = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*newCloud);
        m_toProcessQueue.push(cloudCopy);
    }
    std::cout << "To process queue size: " << m_toProcessQueue.size() << std::endl;
    std::cout << "Processed queue size: " << m_counter << std::endl;
}

void ThreadedIcpHandler::clear()
{
    m_firstCloud = true;
    m_toProcessQueue.clear();
    m_counter = 0;
}

void ThreadedIcpHandler::togglePause()
{
    m_paused = !m_paused;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ThreadedIcpHandler::getFinalCloudPtr()
{
    return m_finalCloud;
}

std::mutex& ThreadedIcpHandler::getFinalCloudMutex()
{
    return m_finalCloudMutex;
}

size_t ThreadedIcpHandler::toProcessQueueSize()
{
    return m_toProcessQueue.size();
}

size_t ThreadedIcpHandler::getProcessedCount()
{
    return m_counter;
}

ThreadSafeQueue<pcl::PointCloud<pcl::PointXYZI>>& ThreadedIcpHandler::getFinishedCloudsQueue()
{
    return m_finishedClouds;
}

float ThreadedIcpHandler::mapVal(float value, float inputMin, float inputMax, float outputMin, float outputMax, bool clamp)
{
    if (fabs(inputMin - inputMax) < FLT_EPSILON)
    {
        return outputMin;
    }
    else
    {
        float outVal = ((value - inputMin) / (inputMax - inputMin) * (outputMax - outputMin) + outputMin);

        if (clamp)
        {
            if (outputMax < outputMin)
            {
                if (outVal < outputMax)outVal = outputMax;
                else if (outVal > outputMin)outVal = outputMin;
            }
            else
            {
                if (outVal > outputMax)outVal = outputMax;
                else if (outVal < outputMin)outVal = outputMin;
            }
        }
        return outVal;
    }
}
