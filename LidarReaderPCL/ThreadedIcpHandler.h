#pragma once

#include "IterativeClosestPointWrapper.h"

#include "ThreadSafeQueue.h"
#include "DownSampler.h"

#include <pcl/point_types.h>


class ThreadedIcpHandler
{
public:
    ThreadedIcpHandler();
    ~ThreadedIcpHandler() = default;

    void threadedFunction(IterativeClosestPointWrapper& icp, DownSampler& downsampler);

    void add(pcl::PointCloud<pcl::PointXYZI>::Ptr newCloud);
    void clear();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getFinalCloudPtr();
    std::mutex& getFinalCloudMutex();

    size_t toProcessQueueSize();
    size_t getProcessedCount();
    ThreadSafeQueue<pcl::PointCloud<pcl::PointXYZI>>& getFinishedCloudsQueue();

private:
    float mapVal(float value, float inputMin, float inputMax, float outputMin, float outputMax, bool clamp);

    DownSampler m_downSampler;
    ThreadSafeQueue<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_toProcessQueue;
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_finalCloud, m_lastFinishedCloud;
    std::mutex m_finalCloudMutex;
    std::atomic_bool m_firstCloud;
    std::atomic<int> m_counter;
    std::atomic<int> m_leafSize;

    ThreadSafeQueue<pcl::PointCloud<pcl::PointXYZI>> m_finishedClouds;
};

