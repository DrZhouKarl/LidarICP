#pragma once

#include "IterativeClosestPointWrapper.h"

#include "KeyPointRegistrationWrapper.h"
#include "ThreadSafeQueue.h"
#include "DownSampler.h"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/surface/mls.h>


class ThreadedIcpHandler
{
public:
    ThreadedIcpHandler();
    ~ThreadedIcpHandler() = default;

    void threadedFunction(IterativeClosestPointWrapper& icp, KeyPointRegistrationWrapper& kpr, DownSampler& downsampler);

    void add(pcl::PointCloud<pcl::PointXYZI>::Ptr newCloud);
    void clear();
    void togglePause();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getFinalCloudPtr();
    std::mutex& getFinalCloudMutex();

    size_t toProcessQueueSize();
    size_t getProcessedCount();
    ThreadSafeQueue<pcl::PointCloud<pcl::PointXYZI>>& getFinishedCloudsQueue();

private:
    float mapVal(float value, float inputMin, float inputMax, float outputMin, float outputMax, bool clamp);

    DownSampler m_downSampler;
    pcl::PassThrough<pcl::PointXYZI> m_passThroughFilter;
    pcl::CropBox<pcl::PointXYZI> boxFilter;
    
    ThreadSafeQueue<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_toProcessQueue;
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_finalCloud, m_lastFinishedCloud;
    std::mutex m_finalCloudMutex;
    std::atomic_bool m_firstCloud, m_paused;
    std::atomic<int> m_counter;
    std::atomic<int> m_minLimit, m_maxLimit;

    ThreadSafeQueue<pcl::PointCloud<pcl::PointXYZI>> m_finishedClouds;
};

