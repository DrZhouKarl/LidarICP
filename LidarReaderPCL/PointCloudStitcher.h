#pragma once

#include "ThreadSafeQueue.h"
#include "IMUSensor.h"
#include "ThreadedIcpHandler.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h> 
#include <pcl/io/vlp_grabber.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ndt.h>
#include <pcl/surface/mls.h>


struct path_leaf_string
{
    std::string operator()(const boost::filesystem::directory_entry& entry) const
    {
        return entry.path().leaf().string();
    }
};

class PointCloudStitcher
{
public:
    PointCloudStitcher();
    ~PointCloudStitcher() = default;

    void startVisualisation();

    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);
    void newCloudCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& newCloud);

private:
    ThreadedIcpHandler m_threadedIcp;
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr m_newestCloud;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> m_newCloudColor, m_finalCloudColor;
    bool m_isFirstCloud;
    std::mutex m_newestCloudMutex;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_finisedClouds;
    std::vector<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>> m_finisedCloudsColours;




    void print4x4Matrix(const Eigen::Matrix4d& matrix) const;
    void read_directory(const std::string& name, std::vector<std::string>& v);
    void setupVisualisation();
    void setupIcpParameters();

    double applyTransformation(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud, IMUSensor& imu);

    void applyStatisticalOutliner();
    void applyMLSSmoothing();

    void applyNdtSystem();










    pcl::VLPGrabber m_grabber;
    boost::signals2::connection m_cloudConnection;
    boost::mutex m_cloudMutex;

    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> m_sor;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree;
    pcl::MovingLeastSquaresOMP<pcl::PointXYZI, pcl::PointNormal> mls;
    pcl::PointCloud<pcl::PointNormal> mls_points;
    
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

    std::vector<pcl::PointCloud<pcl::PointXYZI>> m_cloudsToSave;

    IMUSensor m_imu;

    int m_iterations;
    bool m_nextIteration;
    bool m_newCloud;
    bool m_recordFrames;
    int m_recorderCounter;
    bool m_useRecodings;
    int m_mergedCloudsCounter;
    double m_elapsedTimeTotal;
    Eigen::Matrix4d m_transformationMatrix;
    pcl::console::TicToc m_timer;

    std::stringstream m_viewerString;
    pcl::visualization::PCLVisualizer m_viewer;
    float m_backgroundColor;
    float m_textColor;
   // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> m_cloudNewTargetColor;

    ThreadSafeQueue<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_queue, m_recentCloudsQueue;

    pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud_in; // Original point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud_icp; // ICP output point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud_recent_transformed; // ICP output point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_lastNclouds;
};

