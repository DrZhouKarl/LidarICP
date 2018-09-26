#pragma once

#include <pcl/point_types.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/point_cloud.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/search/kdtree.h>
//#include <pcl/registration/transformation_estimation_svd.h>
//#include <pcl/features/integral_image_normal.h>

#include <mutex>

class KeyPointRegistrationWrapper
{
public:
    KeyPointRegistrationWrapper();
    ~KeyPointRegistrationWrapper();

    pcl::PointCloud<pcl::PointXYZI> apply(pcl::PointCloud<pcl::PointXYZI>::Ptr finalCloud, std::mutex& finalCloudMutex, pcl::PointCloud<pcl::PointXYZI>::Ptr newCloud);

private:
    boost::shared_ptr<pcl::Keypoint<pcl::PointXYZI, pcl::PointXYZI> > keypoint_detector_;
    void detectKeypoints(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr keyPoints);
    void extractDescriptors(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr keyPoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features);
    void findCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source, pcl::PointCloud<pcl::FPFHSignature33>::Ptr target, std::vector<int>& correspondences);
    void filterCorrespondences(std::vector<int>& source2target_, std::vector<int>& target2source_, pcl::CorrespondencesPtr correspondences_, pcl::PointCloud<pcl::PointXYZI>::Ptr keyPointsSource, pcl::PointCloud<pcl::PointXYZI>::Ptr keyPointsTarget);

    pcl::Feature<pcl::PointXYZI, pcl::FPFHSignature33>::Ptr m_featureExtractor;

    //pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointWithScale> sift;
    //pcl::ISSKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> iss_detector;

    //double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud);
};

