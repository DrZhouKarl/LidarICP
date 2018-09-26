#include "stdafx.h"
#include "KeyPointRegistrationWrapper.h"

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

KeyPointRegistrationWrapper::KeyPointRegistrationWrapper()
    : m_featureExtractor(new pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33>)
{
    //m_featureExtractor->setSearchMethod(pcl::search::Search<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>));
    //m_featureExtractor->setRadiusSearch(0.5);
    m_featureExtractor->setKSearch(100);
    pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointXYZI>* sift3D = new pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointXYZI>;
    sift3D->setScales(0.1f, 6, 10);
    sift3D->setMinimumContrast(0.0001);
    keypoint_detector_.reset(sift3D);
    

    /*
    pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI>* harris3D = new pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI>(pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI>::HARRIS);
    harris3D->setNonMaxSupression(true);
    harris3D->setRadius(0.01f);
    harris3D->setRadiusSearch(0.01f);
    keypoint_detector_.reset(harris3D);
    */
}


KeyPointRegistrationWrapper::~KeyPointRegistrationWrapper()
{
}

pcl::PointCloud<pcl::PointXYZI> KeyPointRegistrationWrapper::apply(pcl::PointCloud<pcl::PointXYZI>::Ptr finalCloud, std::mutex& finalCloudMutex, pcl::PointCloud<pcl::PointXYZI>::Ptr newCloud)
{
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr finalCloudCopy(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr returnCloud(new pcl::PointCloud<pcl::PointXYZI>());
    {
        std::unique_lock<std::mutex> lock(finalCloudMutex);
        pcl::copyPointCloud(*finalCloud, *finalCloudCopy);
    }
    pcl::copyPointCloud(*newCloud, *returnCloud);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypointsFinal(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypointsNew(new pcl::PointCloud<pcl::PointXYZI>());
    detectKeypoints(finalCloudCopy, keypointsFinal);
    detectKeypoints(returnCloud, keypointsNew);

    if(keypointsNew->empty() || keypointsFinal->empty())
    {
        return *returnCloud;
    }

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr featuresFinal(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr featuresNew(new pcl::PointCloud<pcl::FPFHSignature33>());
    extractDescriptors(returnCloud, keypointsNew, featuresNew);
    extractDescriptors(finalCloudCopy, keypointsFinal, featuresFinal);

    std::vector<int> newToFinal;
    std::vector<int> finalToNew;

    findCorrespondences(featuresNew, featuresFinal, newToFinal);
    findCorrespondences(featuresFinal, featuresNew, finalToNew);

    pcl::CorrespondencesPtr correspondences_(new pcl::Correspondences());
    Eigen::Matrix4f initial_transformation_matrix_;

    {
        pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
        est.setInputSource(featuresNew);
        est.setInputTarget(featuresFinal);
        est.determineCorrespondences(*correspondences);

        // Correspondance rejection RANSAC

        pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector_sac;
        pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
        rejector_sac.setInputSource(keypointsNew);
        rejector_sac.setInputTarget(keypointsFinal);
        rejector_sac.setInlierThreshold(2.5); // distance in m, not the squared distance
        rejector_sac.setMaximumIterations(1000000);
        rejector_sac.setRefineModel(false);
        rejector_sac.setInputCorrespondences(correspondences);;
        rejector_sac.getCorrespondences(*correspondences_filtered);
        correspondences.swap(correspondences_filtered);
        std::cout << correspondences->size() << " vs. " << correspondences_filtered->size() << std::endl;
        initial_transformation_matrix_ = rejector_sac.getBestTransformation();   // Transformation Estimation method
    }

    /*
    {
        filterCorrespondences(newToFinal, finalToNew, correspondences_, keypointsNew, keypointsFinal);

        
        std::cout << "initial alignment..." << std::flush;
        pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Ptr transformation_estimation(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI>);

        transformation_estimation->estimateRigidTransformation(*keypointsNew, *keypointsFinal, *correspondences_, initial_transformation_matrix_);

    }
    */
    pcl::PointCloud<pcl::PointXYZI> rrCloud;
    pcl::transformPointCloud(*returnCloud, rrCloud, initial_transformation_matrix_);
    std::cout << "OK" << std::endl;
    return rrCloud;


    /*
    const float min_scale = 0.1f;
    const int n_octaves = 6;
    const int n_scales_per_octave = 10;
    const float min_contrast = 0.5f;

    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(newCloud);
    sift.compute(result);
    */

    /*
    double model_resolution = computeCloudResolution(newCloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    iss_detector.setSearchMethod(tree);
    iss_detector.setSalientRadius(10 * model_resolution);
    iss_detector.setNonMaxRadius(8 * model_resolution);
    iss_detector.setThreshold21(0.2);
    iss_detector.setThreshold32(0.2);
    iss_detector.setMinNeighbors(10);
    iss_detector.setNumberOfThreads(10);
    iss_detector.setInputCloud(newCloud);
    iss_detector.compute((*source_keypoints));
    pcl::PointIndicesConstPtr keypoints_indices = iss_detector.getKeypointsIndices();
    std::cout << "No of ISS points in the result are " << (*source_keypoints).points.size() << std::endl;

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(newCloud);
    normalEstimation.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud< pcl::Normal>);
    normalEstimation.setRadiusSearch(0.2);
    normalEstimation.compute(*source_normals);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(newCloud);
    fpfh.setInputNormals(source_normals);
    fpfh.setIndices(keypoints_indices);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(0.2);
    fpfh.compute(*source_features);


    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints(new pcl::PointCloud<pcl::PointXYZI>());
    {
        std::unique_lock<std::mutex> lock(finalCloudMutex);
        double model_resolution_1 = computeCloudResolution(finalCloud);

        pcl::ISSKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> iss_detector_1;
        

        iss_detector_1.setSearchMethod(tree);
        iss_detector_1.setSalientRadius(10 * model_resolution_1);
        iss_detector_1.setNonMaxRadius(8 * model_resolution_1);
        iss_detector_1.setThreshold21(0.2);
        iss_detector_1.setThreshold32(0.2);
        iss_detector_1.setMinNeighbors(10);
        iss_detector_1.setNumberOfThreads(10);
        iss_detector_1.setInputCloud(finalCloud);
        iss_detector_1.compute((*target_keypoints));
        pcl::PointIndicesConstPtr keypoints_indices_1 = iss_detector_1.getKeypointsIndices();


        // Compute the normals
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimation_1;
        normalEstimation_1.setInputCloud(finalCloud);
        normalEstimation_1.setSearchMethod(tree);

        pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud< pcl::Normal>);
        normalEstimation_1.setRadiusSearch(0.2);
        normalEstimation_1.compute(*target_normals);


        
        pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh1;
        fpfh1.setInputCloud(finalCloud);
        fpfh1.setInputNormals(target_normals);
        fpfh1.setIndices(keypoints_indices_1);
        fpfh1.setSearchMethod(tree);
        fpfh1.setRadiusSearch(0.2);
        fpfh1.compute(*target_features);

    }


    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    est.setInputSource(source_features);
    est.setInputTarget(target_features);
    est.determineCorrespondences(*correspondences);

    // Duplication rejection Duplicate

    pcl::CorrespondencesPtr correspondences_result_rej_one_to_one(new pcl::Correspondences());
    pcl::registration::CorrespondenceRejectorOneToOne corr_rej_one_to_one;
    corr_rej_one_to_one.setInputCorrespondences(correspondences);
    corr_rej_one_to_one.getCorrespondences(*correspondences_result_rej_one_to_one);


    // Correspondance rejection RANSAC

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector_sac;
    pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
    rejector_sac.setInputSource(source_keypoints);
    rejector_sac.setInputTarget(target_keypoints);
    rejector_sac.setInlierThreshold(2.5); // distance in m, not the squared distance
    rejector_sac.setMaximumIterations(1000000);
    rejector_sac.setRefineModel(false);
    rejector_sac.setInputCorrespondences(correspondences_result_rej_one_to_one);;
    rejector_sac.getCorrespondences(*correspondences_filtered);
    correspondences.swap(correspondences_filtered);
    std::cout << correspondences->size() << " vs. " << correspondences_filtered->size() << std::endl;
    transform = rejector_sac.getBestTransformation();   // Transformation Estimation method 1


                                                        // Transformation Estimation method 2
                                                        //pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;
                                                        //transformation_estimation.estimateRigidTransformation(*source_keypoints, *target_keypoints, *correspondences, transform);
    std::cout << "Estimated Transform:" << std::endl << transform << std::endl;

    //pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr final_output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*newCloud, *newCloud, transform);
    */
}

void KeyPointRegistrationWrapper::detectKeypoints(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr keyPoints)
{
    std::cout << "keypoint detection..." << std::flush;
    keypoint_detector_->setInputCloud(cloud);
    keypoint_detector_->compute(*keyPoints);
    std::cout << "OK. keypoints found: " << keyPoints->points.size() << std::endl;
}

void KeyPointRegistrationWrapper::extractDescriptors(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr keyPoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr kpts(new pcl::PointCloud<pcl::PointXYZI>);
    kpts->points.resize(keyPoints->points.size());

    pcl::copyPointCloud(*keyPoints, *kpts);

    pcl::FeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33>> (m_featureExtractor);
    
    m_featureExtractor->setSearchSurface(cloud);
    m_featureExtractor->setInputCloud(kpts);
    
    if (feature_from_normals)
    {
        std::cout << "normal estimation..." << std::flush;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new  pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimation;
        normal_estimation.setSearchMethod(pcl::search::Search<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>));
        normal_estimation.setRadiusSearch(0.1);
        normal_estimation.setInputCloud(cloud);
        normal_estimation.compute(*normals);
        feature_from_normals->setInputNormals(normals);
        std::cout << "OK" << std::endl;
    }

    std::cout << "descriptor extraction..." << std::flush;
    m_featureExtractor->compute(*features);
    std::cout << "OK" << std::endl;
}

void KeyPointRegistrationWrapper::findCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source, pcl::PointCloud<pcl::FPFHSignature33>::Ptr target, std::vector<int>& correspondences)
{
    std::cout << "correspondence assignment..." << std::flush;
    correspondences.resize(source->size());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::KdTreeFLANN<pcl::FPFHSignature33> descriptor_kdtree;
    descriptor_kdtree.setInputCloud(target);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices(k);
    std::vector<float> k_squared_distances(k);
    for (int i = 0; i < static_cast<int> (source->size()); ++i)
    {
        descriptor_kdtree.nearestKSearch(*source, i, k, k_indices, k_squared_distances);
        correspondences[i] = k_indices[0];
    }
    std::cout << "OK" << std::endl;
}

void KeyPointRegistrationWrapper::filterCorrespondences(std::vector<int>& source2target_, std::vector<int>& target2source_, pcl::CorrespondencesPtr correspondences_, pcl::PointCloud<pcl::PointXYZI>::Ptr keyPointsSource, pcl::PointCloud<pcl::PointXYZI>::Ptr keyPointsTarget)
{
    std::cout << "correspondence rejection..." << std::flush;
    std::vector<std::pair<unsigned, unsigned> > correspondences;
    for (unsigned cIdx = 0; cIdx < source2target_.size(); ++cIdx)
        if (target2source_[source2target_[cIdx]] == static_cast<int> (cIdx))
            correspondences.push_back(std::make_pair(cIdx, source2target_[cIdx]));

    correspondences_->resize(correspondences.size());
    for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx)
    {
        (*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
        (*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
    }

    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector;
    rejector.setInputSource(keyPointsSource);
    rejector.setInputTarget(keyPointsTarget);
    rejector.setInputCorrespondences(correspondences_);
    rejector.getCorrespondences(*correspondences_);
    std::cout << "OK" << std::endl;
}

/*
double KeyPointRegistrationWrapper::computeCloudResolution(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
{
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> sqr_distances(2);
    pcl::search::KdTree<pcl::PointXYZI> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (!pcl_isfinite((*cloud)[i].x))
        {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            res += sqrt(sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0)
    {
        res /= n_points;
    }
    return res;
}
*/
