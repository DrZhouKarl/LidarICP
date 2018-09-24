#include "stdafx.h"

#include "PointCloudStitcher.h"

#include <pcl/impl/point_types.hpp>

#include <pcl/io/pcd_io.h>

#include <vector>
#include <thread>
#include <iostream>
#include <string>
#include <algorithm>
#include <iterator>



PointCloudStitcher::PointCloudStitcher()
    : m_grabber()
      , m_iterations(1)
      , m_nextIteration(false)
      , m_newCloud(true)
      , m_recordFrames(false)
      , m_useRecodings(false)
      , m_recorderCounter(0)
      , m_mergedCloudsCounter(0)
      , m_transformationMatrix(Eigen::Matrix4d::Identity())
      , m_viewer("Lidar ICP")
      , m_backgroundColor(0.0)
      , m_textColor(1.0 - m_backgroundColor)
    , m_finalCloudColor(m_threadedIcp.getFinalCloudPtr(), (int)255 * m_textColor, (int)255 * m_textColor, (int)255 * m_textColor)
    , m_newCloudColor(m_newestCloud, 180, 20, 20)
      //, m_cloudNewTargetColor(m_cloud_recent_transformed, (int)255 * m_textColor, (int)0 * m_textColor, (int)0 * m_textColor)
    , m_newestCloud(new pcl::PointCloud<pcl::PointXYZI>)
    , m_isFirstCloud(true)
      , tree(new pcl::search::KdTree<pcl::PointXYZI>)
{
    if (m_useRecodings)
    {
        std::string path = "H:\\01_DROPBOX\\Dropbox\\0_MARCELSCHWITTLICK\\2018_JULIUSVONBISMARCK_HELM\\recordings\\";

        std::vector<std::string> v;
        read_directory(path, v);
        std::copy(v.begin(), v.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
        for (const auto& file : v)
        {
            std::stringstream fullPath;
            fullPath << path << file;

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(fullPath.str(), *cloud) == -1) //* load the file
            {
                std::cout << "Couldn't read file " << fullPath.str() << std::endl;
            }
            std::cout << "Loaded cloud " << file << std::endl;

            m_queue.push(cloud);
        }

        while (true)
        {
            if (m_queue.size() > 3)
            {
                m_queue.waitAndPop(); // drop first faulty frame
                m_cloud_icp = m_queue.waitAndPop();
                pcl::copyPointCloud(*m_cloud_icp, *m_cloud_recent_transformed);
                m_cloud_in = m_queue.waitAndPop();
                std::cout << "Cloud 1: " << " (" << m_cloud_in->size() << " points)" << std::endl;
                std::cout << "Cloud 2 " << " (" << m_cloud_icp->size() << " points)" << std::endl;
                break;
            }
            std::cout << "sleeping,..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    else
    {
        boost::function<void(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&)> cloud_cb = boost::bind(&PointCloudStitcher::newCloudCallback, this, _1);
        std::cout << m_grabber.getName() << std::endl;
        m_cloudConnection = m_grabber.registerCallback(cloud_cb);

        m_grabber.start();
    }

    setupVisualisation();
}


void PointCloudStitcher::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
    {
        m_nextIteration = true;
    }
    else if (event.getKeySym() == "a" && event.keyDown())
    {
        m_newCloud = !m_newCloud;
    }
    else if (event.getKeySym() == "r" && event.keyDown())
    {
        m_recordFrames = !m_recordFrames;
        if (m_recordFrames == false)
        {
            for (const auto& c : m_cloudsToSave)
            {
                std::stringstream filename;
                filename << "H:\\01_DROPBOX\\Dropbox\\0_MARCELSCHWITTLICK\\2018_JULIUSVONBISMARCK_HELM\\recordings2\\vlp16_recording";
                filename << setfill('0') << setw(4) << m_recorderCounter;
                filename << ".pcd";
                pcl::io::savePCDFileASCII(filename.str(), c);
                m_recorderCounter += 1;
            }
        }
    } else if(event.getKeySym() == "c" && event.keyDown())
    {
        m_isFirstCloud = true;
        m_threadedIcp.clear();
    }
}

void PointCloudStitcher::newCloudCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& newCloud)
{
    if (m_recordFrames)
    {
        m_cloudsToSave.push_back(*newCloud);
    }

    //if (m_threadedIcp.toProcessQueueSize() < 6)
    {
        if(m_isFirstCloud)
        {
            m_isFirstCloud = false;
            //return;
        }
        auto c = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*newCloud);
        

        auto elapsed = applyTransformation(c, m_imu);
        {
            std::unique_lock<std::mutex> lock(m_newestCloudMutex);
            m_newestCloud = c;// boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*c);
            std::cout << "Transformation new cloud took: " << elapsed << "s" << std::endl;
            m_threadedIcp.add(c);
        }
    }
}

void PointCloudStitcher::read_directory(const std::string& name, std::vector<std::string>& v)
{
    boost::filesystem::path p(name);
    boost::filesystem::directory_iterator start(p);
    boost::filesystem::directory_iterator end;
    std::transform(start, end, std::back_inserter(v), path_leaf_string());
}


void PointCloudStitcher::applyNdtSystem()
{
    //downsample(m_downsampleLeafSize);
    applyMLSSmoothing();
    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon(0.01);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(0.2);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(2);

    // Setting max number of registration iterations.
    ndt.setMaximumIterations(35);

    // Setting point cloud to be aligned.
    ndt.setInputSource(m_cloud_icp);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget(m_cloud_in);

    // Set initial alignment estimate found using robot odometry.
    Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    // Calculating required rigid transform to align the input cloud to the target cloud.
    //pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    ndt.align(*m_cloud_icp);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
        << " score: " << ndt.getFitnessScore() << std::endl;

    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud(*m_cloud_icp, *m_cloud_icp, ndt.getFinalTransformation());
}

void PointCloudStitcher::startVisualisation()
{
    while (!m_viewer.wasStopped())
    {
        std::stringstream s1;
        s1 << "To Process: " << m_threadedIcp.toProcessQueueSize() << " clouds.";
        m_viewer.updateText(s1.str(), 10, 15, 16, m_textColor, m_textColor, m_textColor, "icp_info_2");

        std::stringstream s2;
        s2 << "Processed: " << m_threadedIcp.getProcessedCount() << " clouds.";
        m_viewer.updateText(s2.str(), 10, 40, 16, m_textColor, m_textColor, m_textColor, "iterations_cnt");
        /*
        if(m_threadedIcp.getFinishedCloudsQueue().size() > 0)
        {
            auto c = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(m_threadedIcp.getFinishedCloudsQueue().waitAndPop());
            if(m_finisedClouds.size() > 0)
            {
                IterativeClosestPointWrapper it;
                std::mutex m;
                it.apply(m_finisedClouds.at(m_finisedClouds.size() - 1), m, c);
            }
            m_finisedClouds.push_back(c);
            std::stringstream name;
            name << "cloud" << m_finisedClouds.size();
            auto color = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(m_finisedClouds.at(m_finisedClouds.size() - 1), 255, m_finisedClouds.size(), 255);
            
            
            m_viewer.addPointCloud(m_finisedClouds.at(m_finisedClouds.size() - 1), color, name.str(), 1);
        }
        */
        {
            std::unique_lock<std::mutex> lock(m_threadedIcp.getFinalCloudMutex());
            m_finalCloudColor = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(m_threadedIcp.getFinalCloudPtr(), 255, 255, 255);
        }
        {
            std::unique_lock<std::mutex> lock(m_newestCloudMutex);
            m_newCloudColor = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(m_newestCloud, 180, 20, 20);
            m_viewer.updatePointCloud(m_newestCloud, m_newCloudColor, "cloudNewest");
        }
        {
            std::unique_lock<std::mutex> lock(m_threadedIcp.getFinalCloudMutex());
            m_viewer.updatePointCloud(m_threadedIcp.getFinalCloudPtr(), m_finalCloudColor, "cloudFinal");
        }
        m_viewer.spinOnce();
    }

    m_grabber.stop();
    m_cloudConnection.disconnect();
}

void PointCloudStitcher::print4x4Matrix(const Eigen::Matrix4d& matrix) const
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void PointCloudStitcher::setupVisualisation()
{
    int v2(1);
    m_viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v2);

    // Original point cloud is white
    m_viewer.addPointCloud(m_newestCloud, m_newCloudColor, "cloudNewest", v2);

    // ICP aligned point cloud is red
    m_viewer.addPointCloud(m_threadedIcp.getFinalCloudPtr(), m_finalCloudColor, "cloudFinal", v2);
    //m_viewer.addPointCloud(m_cloud_recent_transformed, m_cloudNewTargetColor, "cloud_new_v2", v2);

    m_viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, m_textColor, m_textColor, m_textColor, "icp_info_2", v2);

    m_viewerString << m_iterations;
    std::string iterations_cnt = "ICP iterations = " + m_viewerString.str();
    m_viewer.addText(iterations_cnt, 10, 60, 16, m_textColor, m_textColor, m_textColor, "iterations_cnt", v2);

    // Set background color
    m_viewer.setBackgroundColor(m_backgroundColor, m_backgroundColor, m_backgroundColor, v2);

    // Set camera position and orientation
    m_viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    m_viewer.setSize(1920, 1000); // Visualiser window size

    // Register keyboard callback :
    m_viewer.registerKeyboardCallback(&PointCloudStitcher::keyboardEventOccurred, *this);
}

double PointCloudStitcher::applyTransformation(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud, IMUSensor& imu)
{
    m_timer.tic();
    auto att = imu.getAttitude();

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.0 , 0.0 , 0.0;
    transform.rotate(Eigen::AngleAxisf((-att.ang_x * M_PI) / 180, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf((-att.ang_y * M_PI) / 180, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf((-att.heading * M_PI) / 180, Eigen::Vector3f::UnitZ()));

    /*
    auto header = pointCloud->header;

    //std::chrono::microseconds s(header.stamp);
    //std::chrono::microseconds s2(_byteswap_uint64(header.stamp));

    uint32_t timestamp1 = 0;
    uint32_t timestamp2 = 0;

    auto stamp = m_imu.getStamp();
    timestamp2 = (uint32_t)header.stamp;
    timestamp1 = header.stamp >> 32;

    std::chrono::milliseconds imuStampMillis = std::chrono::duration_cast<std::chrono::milliseconds>(stamp);

    //std::cout << "Lidar Stamp: " << header.stamp << std::endl;
    //std::cout << "Lidar Stamp: " << timestamp1 << std::endl;
    //std::cout << "Lidar Stamp: " << timestamp2 << std::endl;
    //std::cout << "IMU Stamp:   " << stamp.count() << std::endl;
    //std::cout << "IMU StampMs: " << imuStampMillis.count() << std::endl;

    */

    pcl::transformPointCloud(*pointCloud, *pointCloud, transform); //Only rotate target cloud

    return m_timer.toc();
}

void PointCloudStitcher::applyStatisticalOutliner()
{
    /*
    m_sor.setInputCloud(m_cloud_in);
    m_sor.setMeanK(50);
    m_sor.setStddevMulThresh(1.0);
    m_sor.filter(*m_cloud_in);
    */

    m_sor.setInputCloud(m_cloud_icp);
    m_sor.setMeanK(50);
    m_sor.setStddevMulThresh(1.0);
    m_sor.filter(*m_cloud_icp);
}

void PointCloudStitcher::applyMLSSmoothing()
{
    {
        mls.setComputeNormals(true);

        // Set parameters
        mls.setInputCloud(m_cloud_icp);
        mls.setPolynomialOrder(2);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(0.2);

        // Reconstruct
        mls.process(mls_points);

        pcl::copyPointCloud(mls_points, *m_cloud_icp);
    }

    {
        mls.setComputeNormals(true);

        // Set parameters
        mls.setInputCloud(m_cloud_in);
        mls.setPolynomialOrder(2);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(0.2);

        // Reconstruct
        mls.process(mls_points);

        pcl::copyPointCloud(mls_points, *m_cloud_in);
    }
}
