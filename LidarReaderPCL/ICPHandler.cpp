#include "stdafx.h"

#include "ICPHandler.h"

#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>

#include <vector>
#include <thread>

void ICPHandler::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
    {
        m_nextIteration = true;
    }
    else if (event.getKeySym() == "a" && event.keyDown())
    {
        m_newCloud = true;
    }
}

ICPHandler::ICPHandler()
    : m_iterations(20)
      , m_nextIteration(false)
      , m_newCloud(false)
      , m_transformationMatrix(Eigen::Matrix4d::Identity())
      , m_cloud_in(new pcl::PointCloud<pcl::PointXYZ>)
      , m_cloud_tr(new pcl::PointCloud<pcl::PointXYZ>)
      , m_cloud_icp(new pcl::PointCloud<pcl::PointXYZ>)
      , m_viewer("Lidar ICP")
      , m_backgroundColor(0.0)
      , m_textColor(1.0 - m_backgroundColor)
      , m_cloudInColor(m_cloud_in, (int)255 * m_textColor, (int)255 * m_textColor, (int)255 * m_textColor)
      , m_cloudIcpColor(m_cloud_icp, 180, 20, 20)
      , m_downsampleLeafSize(0.04)

{

    std::vector<std::string> files;
      
    files.push_back("2018-09-13-16-09-39_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-09-45_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-09-51_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-09-57_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-10-05_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-10-12_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-10-19_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-10-26_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-10-33_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-10-40_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-10-46_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-10-51_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-10-57_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-11-03_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-11-08_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-11-15_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-11-23_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-11-29_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-11-36_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-11-42_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-11-48_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-11-54_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-13-16-12-02_Velodyne-VLP-16-Data.csv");

    std::string basePath("H:\\01_DROPBOX\\Dropbox\\0_MARCELSCHWITTLICK\\2018_JULIUSVONBISMARCK_HELM\\");

    for (const auto& file : files)
    {
        auto fullPath = basePath + file;
        std::vector<std::vector<double>> data = parse2DCsvFile(fullPath);
        pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);
        loadPointCloud(data, newCloud);
        m_queue.push(newCloud);
        std::cout << "Loaded cloud from " << fullPath << " with points: " << newCloud->size() << std::endl;
    }

    auto csvFile1 = "H:\\01_DROPBOX\\Dropbox\\0_MARCELSCHWITTLICK\\2018_JULIUSVONBISMARCK_HELM\\2018-09-13-16-09-26_Velodyne-VLP-16-Data.csv";
    std::vector<std::vector<double>> data = parse2DCsvFile(csvFile1);
    loadPointCloud(data, m_cloud_in);

    auto csvFile2 = "H:\\01_DROPBOX\\Dropbox\\0_MARCELSCHWITTLICK\\2018_JULIUSVONBISMARCK_HELM\\2018-09-13-16-09-32_Velodyne-VLP-16-Data.csv";
    std::vector<std::vector<double>> data2 = parse2DCsvFile(csvFile2);
    loadPointCloud(data2, m_cloud_icp);

    std::cout << "Cloud 1: " << " (" << m_cloud_in->size() << " points)" << std::endl;
    std::cout << "Cloud 2 " << " (" << m_cloud_icp->size() << " points)" << std::endl;

    downsample(m_downsampleLeafSize);

    m_icp.setMaximumIterations(m_iterations);
    m_icp.setInputSource(m_cloud_icp);
    m_icp.setInputTarget(m_cloud_in);

    m_icp.setTransformationEpsilon(0.00001);
    //icp.setMaxCorrespondenceDistance(0.05);
    //icp.setEuclideanFitnessEpsilon(1);
    //icp.setRANSACOutlierRejectionThreshold(1.5);

    //m_icp.align(*m_cloud_icp);
    m_icp.setMaximumIterations(m_iterations); // We set this variable to 1 for the next time we will call .align () function

    m_cloudInColor = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(m_cloud_in, (int)255 * m_textColor, (int)255 * m_textColor, (int)255 * m_textColor);
    m_cloudIcpColor = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(m_cloud_icp, 180, 20, 20);

    setupVisualisation();
}

ICPHandler::~ICPHandler()
{
}

void ICPHandler::setupNewCycle()
{
    if (m_queue.size() >= 1)
    {
        m_timer.tic();
        *m_cloud_in += *m_cloud_icp;

        m_cloud_icp = m_queue.waitAndPop();

        downsample(m_downsampleLeafSize);

        m_icp.setMaximumIterations(m_iterations);
        m_icp.setInputSource(m_cloud_icp);
        m_icp.setInputTarget(m_cloud_in);

        m_icp.setTransformationEpsilon(0.00001);
        //icp.setMaxCorrespondenceDistance(0.05);
        //icp.setEuclideanFitnessEpsilon(1);
        //icp.setRANSACOutlierRejectionThreshold(1.5);

        //m_icp.align(*m_cloud_icp);
        m_icp.setMaximumIterations(m_iterations); // We set this variable to 1 for the next time we will call .align () function

        m_cloudInColor = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(m_cloud_in, (int)255 * m_textColor, (int)255 * m_textColor, (int)255 * m_textColor);
        m_cloudIcpColor = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(m_cloud_icp, 180, 20, 20);

        std::cout << "Cloud 1: " << " (" << m_cloud_in->size() << " points)" << std::endl;
        std::cout << "Cloud 2 " << " (" << m_cloud_icp->size() << " points)" << std::endl;

        std::cout << "Loading new cloud took " << m_timer.toc() << " ms" << std::endl;

        m_timer.tic();
//#pragma omp parallel for num_threads(4)
        for(int i = 0; i < 5; i++ )
        {
//#pragma omp critical
            {
                m_icp.align(*m_cloud_icp);
                m_viewer.updatePointCloud(m_cloud_icp, m_cloudIcpColor, "cloud_icp_v2");
                m_viewer.spinOnce();
            }
        }
        m_viewer.updatePointCloud(m_cloud_in, m_cloudInColor, "cloud_in_v2");
        
        std::cout << "Aligning clouds took " << m_timer.toc() << " ms" << std::endl;
    }
}

void ICPHandler::downsample(double leafSize)
{
    m_timer.tic();
    std::cout << "--- Before downsample ---" << std::endl;
    std::cout << "Cloud 1: " << " (" << m_cloud_in->size() << " points)" << std::endl;
    std::cout << "Cloud 2 " << " (" << m_cloud_icp->size() << " points)" << std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(m_cloud_in);
    sor.setLeafSize(leafSize, leafSize, leafSize);
    sor.filter(*m_cloud_in);

    pcl::VoxelGrid<pcl::PointXYZ> sor2;
    sor2.setInputCloud(m_cloud_icp);
    sor2.setLeafSize(leafSize, leafSize, leafSize);
    sor2.filter(*m_cloud_icp);

    std::cout << "--- After downsample ---" << std::endl;
    std::cout << "Cloud 1: " << " (" << m_cloud_in->size() << " points)" << std::endl;
    std::cout << "Cloud 2 " << " (" << m_cloud_icp->size() << " points)" << std::endl;

    std::cout << "Downsampling took " << m_timer.toc() << " ms" << std::endl;
}

void ICPHandler::startVisualisation()
{
    while (!m_viewer.wasStopped())
    {
        m_viewer.spinOnce();

        if (m_newCloud)
        {
        
            setupNewCycle();
            
           
            //m_newCloud = false;
        }

        // The user pressed "space" :
        if (m_nextIteration)
        {
            // The Iterative Closest Point algorithm
            m_timer.tic();
            m_icp.align(*m_cloud_icp);
            m_iterations++;
            std::cout << "Applied 1 ICP iteration in " << m_timer.toc() << " ms" << std::endl;

            if (m_icp.hasConverged())
            {
                //std::cout << "ICP has converged, score is " << m_icp.getFitnessScore() << std::endl;
                
                m_transformationMatrix *= m_icp.getFinalTransformation().cast<double>(); // WARNING /!\ This is not accurate! For "educational" purpose only!
                //print4x4Matrix(m_transformationMatrix); // Print the transformation between original pose and current pose

                m_viewerString.str("");
                m_viewerString << m_iterations;
                std::string iterations_cnt = "ICP iterations = " + m_viewerString.str();
                m_viewer.updateText(iterations_cnt, 10, 60, 16, m_textColor, m_textColor, m_textColor, "iterations_cnt");
                m_viewer.updatePointCloud(m_cloud_icp, m_cloudIcpColor, "cloud_icp_v2");
            }
            else
            {
                PCL_ERROR("\nICP has not converged.\n");
            }
        }
        m_nextIteration = false;
    }
}

void ICPHandler::loadPointCloud(std::vector<std::vector<double>>& data, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const
{
    data.erase(data.begin());

    for (auto it = data.begin(); it != data.end(); ++it)
    {
        const auto& vec = *it;
        double x = vec.at(0);
        double y = vec.at(1);
        double z = vec.at(2);
        cloud->push_back(pcl::PointXYZ(x, y, z));
    }
}

std::vector<std::vector<double>> ICPHandler::parse2DCsvFile(std::string inputFileName) const
{
    std::vector<std::vector<double>> data;
    std::ifstream inputFile(inputFileName);
    int l = 0;

    while (inputFile)
    {
        l++;
        std::string s;
        if (!getline(inputFile, s)) break;
        if (s[0] != '#')
        {
            std::istringstream ss(s);
            std::vector<double> record;

            while (ss)
            {
                std::string line;
                if (!getline(ss, line, ','))
                    break;
                try
                {
                    record.push_back(stof(line));
                }
                catch (const std::invalid_argument e)
                {
                    //cout << "NaN found in file " << inputFileName << " line " << l << endl;
                    e.what();
                }
            }

            data.push_back(record);
        }
    }

    if (!inputFile.eof())
    {
        cerr << "Could not read file " << inputFileName << "\n";
    }

    return data;
}

void ICPHandler::print4x4Matrix(const Eigen::Matrix4d& matrix) const
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void ICPHandler::setupVisualisation()
{
    int v2(1);
    m_viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v2);

    // Original point cloud is white
    m_viewer.addPointCloud(m_cloud_in, m_cloudInColor, "cloud_in_v2", v2);

    // ICP aligned point cloud is red
    m_viewer.addPointCloud(m_cloud_icp, m_cloudIcpColor, "cloud_icp_v2", v2);

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
    m_viewer.registerKeyboardCallback(&ICPHandler::keyboardEventOccurred, *this);
}
