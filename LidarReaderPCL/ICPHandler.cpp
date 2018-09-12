#include "stdafx.h"

#include "ICPHandler.h"
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
#include <vector>

void ICPHandler::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
    {
        m_nextIteration = true;
    }
    else if (event.getKeySym() == "a" && event.keyDown())
    {
    }
}

ICPHandler::ICPHandler()
    : m_iterations(1)
      , m_nextIteration(false)
      , m_transformationMatrix(Eigen::Matrix4d::Identity())
      , m_cloud_in(new pcl::PointCloud<pcl::PointXYZ>)
      , m_cloud_tr(new pcl::PointCloud<pcl::PointXYZ>)
      , m_cloud_icp(new pcl::PointCloud<pcl::PointXYZ>)
{
    std::vector<std::string> files;
    files.push_back("2018-09-12-19-21-30_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-12-19-21-41_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-12-19-21-50_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-12-19-22-04_Velodyne-VLP-16-Data.csv");
    files.push_back("2018-09-12-19-22-12_Velodyne-VLP-16-Data.csv");

    std::string basePath("H:\\01_DROPBOX\\Dropbox\\0_MARCELSCHWITTLICK\\2018_JULIUSVONBISMARCK_HELM\\");

    auto csvFile1 = "H:\\01_DROPBOX\\Dropbox\\0_MARCELSCHWITTLICK\\2018_JULIUSVONBISMARCK_HELM\\2018-09-12-19-21-30_Velodyne-VLP-16-Data.csv";
    std::vector<std::vector<double>> data = parse2DCsvFile(csvFile1);
    loadPointCloud(data, m_cloud_in);

    auto csvFile2 = "H:\\01_DROPBOX\\Dropbox\\0_MARCELSCHWITTLICK\\2018_JULIUSVONBISMARCK_HELM\\2018-09-12-19-21-41_Velodyne-VLP-16-Data.csv";
    std::vector<std::vector<double>> data2 = parse2DCsvFile(csvFile2);
    loadPointCloud(data2, m_cloud_icp);

    std::cout << "Cloud 1: " << " (" << m_cloud_in->size() << " points)" << std::endl;
    std::cout << "Cloud 2 " << " (" << m_cloud_icp->size() << " points)" << std::endl;
}

ICPHandler::~ICPHandler()
{
}

void ICPHandler::setupICP()
{
    m_icp.setMaximumIterations(m_iterations);
    m_icp.setInputSource(m_cloud_icp);
    m_icp.setInputTarget(m_cloud_in);

    m_icp.setTransformationEpsilon(0.00001);
    //icp.setMaxCorrespondenceDistance(0.05);
    //icp.setEuclideanFitnessEpsilon(1);
    //icp.setRANSACOutlierRejectionThreshold(1.5);

    //m_icp.align(*m_cloud_icp);
    m_icp.setMaximumIterations(m_iterations); // We set this variable to 1 for the next time we will call .align () function
}

void ICPHandler::downsample(double leafSize)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(m_cloud_in);
    sor.setLeafSize(leafSize, leafSize, leafSize);
    sor.filter(*m_cloud_in);

    pcl::VoxelGrid<pcl::PointXYZ> sor2;
    sor2.setInputCloud(m_cloud_icp);
    sor2.setLeafSize(leafSize, leafSize, leafSize);
    sor2.filter(*m_cloud_icp);

    std::cout << "Cloud 1: " << " (" << m_cloud_in->size() << " points)" << std::endl;
    std::cout << "Cloud 2 " << " (" << m_cloud_icp->size() << " points)" << std::endl;
}

void ICPHandler::startVisualisation()
{
    pcl::visualization::PCLVisualizer viewer("ICP demo");
    // Create two vertically separated viewports
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    // The color we will be using
    float bckgr_gray_level = 0.0; // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(m_cloud_in, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
                                                                                     (int)255 * txt_gray_lvl);
    viewer.addPointCloud(m_cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud(m_cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h(m_cloud_tr, 20, 180, 20);
    viewer.addPointCloud(m_cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_icp_color_h(m_cloud_icp, 180, 20, 20);
    viewer.addPointCloud(m_cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

    // Adding text descriptions in each viewport
    viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    std::stringstream ss;
    ss << m_iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str();
    viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // Set background color
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // Set camera position and orientation
    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize(1280, 1024); // Visualiser window size

    // Register keyboard callback :

    viewer.registerKeyboardCallback(&ICPHandler::keyboardEventOccurred, *this);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();

        // The user pressed "space" :
        if (ICPHandler::m_nextIteration)
        {
            // The Iterative Closest Point algorithm
            m_timer.tic();
            m_icp.align(*m_cloud_icp);
            std::cout << "Applied 1 ICP iteration in " << m_timer.toc() << " ms" << std::endl;

            if (m_icp.hasConverged())
            {
                //printf("\033[11A"); // Go up 11 lines in terminal output.
                std::cout << "ICP has converged, score is " << m_icp.getFitnessScore() << std::endl;
                //printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
                std::cout << "\nICP transformation " << ++m_iterations << " : cloud_icp -> cloud_in" << std::endl;
                m_transformationMatrix *= m_icp.getFinalTransformation().cast<double>(); // WARNING /!\ This is not accurate! For "educational" purpose only!
                print4x4Matrix(m_transformationMatrix); // Print the transformation between original pose and current pose

                ss.str("");
                ss << m_iterations;
                std::string iterations_cnt = "ICP iterations = " + ss.str();
                viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
                viewer.updatePointCloud(m_cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
            }
            else
            {
                PCL_ERROR("\nICP has not converged.\n");
            }
        }
        ICPHandler::m_nextIteration = false;
    }
}

void ICPHandler::loadPointCloud(std::vector<std::vector<double>>& data, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const
{
    data.erase(data.begin());

    for (auto it = data.begin(); it != data.end(); ++it)
    {
        const auto& vec = *it;
        double x = vec.at(0) + 2.0;
        double y = vec.at(1) + 2.0;
        double z = vec.at(2) + 2.0;
        cloud->push_back(pcl::PointXYZ(x / 10.0, y / 10.0, z / 10.0));
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
