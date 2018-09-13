// LidarReaderPCL.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <string>

#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include "VLPViewerSimple.h"
#include "ICPHandler.h"
#include <random>

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;


void runSimpleViewer()
{
    VLPGrabber grabber;

    PointCloudColorHandlerGenericField<PointXYZI>* color_handler = new PointCloudColorHandlerGenericField<PointXYZI>("intensity");

    SimpleVLPViewer<PointXYZI> v(grabber, color_handler);
    v.run();
}


void runICPSimple()
{
    ICPHandler h;
    //h.downsample(0.001);
    //h.setupICP();
    h.startVisualisation();
}

int main(int argc,
         char* argv[])
{
    //runSimpleViewer();
    runICPSimple();

    return (0);
}
