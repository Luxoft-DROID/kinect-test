#ifndef APPLICATION_H
#define APPLICATION_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>

#include "KinectDevice.h"
#include "PCLDetector.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

class Application
{
  public:
    Application(KinectDevice *device);
    void runApplication();

  private:
    void getKinectData();

    PointCloud::Ptr _cloud;
    pcl::visualization::PCLVisualizer _viewer;
    KinectDevice *_device;
    PCLDetector _detector;
};

#endif //APPLICATION_H
