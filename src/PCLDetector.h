#ifndef PCLDETECTOR_H
#define PCLDETECTOR_H

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

typedef struct Cube
{
    float xMin;
    float xMax;
    float yMin;
    float yMax;
    float zMin;
    float zMax;
}Cube;

class PCLDetector
{
  public:
    PCLDetector() {}
    PCLDetector(PointCloud::Ptr cloud);
    PointCloud::Ptr getPlane();
    PointCloud::Ptr getObsticles();
    PointCloud::Ptr getCloud();
    void setPointCloud(PointCloud::Ptr cloud);
    std::vector<Cube> getObjectsRectangles();

    pcl::PointXYZ getNearestPointOnPlane();
    pcl::PointXYZ getFarestPointOnPlane();
    pcl::PointXYZ getNearestObjectPonit();

  private:
    void extractPlain();
    Cube getCube(PointCloud::Ptr cloud);

    PointCloud::Ptr _cloud;
    PointCloud::Ptr _plane;
    PointCloud::Ptr _obsticles;
};

#endif