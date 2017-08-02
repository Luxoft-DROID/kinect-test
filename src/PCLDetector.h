#ifndef PCLDETECTOR_H
#define PCLDETECTOR_H

#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/sac_segmentation.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

typedef struct meaningfulPoints
{
    pcl::PointXYZ nearestOnPlane;
    pcl::PointXYZ farestOnPlane;
    pcl::PointXYZ nearestObj;
} mPoints;

class PCLDetector
{
public:
    PCLDetector(){}
    PCLDetector(PointCloud::Ptr cloud);
    mPoints getMeaningfulPoints();
    PointCloud::Ptr getPlane();
    PointCloud::Ptr getObsticles();
    PointCloud::Ptr getCloud();
    void setPointCloud(PointCloud::Ptr cloud);

    pcl::PointXYZ getNearestPointOnPlane();
    pcl::PointXYZ getFarestPointOnPlane();
    pcl::PointXYZ getNearestObjectPonit();
private:
    void extractPlain();

    PointCloud::Ptr _cloud;
    PointCloud::Ptr _plane;
  	PointCloud::Ptr _obsticles;
};

#endif