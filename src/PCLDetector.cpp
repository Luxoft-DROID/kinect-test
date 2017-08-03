#include "PCLDetector.h"

#include "pcl/kdtree/kdtree.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/features/normal_3d.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/point_types.h"

#include <algorithm>

PCLDetector::PCLDetector(PointCloud::Ptr cloud)
    : _cloud(cloud)
{
    _plane = PointCloud::Ptr(new PointCloud);
    _obsticles = PointCloud::Ptr(new PointCloud);
    extractPlain();
}

void PCLDetector::setPointCloud(PointCloud::Ptr cloud)
{
    _cloud = cloud;
    _plane->clear();
    _obsticles->clear();
    extractPlain();
}

PointCloud::Ptr PCLDetector::getCloud()
{
    return _cloud;
}

PointCloud::Ptr PCLDetector::getPlane()
{
    return _plane;
}

PointCloud::Ptr PCLDetector::getObsticles()
{
    return _obsticles;
}

void PCLDetector::extractPlain()
{
    if(_cloud->size() == 0) return;

    static pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
    static pcl::PointIndices::Ptr inliersPlane(new pcl::PointIndices);

    plane->values.resize(4);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setDistanceThreshold(7.0f);
    seg.setMaxIterations (50);

    seg.setInputCloud(_cloud);
    seg.segment(*inliersPlane, *plane);
    static pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    extract.setInputCloud(_cloud);
    extract.setIndices(inliersPlane);
    extract.setNegative(false);
    extract.filter(*_plane);

    extract.setNegative(true);
    extract.filter(*_obsticles);
}

pcl::PointXYZ PCLDetector::getNearestPointOnPlane()
{
    auto nearestPoint = pcl::PointXYZ(0, -240, 2048);
    for (int i = _plane->size() - 1; i >= 0; i--)
    {
        if (_plane->at(i).y > nearestPoint.y)
        {
            nearestPoint.y = _plane->at(i).y;
            nearestPoint.z = _plane->at(i).z;
        }

        if (nearestPoint.y > _plane->at(i).y + 10) //optimization
        {
            break;
        }
    }

    return nearestPoint;
}

pcl::PointXYZ PCLDetector::getFarestPointOnPlane()
{
    auto farestPoint = pcl::PointXYZ(0, 240, 0);
    for (int i = 0; i < _plane->size(); i++)
    {
        if (_plane->at(i).y < farestPoint.y)
        {
            farestPoint.x = _plane->at(i).x;
            farestPoint.y = _plane->at(i).y;
            farestPoint.z = _plane->at(i).z;
        }

        if (farestPoint.y < _plane->at(i).y - 10) //optimization
        {
            break;
        }
    }

    return farestPoint;
}

pcl::PointXYZ PCLDetector::getNearestObjectPonit()
{
    static const auto threshHold = 10;

    std::sort(_obsticles->begin(), _obsticles->end(),
              [](pcl::PointXYZRGB &a, pcl::PointXYZRGB &b) {
                  return a.x * a.x + a.y * a.y + a.z * a.z > b.x * b.x + b.y * b.y + b.z * b.z;
              });

    pcl::PointXYZ nearestObj;
    for (int i = 0; i < _obsticles->size() - 1; i++)
    {
        if (_obsticles->at(i).y - _obsticles->at(i + 1).y < threshHold &&
            _obsticles->at(i).z - _obsticles->at(i + 1).z < threshHold)
        {
            nearestObj.x = _obsticles->at(i).x;
            nearestObj.y = _obsticles->at(i).y;
            nearestObj.z = _obsticles->at(i).z;
        }
    }

    return nearestObj;
}

std::vector<Cube> PCLDetector::getObjectsRectangles()
{
    static const double tolerance(35.0);
    static const int minClusterSize(25);
    static const int maxClusterSize(2000);

        std::vector<Cube> retArr;
    if (_obsticles->size() == 0)
        return retArr;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(_obsticles);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(minClusterSize);
    ec.setMaxClusterSize(maxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(_obsticles);
    ec.extract(clusterIndices);
    
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
    {
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloudCluster->points.push_back(_obsticles->points[*pit]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        retArr.push_back(getCube(cloudCluster));
        cloudCluster->clear();
    }

    return retArr;
}

Cube PCLDetector::getCube(PointCloud::Ptr cloud)
{
    Cube retCube{320, -320, 240, -240, 2048, 0};
    for (auto &p : *cloud)
    {
        if (p.x < retCube.xMin)
            retCube.xMin = p.x;
        if (p.x > retCube.xMax)
            retCube.xMax = p.x;
        if (p.y < retCube.yMin)
            retCube.yMin = p.y;
        if (p.y > retCube.yMax)
            retCube.yMax = p.y;
        if (p.z < retCube.zMin)
            retCube.zMin = p.z;
        if (p.z > retCube.zMax)
            retCube.zMax = p.z;
    }
    return retCube;
}