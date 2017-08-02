#include "PCLDetector.h"

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
    _plane = PointCloud::Ptr(new PointCloud);
    _obsticles = PointCloud::Ptr(new PointCloud);
    extractPlain();
}

mPoints PCLDetector::getMeaningfulPoints()
{
    mPoints points;
    points.nearestOnPlane = getNearestPointOnPlane();
    points.farestOnPlane = getFarestPointOnPlane();
    points.nearestObj = getNearestObjectPonit();

    return points;
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
    pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliersPlane(new pcl::PointIndices);

    plane->values.resize(4);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setDistanceThreshold(7.0f);

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
    static const auto threshHold = 5;

    std::sort(_obsticles->begin(), _obsticles->end(),
              [](pcl::PointXYZRGB &a, pcl::PointXYZRGB &b) {
                  return a.x * a.x + a.y * a.y + a.z * a.z > b.x * b.x + b.y * b.y + b.z * b.z;
              });

    pcl::PointXYZ nearestObj;
    for (int i = 0; i < _obsticles->size() - 1; i++)
    {
        if (_obsticles->at(i).y - _obsticles->at(i + 1).y < threshHold)
        {
            nearestObj.x = _obsticles->at(i).x;
            nearestObj.y = _obsticles->at(i).y;
            nearestObj.z = _obsticles->at(i).z;
        }
    }

    return nearestObj;
}