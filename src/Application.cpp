#include "Application.h"

#include <string>

Application::Application(KinectDevice *device)
    : _viewer(), _cloud(new PointCloud), _device(device), _detector(_cloud)
{
    _cloud->header.frame_id = "some_tf_frame";
    _cloud->height = _cloud->width = 1;
    _cloud->is_dense = true;
}

void Application::runApplication()
{
    _viewer.addCoordinateSystem(1, "cloud", 0);
    _viewer.addPointCloud(_detector.getObsticles(), "obsticles");
    _viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "obsticles");

    while (!_viewer.wasStopped())
    {
        getKinectData();

        _detector.setPointCloud(_cloud);
        auto cubeArr = _detector.getObjectsRectangles();
        for (auto &cube : cubeArr)
        {
            static int i = 0;
            _viewer.addCube(cube.xMin, cube.xMax, cube.yMin, cube.yMax, cube.zMin, cube.zMax,
                            1.0, 0.0, 0.0, std::to_string(i), 0);
            i++;
        }

        _viewer.updatePointCloud(_detector.getObsticles(), "obsticles");
        _viewer.spinOnce();
        _cloud->clear();

        _viewer.removeAllShapes(0);
    }
}

void Application::getKinectData()
{
    static const int depthTreshold = 1000;
    static const int spaceBetweenPts = 57;
    static std::vector<uint16_t> depth(KinectDevice::FREENECT_FRAME_PIX);
    static std::vector<uint8_t> rgb(KinectDevice::FREENECT_VIDEO_RGB_SIZE);

    _device->getRGB(rgb);
    _device->getDepth(depth);

    for (int i = 0; i < depth.size(); i += spaceBetweenPts)
    {
        if (depth[i] < depthTreshold)
        {
            auto point = pcl::PointXYZRGB(rgb[3 * i], rgb[3 * i + 1], rgb[3 * i + 2]);
            point.x = i % KinectDevice::FREENECT_FRAME_W - KinectDevice::FREENECT_FRAME_W / 2;
            point.y = i / KinectDevice::FREENECT_FRAME_W - KinectDevice::FREENECT_FRAME_H / 2;
            point.z = depth[i];
            _cloud->points.push_back(point);
        }
    }
}
