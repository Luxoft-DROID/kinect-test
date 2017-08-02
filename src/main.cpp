#include "kinectdevice.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <algorithm>
#include "PCLDetector.h"

void run(KinectDevice *device)
{
	PointCloud::Ptr msg(new PointCloud);
	msg->header.frame_id = "some_tf_frame";
	msg->height = msg->width = 1;
	msg->is_dense = false;
	pcl::visualization::PCLVisualizer viewer("kinect depth image");

	PCLDetector detector(msg);
	viewer.addCoordinateSystem(1, "cloud", 0);
	viewer.addPointCloud(detector.getObsticles(), "obsticles");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "obsticles");

	while (!viewer.wasStopped())
	{
		static std::vector<uint16_t> depth(640 * 480);
		static std::vector<uint8_t> rgb(640 * 480 * 4);
		device->getRGB(rgb);
		device->getDepth(depth);

		for (int i = 0; i < depth.size(); i += 19)
		{
			if (depth[i] < 1000)
			{
				auto point = pcl::PointXYZRGB(rgb[3 * i], rgb[3 * i + 1], rgb[3 * i + 2]);
				point.x = i % 640 - 320;
				point.y = i / 640 - 240;
				point.z = depth[i];
				msg->points.push_back(point);
			}
		}

		detector.setPointCloud(msg);
		auto nearestPoint = detector.getNearestPointOnPlane();
		auto nearestObj = detector.getNearestObjectPonit();
		viewer.addLine(nearestPoint, detector.getFarestPointOnPlane(), 0, 255, 0, "Best tragectory");
		viewer.addLine(pcl::PointXYZ(0, 0, 0), nearestObj, 255, 0, 0, "nearestObj");
		viewer.addLine(nearestPoint, nearestObj, 255, 0, 0, "nearestObjplane");

		viewer.updatePointCloud(detector.getObsticles(), "obsticles");
		viewer.spinOnce();
		msg->clear();
		detector.getPlane()->clear();
		detector.getObsticles()->clear();

		viewer.removeShape("Best tragectory", 0);
		viewer.removeShape("nearestObj", 0);
		viewer.removeShape("nearestObjplane", 0);
	}
}

int main(int argc, char *argv[])
{
	Freenect::Freenect freenect;
	KinectDevice *device;
	double freenect_angle(0);
	freenect_video_format requested_format(FREENECT_VIDEO_RGB);
	//Get Kinect Device
	device = &freenect.createDevice<KinectDevice>(0);
	//Start Kinect Device
	device->setTiltDegrees(-16);
	device->startVideo();
	device->startDepth();
	//handle Kinect Device Data
	device->setLed(LED_GREEN);
	run(device);

	device->stopVideo();
	device->stopDepth();
	device->setLed(LED_OFF);

	return 0;
}