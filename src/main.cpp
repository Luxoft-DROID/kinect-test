#include "KinectDevice.h"
#include "Application.h"

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

	Application application(device);
	application.runApplication();

	device->stopVideo();
	device->stopDepth();
	device->setLed(LED_OFF);

	return 0;
}