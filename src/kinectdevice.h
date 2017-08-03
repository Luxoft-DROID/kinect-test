#ifndef KINECTDEVICE_H
#define KINECTDEVICE_H

#include <libfreenect.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <mutex>

#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

class KinectDevice final : public Freenect::FreenectDevice
{
  public:
    KinectDevice(freenect_context *_ctx, int _index);
    void VideoCallback(void *_rgb, uint32_t) override;
    void DepthCallback(void *_depth, uint32_t timestamp) override;
    std::vector<uint8_t> createDepthImg();
    bool getRGB(std::vector<uint8_t> &buffer);
    bool getDepth(std::vector<uint16_t> &buffer);

    static const int FREENECT_FRAME_W;
    static const int FREENECT_FRAME_H;
    static const int FREENECT_FRAME_PIX;
    static const int FREENECT_VIDEO_RGB_SIZE;
    static const int COLOR_MAX_VALUE;
  private:
    std::vector<uint16_t> m_buffer_depth;
    std::vector<uint8_t> m_buffer_video;
    std::mutex m_rgb_mutex;
    std::mutex m_depth_mutex;
    bool m_new_rgb_frame;
    bool m_new_depth_frame;
};

#endif // KINECTDEVICE_H
