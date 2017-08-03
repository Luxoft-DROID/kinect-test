#include "kinectdevice.h"

#include <numeric>

const int KinectDevice::FREENECT_FRAME_W = int(640);
const int KinectDevice::FREENECT_FRAME_H = int(480);
const int KinectDevice::FREENECT_FRAME_PIX = int(FREENECT_FRAME_W * FREENECT_FRAME_H);
const int KinectDevice::FREENECT_VIDEO_RGB_SIZE = int(FREENECT_FRAME_PIX * 3);
const int KinectDevice::COLOR_MAX_VALUE = int(255);

KinectDevice::KinectDevice(freenect_context *_ctx, int _index)
    : Freenect::FreenectDevice(_ctx, _index),
      m_buffer_depth(FREENECT_FRAME_PIX),
      m_buffer_video(FREENECT_VIDEO_RGB_SIZE),
      m_new_rgb_frame(false),
      m_new_depth_frame(false)
{
}
void KinectDevice::VideoCallback(void *_rgb, uint32_t)
{
    m_rgb_mutex.lock();
    uint8_t *rgb = static_cast<uint8_t *>(_rgb);
    copy(rgb, rgb + getVideoBufferSize(), m_buffer_video.begin());
    m_new_rgb_frame = true;
    m_rgb_mutex.unlock();
}

void KinectDevice::DepthCallback(void *_depth, uint32_t)
{
    m_depth_mutex.lock();
    uint16_t *depth = static_cast<uint16_t *>(_depth);
    for (unsigned int i = 0; i < FREENECT_FRAME_PIX; i++)
    {
        m_buffer_depth[i] = depth[i];
    }
    m_new_depth_frame = true;
    m_depth_mutex.unlock();
}

bool KinectDevice::getRGB(std::vector<uint8_t> &buffer)
{
    m_rgb_mutex.lock();
    if (m_new_rgb_frame)
    {
        buffer.swap(m_buffer_video);
        m_new_rgb_frame = false;
        m_rgb_mutex.unlock();
        return true;
    }
    else
    {
        m_rgb_mutex.unlock();
        return false;
    }
}

bool KinectDevice::getDepth(std::vector<uint16_t> &buffer)
{
    m_depth_mutex.lock();
    if (m_new_depth_frame)
    {
        buffer.swap(m_buffer_depth);
        m_new_depth_frame = false;
        m_depth_mutex.unlock();
        return true;
    }
    else
    {
        m_depth_mutex.unlock();
        return false;
    }
}
std::vector<uint8_t> KinectDevice::createDepthImg()
{
    std::vector<uint16_t> depth(FREENECT_FRAME_PIX);
    getDepth(depth);
    std::vector<uint8_t> depthImg(FREENECT_VIDEO_RGB_SIZE);
    for (unsigned int i = 0; i < depth.size(); i++)
    {
        int pval = depth[i];
        int lb = pval & 0xff;
        switch (pval >> 8)
        {
        case 0:
            depthImg[3 * i + 0] = COLOR_MAX_VALUE;
            depthImg[3 * i + 1] = COLOR_MAX_VALUE - lb;
            depthImg[3 * i + 2] = COLOR_MAX_VALUE - lb;
            break;
        case 1:
            depthImg[3 * i + 0] = COLOR_MAX_VALUE;
            depthImg[3 * i + 1] = lb;
            depthImg[3 * i + 2] = 0;
            break;
        case 2:
            depthImg[3 * i + 0] = COLOR_MAX_VALUE - lb;
            depthImg[3 * i + 1] = COLOR_MAX_VALUE;
            depthImg[3 * i + 2] = 0;
            break;
        case 3:
            depthImg[3 * i + 0] = 0;
            depthImg[3 * i + 1] = COLOR_MAX_VALUE;
            depthImg[3 * i + 2] = lb;
            break;
        case 4:
            depthImg[3 * i + 0] = 0;
            depthImg[3 * i + 1] = COLOR_MAX_VALUE - lb;
            depthImg[3 * i + 2] = COLOR_MAX_VALUE;
            break;
        case 5:
            depthImg[3 * i + 0] = 0;
            depthImg[3 * i + 1] = 0;
            depthImg[3 * i + 2] = COLOR_MAX_VALUE - lb;
            break;
        default:
            depthImg[3 * i + 0] = 0;
            depthImg[3 * i + 1] = 0;
            depthImg[3 * i + 2] = 0;
            break;
        }
    }
    return depthImg;
}
