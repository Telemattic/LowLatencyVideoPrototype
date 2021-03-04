/* v4l2util.cpp */

#include "v4l2util.hpp"

#include <cstring>
#include <stdexcept>
#include <errno.h>

std::vector<v4l2_fmtdesc>
V4L2Util::getFormats(int fd)
{
  std::vector<v4l2_fmtdesc> fmts;

  for (int index = 0; ; ++index) {

    v4l2_fmtdesc fmt;
    fmt.index = index;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (auto ret = ioctl_nothrow(fd, VIDIOC_ENUM_FMT, &fmt)) {
      if (EINVAL == ret)
	break;
      throw std::runtime_error("v4l2_ioctl");
    }

    fmts.push_back(fmt);
  }

  return fmts;
}

v4l2_fract
V4L2Util::getInterval(int fd)
{
  v4l2_streamparm param;
  memset(&param, 0, sizeof(param));
  param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ioctl_throw(fd, VIDIOC_G_PARM, &param);
  return param.parm.capture.timeperframe;
}

std::vector<v4l2_frmsizeenum>
V4L2Util::getSizes(int fd, const v4l2_fmtdesc& format)
{
  std::vector<v4l2_frmsizeenum> sizes;

  for (int index = 0; ; ++index) {

    v4l2_frmsizeenum size;
    size.index = index;
    size.pixel_format = format.pixelformat;

    if (auto ret = ioctl_nothrow(fd, VIDIOC_ENUM_FRAMESIZES, &size)) {
      if (EINVAL == ret)
	break;
      throw std::runtime_error("v4l2_ioctl");
    }

    sizes.push_back(size);

    // only discrete has multiple enumerations
    if (size.type != V4L2_FRMSIZE_TYPE_DISCRETE)
      break;
  }

  return sizes;
}

std::vector<v4l2_frmivalenum>
V4L2Util::getIntervals(int fd, const v4l2_fmtdesc& format, const v4l2_frmsizeenum& size)
{
  std::vector<v4l2_frmivalenum> intervals;

  for (int index = 0; ; ++index) {

    v4l2_frmivalenum interval;
    interval.index = index;
    interval.pixel_format = format.pixelformat;
    interval.width = size.discrete.width;
    interval.height = size.discrete.height;

    if (auto ret = ioctl_nothrow(fd, VIDIOC_ENUM_FRAMEINTERVALS, &interval)) {
      if (EINVAL == ret)
	break;
      throw std::runtime_error("v4l2_ioctl");
    }

    intervals.push_back(interval);
  }

  return intervals;
}

bool
V4L2Util::canCaptureReadWrite(int fd)
{
  v4l2_capability cap;
  ioctl_throw(fd, VIDIOC_QUERYCAP, &cap);

  return (cap.capabilities & V4L2_CAP_READWRITE) == V4L2_CAP_READWRITE;
}

bool
V4L2Util::canCaptureUserPtr(int fd)
{
  v4l2_capability cap;
  ioctl_throw(fd, VIDIOC_QUERYCAP, &cap);

  if ((cap.capabilities & V4L2_CAP_STREAMING) != V4L2_CAP_STREAMING)
    return false;

  v4l2_requestbuffers req;

  // test userptr
  memset(&req, 0, sizeof req);
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.count = 1;
  req.memory = V4L2_MEMORY_USERPTR;
  
  if (0 != v4l2_ioctl(fd, VIDIOC_REQBUFS, &req))
    return false;

  req.count = 0;
  // blind ioctl, some drivers don't like count = 0
  v4l2_ioctl(fd, VIDIOC_REQBUFS, &req);

  return true;
}

bool
V4L2Util::canCaptureMMap(int fd)
{
  v4l2_capability cap;
  ioctl_throw(fd, VIDIOC_QUERYCAP, &cap);

  if ((cap.capabilities & V4L2_CAP_STREAMING) != V4L2_CAP_STREAMING)
    return false;

  v4l2_requestbuffers req;

  // test mmap
  memset(&req, 0, sizeof req);
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.count = 1;
  req.memory = V4L2_MEMORY_MMAP;
  if (0 != v4l2_ioctl(fd, VIDIOC_REQBUFS, &req))
    return false;
  
  req.count = 0;
  // blind ioctl, some drivers get don't like count = 0
  v4l2_ioctl(fd, VIDIOC_REQBUFS, &req);

  return true;
}
  
void
V4L2Util::ioctl_throw(int fd, unsigned long int request, void* arg)
{
  if (auto ret = ioctl_nothrow(fd, request, arg)) {
    throw std::runtime_error("v4l2_ioctl");
  }
}
  
int
V4L2Util::ioctl_nothrow(int fd, unsigned long int request, void* arg)
{
  int ret = 0;
  do {
    ret = v4l2_ioctl(fd, request, arg);
  }
  while (-1 == ret && (EINTR == errno || EAGAIN == errno));

  return errno;
}
