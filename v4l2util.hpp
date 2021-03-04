/* v4l2util.h */

#pragma once

#include <vector>

#include <linux/videodev2.h>
#include <libv4l2.h>

namespace V4L2Util
{
  std::vector<v4l2_fmtdesc>     getFormats(int fd);
  v4l2_fract                    getInterval(int fd);
  std::vector<v4l2_frmsizeenum> getSizes(int fd, const v4l2_fmtdesc&);
  std::vector<v4l2_frmivalenum> getIntervals(int fd, const v4l2_fmtdesc&, const v4l2_frmsizeenum&);

  bool canCaptureReadWrite(int fd);
  bool canCaptureUserPtr(int fd);
  bool canCaptureMMap(int fd);
  
  void ioctl_throw(int fd, unsigned long int request, void* arg);
  int  ioctl_nothrow(int fd, unsigned long int request, void* arg);
  void ioctl_raise(unsigned long int request, int err);
  
} // namespace
