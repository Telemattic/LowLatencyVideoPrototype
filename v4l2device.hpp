/* v4l2device.hpp */

#pragma once

#include <memory>
#include <string>

#include "v4l2util.hpp"

class VideoCapture
{
public:
  using Buffer = std::pair<char*, size_t>;

  enum IO { READ, USERPTR, MMAP };

  explicit VideoCapture(const std::string& device, const IO aIO = MMAP);
  ~VideoCapture();

  IO SelectedIO() const { return mIO; }
  std::vector< IO > SupportedIO() const { return mSupported; }
  
  void StartCapture();
  void StopCapture();

  Buffer LockFrame();
  void UnlockFrame();

  v4l2_pix_format GetFormat() 
  {
    return V4L2Util::getFormat(mFd);
  }

  bool SetFormat(const v4l2_pix_format& aFmt)
  {
    return V4L2Util::setFormat(mFd, aFmt);
  }

  v4l2_fract GetInterval()
  {
    return V4L2Util::getInterval(mFd);
  }

  std::vector<v4l2_fmtdesc> GetFormats()
  {
    return V4L2Util::getFormats(mFd);
  }

  std::vector<v4l2_frmsizeenum> GetSizes(const v4l2_fmtdesc& format)
  {
    return V4L2Util::getSizes(mFd, format);
  }

  std::vector<v4l2_frmivalenum> GetIntervals(const v4l2_fmtdesc& format, const v4l2_frmsizeenum& size)
  {
    return V4L2Util::getIntervals(mFd, format, size);
  }

private:
  std::vector<IO> IOMethods();
  
  int mFd;
  IO mIO;
  std::vector< IO > mSupported;
  bool mCapturing;

  bool mIsLocked;

  class ICapture;
  class ReadWriteCapture;
  class StreamingCapture;
  
  std::unique_ptr<ICapture> m_capture;
};
