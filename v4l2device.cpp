/* v4l2device.cpp */

#include "v4l2device.hpp"

#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>

#include <linux/videodev2.h>
#include <libv4l2.h>

#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#define THROW( d ) \
	{ \
	std::ostringstream oss; \
    oss << "[" << __FILE__ << ":" << __LINE__ << "]"; \
	oss << " " << d; \
	throw std::runtime_error( oss.str() ); \
	} \

using Buffer = std::pair<char*, size_t>;

class BufferFactory {

public:
  virtual ~BufferFactory() {};
  virtual Buffer allocate(size_t length, int64_t offset) = 0;
};

class HeapBufferFactory : public BufferFactory {

public:
  HeapBufferFactory();
  ~HeapBufferFactory() override;
  
  Buffer allocate(size_t length, int64_t offset) override;

private:
  std::vector<Buffer> m_buffers;
};

HeapBufferFactory::HeapBufferFactory()
{
}

HeapBufferFactory::~HeapBufferFactory()
{
  for (const auto& i : m_buffers) {
    delete [] i.first;
  }
}

Buffer
HeapBufferFactory::allocate(size_t length, int64_t offset)
{
  auto p = new char[length];
  m_buffers.push_back(Buffer{p, length});
  return m_buffers.back();
}

class MMapBufferFactory : public BufferFactory {
public:
  explicit MMapBufferFactory(int fd);
  ~MMapBufferFactory() override;

  Buffer allocate(size_t length, int64_t offset) override;

private:
  const int            m_fd;
  std::vector<Buffer>  m_buffers;
};

MMapBufferFactory::MMapBufferFactory(int fd)
  : m_fd(fd)
{
}

MMapBufferFactory::~MMapBufferFactory()
{
  for (const auto& i : m_buffers) {
    v4l2_munmap(i.first, i.second);
  }
}

Buffer
MMapBufferFactory::allocate(size_t length, int64_t offset)
{
  auto p = v4l2_mmap(nullptr, length, PROT_READ|PROT_WRITE, MAP_SHARED, m_fd, offset);
  if (MAP_FAILED == p)
    throw std::runtime_error("v4l2_mmap");

  m_buffers.push_back(Buffer{reinterpret_cast<char*>(p), length});
  return m_buffers.back();
}
 
class VideoCapture::ICapture {
public:
  virtual ~ICapture() {}

  virtual void   startCapture(size_t numBuffers, size_t imageSize) = 0;
  virtual void   stopCapture() = 0;
  
  virtual Buffer lockFrame() = 0;
  virtual void   unlockFrame() = 0;

protected:
  std::vector<Buffer> m_buffers;
};

class VideoCapture::ReadWriteCapture : public VideoCapture::ICapture {

public:
  explicit ReadWriteCapture(int fd);
  ~ReadWriteCapture() override;

  void startCapture(size_t numBuffers, size_t sizeImage) override;
  void stopCapture() override;

  Buffer lockFrame() override;
  void   unlockFrame() override;

private:
  const int m_fd;
  std::unique_ptr<BufferFactory>  m_bufferFactory;
};

VideoCapture::ReadWriteCapture::ReadWriteCapture(int fd)
  : m_fd(fd)
  , m_bufferFactory(std::make_unique<HeapBufferFactory>())
{
}

VideoCapture::ReadWriteCapture::~ReadWriteCapture()
{
}

void
VideoCapture::ReadWriteCapture::startCapture(size_t numBuffers, size_t imageSize)
{
  m_buffers.reserve(1);
  m_buffers.push_back(m_bufferFactory->allocate(imageSize, 0));
}

void
VideoCapture::ReadWriteCapture::stopCapture()
{
}

Buffer
VideoCapture::ReadWriteCapture::lockFrame()
{
  auto bytesUsed = v4l2_read(m_fd, m_buffers[0].first, m_buffers[0].second);

  if (bytesUsed < 0) {

    if (errno != EAGAIN && errno != EIO)
      THROW( "read() error" );
  }

  return Buffer{m_buffers[0].first, static_cast<size_t>(bytesUsed)};
}

void
VideoCapture::ReadWriteCapture::unlockFrame()
{
  // nop
}

class VideoCapture::StreamingCapture : public VideoCapture::ICapture {

public:
  StreamingCapture(int fd, v4l2_memory memory);
  ~StreamingCapture() override;

  void startCapture(size_t numBuffers, size_t imageSize) override;
  void stopCapture() override;

  Buffer lockFrame() override;
  void unlockFrame() override;

private:
  const int         m_fd;
  const v4l2_memory m_memory;
  v4l2_buffer       m_lockedBuffer;
  std::unique_ptr<BufferFactory>  m_bufferFactory;
};

VideoCapture::StreamingCapture::StreamingCapture(int fd, v4l2_memory memory)
  : m_fd(fd)
  , m_memory(memory)
{
  if (V4L2_MEMORY_MMAP == memory)
    m_bufferFactory = std::make_unique<MMapBufferFactory>(m_fd);
  else
    m_bufferFactory = std::make_unique<HeapBufferFactory>();
}

VideoCapture::StreamingCapture::~StreamingCapture()
{
}

void
VideoCapture::StreamingCapture::startCapture(size_t numBuffers, size_t imageSize)
{
  // request buffers
  v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = m_memory;
  req.count = numBuffers;
  
  V4L2Util::ioctl_throw(m_fd, VIDIOC_REQBUFS, &req);

  std::cout << "allocating " << req.count << " buffers" << std::endl;

  m_buffers.resize(req.count);
    
  if (V4L2_MEMORY_USERPTR == m_memory) {

    for (auto& i : m_buffers) {
      i = m_bufferFactory->allocate(imageSize, 0);
    }
  }
  else {

    // mmap buffers
    for (size_t i = 0; i != m_buffers.size(); ++i) {

      v4l2_buffer buf;
      memset( &buf, 0, sizeof(buf) );
      buf.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory  = V4L2_MEMORY_MMAP;
      buf.index   = i;
      V4L2Util::ioctl_throw(m_fd, VIDIOC_QUERYBUF, &buf);

      m_buffers[i] = m_bufferFactory->allocate(buf.length, buf.m.offset);

      std::cout << "buffer[" << i << "]: addr=" << std::hex << (unsigned long)m_buffers[i].first
		<< std::dec << " length=" << buf.length
		<< " offset=" << buf.m.offset << std::endl;
    }
  }

  // queue buffers
  for (size_t i = 0; i != m_buffers.size(); ++i) {

    v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.index  = i;
    buf.memory = m_memory;
    
    if (V4L2_MEMORY_USERPTR == m_memory) {

      buf.m.userptr = (unsigned long)m_buffers[i].first;
      buf.length    = m_buffers[i].second;
    }

    V4L2Util::ioctl_throw(m_fd, VIDIOC_QBUF, &buf);
  }

  // start streaming
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  V4L2Util::ioctl_throw(m_fd, VIDIOC_STREAMON, &type);
}

void
VideoCapture::StreamingCapture::stopCapture()
{
  // stop streaming
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  V4L2Util::ioctl_throw(m_fd, VIDIOC_STREAMOFF, &type);
}

Buffer
VideoCapture::StreamingCapture::lockFrame()
{
  memset(&m_lockedBuffer, 0, sizeof(m_lockedBuffer));
  m_lockedBuffer.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  m_lockedBuffer.memory = m_memory;

  if (-1 == v4l2_ioctl(m_fd, VIDIOC_DQBUF, &m_lockedBuffer)) {

    if (errno != EAGAIN && errno != EIO)
      V4L2Util::ioctl_raise(VIDIOC_DQBUF, errno);
  }

  return Buffer{m_buffers[m_lockedBuffer.index].first, m_lockedBuffer.bytesused};
}

void
VideoCapture::StreamingCapture::unlockFrame()
{
  V4L2Util::ioctl_throw(m_fd, VIDIOC_QBUF, &m_lockedBuffer);
}

VideoCapture::VideoCapture(const std::string& device, const IO aIO)
{
  mFd = v4l2_open( device.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (mFd == -1)
    THROW( "can't open " << device << ": " << strerror(errno) );

  // make sure this is a capture device
  v4l2_capability cap;
  V4L2Util::ioctl_throw(mFd, VIDIOC_QUERYCAP, &cap) ;
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    THROW("not a video capture device!");

  mSupported = IOMethods();
  if (mSupported.empty()) THROW("no supported IO methods!");

  bool found_requested = false;
  for( size_t i = 0; i < mSupported.size(); ++i )
    {
      if( mSupported[i] == aIO )
	{
	  found_requested = true;
	  break;
	}
    }

  // use requested IO if supported, otherwise "fastest"
  mIO = ( found_requested ? aIO : mSupported.back() );

  mCapturing = false;
  mIsLocked = false;

  if (READ == mIO) {
    m_capture = std::make_unique<ReadWriteCapture>(mFd);
  }
  else {
    auto memory = (MMAP == mIO) ? V4L2_MEMORY_MMAP : V4L2_MEMORY_USERPTR;
    m_capture = std::make_unique<StreamingCapture>(mFd, memory);
  }
}

VideoCapture::~VideoCapture()
{
  UnlockFrame();
  StopCapture();
  v4l2_close( mFd );
}

void
VideoCapture::StartCapture()
{
  if( mCapturing ) THROW("already capturing!");
  mCapturing = true;

  // grab current frame format
  v4l2_pix_format fmt = GetFormat();

  // from the v4l2 docs: "Buggy driver paranoia."
  unsigned int min = fmt.width * 2;
  if (fmt.bytesperline < min)
    fmt.bytesperline = min;
  min = fmt.bytesperline * fmt.height;
  if (fmt.sizeimage < min)
    fmt.sizeimage = min;

  const unsigned int bufCount = 4;

  m_capture->startCapture(bufCount, fmt.sizeimage);
}

void
VideoCapture::StopCapture()
{
  if( !mCapturing ) return;
  mCapturing = false;

  m_capture->stopCapture();
}

VideoCapture::Buffer
VideoCapture::LockFrame()
{
  if( mIsLocked ) THROW( "already locked!" );
  mIsLocked = true;

  // wait for frame
  while( true )
    {
      fd_set fds;
      FD_ZERO( &fds);
      FD_SET( mFd, &fds );

      timeval tv;
      tv.tv_sec = 2;
      tv.tv_usec = 0;

      int r = select( mFd + 1, &fds, NULL, NULL, &tv);
      if( -1 == r && EINTR == errno )
	{
	  if( EINTR == errno )
	    continue;
	  THROW( "select() error" );
	}

      // timeout
      if( 0 == r ) continue;

      // fd readable
      break;
    }

  return m_capture->lockFrame();
}

void
VideoCapture::UnlockFrame()
{
  if( !mIsLocked ) return;
  mIsLocked = false;

  m_capture->unlockFrame();
}

std::vector<VideoCapture::IO>
VideoCapture::IOMethods()
{
  std::vector< IO > supported;
	
  if (V4L2Util::canCaptureReadWrite(mFd))
    supported.push_back(READ);

  if (V4L2Util::canCaptureUserPtr(mFd))
    supported.push_back(USERPTR);

  if (V4L2Util::canCaptureMMap(mFd))
    supported.push_back(MMAP);

  return supported;
}
