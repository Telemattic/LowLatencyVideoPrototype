#ifndef INCLUDE_V4L2_DEVICE_GENERIC_HPP
#define INCLUDE_V4L2_DEVICE_GENERIC_HPP

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
 
class Capture {
public:
  virtual ~Capture() {}

  virtual void   startCapture(size_t numBuffers, size_t imageSize) = 0;
  virtual void   stopCapture() = 0;
  
  virtual Buffer lockFrame() = 0;
  virtual void   unlockFrame() = 0;

protected:
  std::vector<Buffer> m_buffers;
};

class ReadWriteCapture : public Capture {

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

ReadWriteCapture::ReadWriteCapture(int fd)
  : m_fd(fd)
  , m_bufferFactory(std::make_unique<HeapBufferFactory>())
{
}

ReadWriteCapture::~ReadWriteCapture()
{
}

void
ReadWriteCapture::startCapture(size_t numBuffers, size_t imageSize)
{
  m_buffers.reserve(1);
  m_buffers.push_back(m_bufferFactory->allocate(imageSize, 0));
}

void
ReadWriteCapture::stopCapture()
{
}

Buffer
ReadWriteCapture::lockFrame()
{
  auto bytesUsed = v4l2_read(m_fd, m_buffers[0].first, m_buffers[0].second);

  if (bytesUsed < 0) {

    if (errno != EAGAIN && errno != EIO)
      THROW( "read() error" );
  }

  return Buffer{m_buffers[0].first, static_cast<size_t>(bytesUsed)};
}

void
ReadWriteCapture::unlockFrame()
{
  // nop
}

class StreamingCapture : public Capture {

public:
  StreamingCapture(int fd, v4l2_memory memory);
  ~StreamingCapture() override;

  void startCapture(size_t numBuffers, size_t imageSize) override;
  void stopCapture() override;

  Buffer lockFrame() override;
  void unlockFrame() override;

private:
  static void xioctl(int fd, int request, void* arg);
  
  const int         m_fd;
  const v4l2_memory m_memory;
  v4l2_buffer       m_lockedBuffer;
  std::unique_ptr<BufferFactory>  m_bufferFactory;
};

void
StreamingCapture::xioctl(int fd, int request, void* arg)
{
  int ret = 0;
  do {
    ret = v4l2_ioctl( fd, request, arg );
  }
  while (ret == -1 && ( (errno == EINTR) || (errno == EAGAIN) ));

  if (ret == -1)
    throw std::runtime_error("ioctl");
}

StreamingCapture::StreamingCapture(int fd, v4l2_memory memory)
  : m_fd(fd)
  , m_memory(memory)
{
  if (V4L2_MEMORY_MMAP == memory)
    m_bufferFactory = std::make_unique<MMapBufferFactory>(m_fd);
  else
    m_bufferFactory = std::make_unique<HeapBufferFactory>();
}

StreamingCapture::~StreamingCapture()
{
}

void
StreamingCapture::startCapture(size_t numBuffers, size_t imageSize)
{
  // request buffers
  v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = m_memory;
  req.count = numBuffers;
  
  xioctl(m_fd, VIDIOC_REQBUFS, &req);

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
      xioctl(m_fd, VIDIOC_QUERYBUF, &buf);

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

    xioctl(m_fd, VIDIOC_QBUF, &buf);
  }

  // start streaming
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  xioctl(m_fd, VIDIOC_STREAMON, &type);
}

void
StreamingCapture::stopCapture()
{
  // stop streaming
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  xioctl(m_fd, VIDIOC_STREAMOFF, &type);
}

Buffer
StreamingCapture::lockFrame()
{
  memset(&m_lockedBuffer, 0, sizeof(m_lockedBuffer));
  m_lockedBuffer.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  m_lockedBuffer.memory = m_memory;

  if (-1 == v4l2_ioctl(m_fd, VIDIOC_DQBUF, &m_lockedBuffer)) {

    if (errno != EAGAIN && errno != EIO)
      THROW( "ioctl(VIDIOC_DQBUF) error" );
  }

  return Buffer{m_buffers[m_lockedBuffer.index].first, m_lockedBuffer.bytesused};
}

void
StreamingCapture::unlockFrame()
{
  xioctl(m_fd, VIDIOC_QBUF, &m_lockedBuffer);
}

// wraps a V4L2_CAP_VIDEO_CAPTURE fd
class VideoCapture
{
public:
    enum IO { READ, USERPTR, MMAP };

    VideoCapture( const std::string& device, const IO aIO = MMAP )
    {
        mFd = v4l2_open( device.c_str(), O_RDWR | O_NONBLOCK, 0);
        if( mFd == - 1 )
            THROW( "can't open " << device << ": " << strerror(errno) );

        // make sure this is a capture device
        v4l2_capability cap;
        xioctl( mFd, VIDIOC_QUERYCAP, &cap );
        if( !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) )
            THROW("not a video capture device!");

        mSupported = IOMethods();
        if( mSupported.empty() ) THROW("no supported IO methods!");

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

    ~VideoCapture()
    {
        UnlockFrame();
        StopCapture();
        v4l2_close( mFd );
    }

    IO SelectedIO()
    {
        return mIO;
    }

    std::vector< IO > SupportedIO()
    {
        return mSupported;
    }

    void StartCapture()
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

    void StopCapture()
    {
        if( !mCapturing ) return;
        mCapturing = false;

	m_capture->stopCapture();
    }

    // wait for next frame and return it
    // timeout in seconds
    // someway to indicate timeout?  zero buffer?
    Buffer LockFrame( const float aTimeout = -1.0f )
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

    void UnlockFrame()
    {
        if( !mIsLocked ) return;
        mIsLocked = false;

	m_capture->unlockFrame();
    }


    v4l2_pix_format GetFormat()
    {
        v4l2_format fmt;
        memset( &fmt, 0, sizeof(fmt) );
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl( mFd, VIDIOC_G_FMT, &fmt );
        return fmt.fmt.pix;
    }

    bool SetFormat( const v4l2_pix_format& aFmt )
    {
        v4l2_format fmt;
        memset( &fmt, 0, sizeof(fmt) );
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix = aFmt;

        while( true )
        {
            try
            {
                xioctl( mFd, VIDIOC_S_FMT, &fmt );
            }
            catch( std::exception& e )
            {
                //if( errno == EBUSY ) continue;
                if( errno == EINVAL ) return false;
                throw e;
            }
            break;
        }

        return true;
    }


    v4l2_fract GetInterval()
    {
        v4l2_streamparm param;
        memset( &param, 0, sizeof(param) );
        param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl( mFd, VIDIOC_G_PARM, &param );
        return param.parm.capture.timeperframe;
    }


    std::vector< v4l2_fmtdesc > GetFormats()
    {
        std::vector< v4l2_fmtdesc > fmts;

        int curIndex = 0;
        while( true )
        {
            v4l2_fmtdesc fmt;
            fmt.index = curIndex++;
            fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            try
            {
                xioctl( mFd, VIDIOC_ENUM_FMT, &fmt );
            }
            catch( std::exception& e )
            {
                // hit the end of the enumeration list, exit
                if( errno == EINVAL ) break;
                else throw e;
            }

            fmts.push_back( fmt );
        }

        return fmts;
    }

    std::vector< v4l2_frmsizeenum > GetSizes( const v4l2_fmtdesc& format )
    {
        std::vector< v4l2_frmsizeenum > sizes;

        int curIndex = 0;
        while( true )
        {
            v4l2_frmsizeenum size;
            size.index = curIndex++;
            size.pixel_format = format.pixelformat;

            try
            {
                xioctl( mFd, VIDIOC_ENUM_FRAMESIZES, &size );
            }
            catch( std::exception& e )
            {
                // hit the end of the enumeration list, exit
                if( errno == EINVAL ) break;
                else throw e;
            }

            sizes.push_back( size );

            // only discrete has multiple enumerations
            if( size.type != V4L2_FRMSIZE_TYPE_DISCRETE )
                break;
        }

        return sizes;
    }

    std::vector< v4l2_frmivalenum > GetIntervals( const v4l2_fmtdesc& format, const v4l2_frmsizeenum& size )
    {
        std::vector< v4l2_frmivalenum > intervals;

        int curIndex = 0;
        while( true )
        {
            v4l2_frmivalenum interval;
            interval.index = curIndex++;
            interval.pixel_format = format.pixelformat;
            interval.width = size.discrete.width;
            interval.height = size.discrete.height;

            try
            {
                xioctl( mFd, VIDIOC_ENUM_FRAMEINTERVALS, &interval ); 
            }
            catch( std::exception& e )
            {
                // hit the end of the enumeration list, exit
                if( errno == EINVAL ) break;
                else throw e;
            }

            intervals.push_back( interval );
            curIndex++;
        }

        return intervals;
    }

private:
    std::vector< IO > IOMethods()
    {
        std::vector< IO > supported;

        v4l2_capability cap;
        xioctl( mFd, VIDIOC_QUERYCAP, &cap );

        // test read/write
        if( cap.capabilities & V4L2_CAP_READWRITE )
            supported.push_back( READ );

        if( cap.capabilities & V4L2_CAP_STREAMING )
        {
            v4l2_requestbuffers req;
            int ret = 0;

            // test userptr
            memset( &req, 0, sizeof(req) );
            req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            req.count = 1;
            req.memory = V4L2_MEMORY_USERPTR;
            if( 0 == v4l2_ioctl( mFd, VIDIOC_REQBUFS, &req ) )
            {
                supported.push_back( USERPTR );
                req.count = 0;
                // blind ioctl, some drivers get pissy with count = 0
                v4l2_ioctl( mFd, VIDIOC_REQBUFS, &req );
            }

            // test mmap
            memset( &req, 0, sizeof(req) );
            req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            req.count = 1;
            req.memory = V4L2_MEMORY_MMAP;
            if( 0 == v4l2_ioctl( mFd, VIDIOC_REQBUFS, &req ) )
            {
                supported.push_back( MMAP );
                req.count = 0;
                // blind ioctl, some drivers get pissy with count = 0
                v4l2_ioctl( mFd, VIDIOC_REQBUFS, &req );
            }
        }

        return supported;
    }

    static const char* strreq( int r )
    {
        if( r == VIDIOC_QUERYCAP )  return "VIDIOC_QUERYCAP";
        if( r == VIDIOC_ENUM_FMT )  return "VIDIOC_ENUM_FMT";
        if( r == VIDIOC_G_FMT )     return "VIDIOC_G_FMT";
        if( r == VIDIOC_REQBUFS )   return "VIDIOC_REQBUFS";
        return "UNKNOWN";
    }

    static void xioctl( int fd, int request, void* arg )
    {
        int ret = 0;
        do
        {
            ret = v4l2_ioctl( fd, request, arg );
        }
        while( ret == -1 && ( (errno == EINTR) || (errno == EAGAIN) ) );

        if( ret == -1 )
            THROW( "Req: " << strreq(request) << ", ioctl() error: " << strerror(errno) );
    }

    int mFd;
    IO mIO;
    std::vector< IO > mSupported;
    bool mCapturing;

    bool mIsLocked;
    v4l2_buffer mLockedBuffer;

  std::unique_ptr<Capture> m_capture;
};

#endif
