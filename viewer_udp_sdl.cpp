#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif

#include <sys/socket.h> /* for socket(), connect(), sendto(), and recvfrom() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_addr() */
#include <unistd.h>

#include <array>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <fstream>
#include <memory>
#include <queue>

#include <SDL.h>
#include <SDL_thread.h>
#include <SDL_mutex.h>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
}

#define MAXRECVSTRING 2550

using namespace std;

#define THROW( d ) \
    { \
	std::ostringstream oss; \
    oss << "[" << __FILE__ << ":" << __LINE__ << "]"; \
	oss << " " << d; \
	throw std::runtime_error( oss.str() ); \
	}


SDL_Rect ScaleAspect( const SDL_Rect& src, const SDL_Rect& dst )
{
    SDL_Rect ret;
    ret.w = dst.w;
    ret.h = dst.h;

    double srcRatio = (double)src.w / src.h;
    double dstRatio = (double)dst.w / dst.h;
    if( srcRatio > dstRatio )
        ret.h = dst.w / srcRatio;
    else
        ret.w = dst.h * srcRatio;

    ret.x = (dst.w - ret.w) / 2;
    ret.y = (dst.h - ret.h) / 2;

    return ret;
}

struct Frame {

  uint32_t             numBytes = 0;
  std::vector<uint8_t> data;
};

class FrameQueue
{
public:
  FrameQueue();
  ~FrameQueue();

  void push(std::unique_ptr<Frame> f);
  std::unique_ptr<Frame> pop();

private:
  std::deque<std::unique_ptr<Frame>> m_deque;
  SDL_mutex* m_mutex;
};

FrameQueue::FrameQueue()
  : m_mutex(SDL_CreateMutex())
{
}

FrameQueue::~FrameQueue()
{
  SDL_DestroyMutex(m_mutex);
}

void
FrameQueue::push(std::unique_ptr<Frame> f)
{
  SDL_mutexP(m_mutex);
  m_deque.push_back(std::move(f));
  SDL_mutexV(m_mutex);
}

std::unique_ptr<Frame>
FrameQueue::pop()
{
  SDL_mutexP(m_mutex);
  std::unique_ptr<Frame> retval(m_deque.begin()->release());
  m_deque.pop_front();
  SDL_mutexV(m_mutex);
  return retval;
}

class Decoder {

public:
  Decoder(uint16_t width, uint16_t height);

  std::unique_ptr<Frame> operator()(uint8_t *data, size_t len);

private:
  const uint16_t  m_width;
  const uint16_t  m_height;
  AVCodec*        m_codec    = nullptr;
  AVCodecContext* m_codecCtx = nullptr;
  AVFrame*        m_frame    = nullptr;
  uint32_t        m_numBytes = 0;
};

Decoder::Decoder(uint16_t width, uint16_t height)
  : m_width(width)
  , m_height(height)
{
  m_codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!m_codec)
    throw std::runtime_error("avcodec_find_decoder");

  m_codecCtx = avcodec_alloc_context3(m_codec);
  if (!m_codecCtx)
    throw std::runtime_error("avcodec_alloc_context3");

  m_codecCtx->pix_fmt = AV_PIX_FMT_YUV420P;
  m_codecCtx->width = m_width;
  m_codecCtx->height = m_height;
  if (avcodec_open2(m_codecCtx, m_codec, nullptr) < 0)
    throw std::runtime_error("avcodec_open2");

  if (m_codecCtx->time_base.num > 1000 && m_codecCtx->time_base.den == 1)
    m_codecCtx->time_base.den = 1000;

  m_frame = av_frame_alloc();
  if (!m_frame)
    throw std::runtime_error("av_frame_alloc");
}

std::unique_ptr<Frame>
Decoder::operator()(uint8_t* data, size_t len)
{
  // Decode video frame
  AVPacket avpkt;
  av_init_packet( &avpkt );
  avpkt.data = data;
  avpkt.size = len;

  int gotFrame = 0;
  avcodec_decode_video2(m_codecCtx, m_frame, &gotFrame, &avpkt );

  m_numBytes += len;

  if (0 == gotFrame)
    return {};

  std::array<unsigned int, 3> widths;
  widths[0] = m_frame->width;
  widths[1] = m_frame->width / 2;
  widths[2] = m_frame->width / 2;
      
  std::array<unsigned int, 3> heights;
  heights[0] = m_frame->height;
  heights[1] = m_frame->height / 2;
  heights[2] = m_frame->height / 2;
      
  auto retval = std::make_unique<Frame>();
  
  retval->numBytes = m_numBytes;
  m_numBytes = 0;
  {
    size_t n = 0;
    for (unsigned int i = 0; i != 3; ++i) {
      n += widths[i] * heights[i];
    }
    retval->data.reserve(n);
  }

  for ( unsigned int plane = 0; plane < 3; ++plane ){

    auto base = m_frame->data[plane];
    const auto h = heights[plane];
    const auto w = widths[plane];
	
    for ( unsigned int y = 0; y != h; ++y ) {

      retval->data.insert(retval->data.end(), base, base + w);
      base += m_frame->linesize[plane];
    }
  }

  return retval;
}

class UDPSocket {
public:
  explicit UDPSocket(uint16_t port);
  ~UDPSocket();

  size_t read(void* buf, size_t len);
private:
  int m_socket = -1;
};

UDPSocket::UDPSocket(uint16_t port)
{
  m_socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (m_socket < 0)
      throw std::runtime_error("socket");

  /* Construct bind structure */
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof addr);
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port);
    
  /* Bind to the broadcast port */
  if (bind(m_socket, reinterpret_cast<const sockaddr *>(&addr), sizeof addr) < 0)
    throw std::runtime_error("bind");
}

UDPSocket::~UDPSocket()
{
  ::close(m_socket);
}

size_t
UDPSocket::read(void* data, size_t len)
{
  const auto x = ::recvfrom(m_socket, data, len, 0, nullptr, nullptr);
  if (x < 0)
    throw std::runtime_error("recvfrom");

  return static_cast<size_t>(x);
}

class Reader {

public:
  static int ThreadFunc(void*);
  
  Reader(UDPSocket&, Decoder&, uint32_t frameEventNumber, FrameQueue&);

  void run();

private:
  UDPSocket&     m_socket;
  Decoder&       m_decoder;
  const uint32_t m_frameEventNumber;
  FrameQueue&    m_frameQueue;
};

int
Reader::ThreadFunc(void* arg)
{
  reinterpret_cast<Reader*>(arg)->run();
  return 0;
}

Reader::Reader(UDPSocket& socket, Decoder& decoder,
	       uint32_t frameEventNumber, FrameQueue& frameQueue)
  : m_socket(socket)
  , m_decoder(decoder)
  , m_frameEventNumber(frameEventNumber)
  , m_frameQueue(frameQueue)
{
}

void
Reader::run()
{
  for (;;) {

    std::array<uint8_t, 2550> buf;
    const auto len = m_socket.read(buf.data(), buf.size());

    if (auto frame = m_decoder(buf.data(), len)) {

      m_frameQueue.push(std::move(frame));
      
      SDL_Event event;
      event.type = m_frameEventNumber;
      SDL_PushEvent(&event);
    }
  }
}

class UI {
  
public:
  UI(uint16_t width, uint16_t height, uint32_t frameEventNumber, FrameQueue&);
  ~UI();

  void run();

private:
  const uint16_t m_width;
  const uint16_t m_height;
  const uint32_t m_frameEventNumber;
  FrameQueue&    m_frameQueue;
  SDL_Rect       m_winRect;
  SDL_Window*    m_window = nullptr;
  SDL_Renderer*  m_renderer = nullptr;
  SDL_Texture*   m_texture = nullptr;
};

UI::UI(uint16_t width, uint16_t height, uint32_t frameEventNumber, FrameQueue& frameQueue)
  : m_width{width}
  , m_height{height}
  , m_frameEventNumber(frameEventNumber)
  , m_frameQueue(frameQueue)
  , m_winRect{0, 0, 640, 360}
{
  m_window = SDL_CreateWindow
    (
     "SDL",
     SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
     m_winRect.w, m_winRect.h,
     SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
     );
  if (!m_window)
    THROW( "Couldn't create window: " << SDL_GetError() );

  m_renderer = SDL_CreateRenderer(m_window, -1, 0);
  if (!m_renderer)
    THROW( "Couldn't create renderer: " << SDL_GetError() );

  m_texture = SDL_CreateTexture(
				m_renderer,
				SDL_PIXELFORMAT_IYUV,
				SDL_TEXTUREACCESS_STREAMING,
				m_width, m_height);
  if (!m_texture)
    THROW( "Couldn't create texture: " << SDL_GetError() );
}

UI::~UI()
{
  SDL_DestroyRenderer(m_renderer);
  SDL_DestroyWindow(m_window);
}

double now()
{
    timespec temp;
    clock_gettime( CLOCK_MONOTONIC, &temp );

    return (double)temp.tv_sec + ( (double)temp.tv_nsec / 1e9 );
}

void
UI::run()
{
  double start = now();
  int numFrames = 0;
  int numBytes = 0;
  
  bool running = true;
  while (running) {

    SDL_Event event;
    while (SDL_PollEvent(&event) ) {

      switch (event.type) {
	
      case SDL_QUIT:
	running = false;
	break;

      case SDL_KEYUP:
	if (event.key.keysym.sym == SDLK_ESCAPE)
	  running = false;
	break;

      case SDL_WINDOWEVENT:
	if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
	  
	  m_winRect.w = event.window.data1;
	  m_winRect.h = event.window.data2;
	  SDL_RenderSetViewport(m_renderer, nullptr);
	}
	else if (event.window.event == SDL_WINDOWEVENT_MOVED) {
	  
	  m_winRect.x = event.window.data1;
	  m_winRect.y = event.window.data2;
	}
	break;

      default:
	if (event.type == m_frameEventNumber) {

	  auto frame = m_frameQueue.pop();
	  SDL_UpdateTexture(m_texture, nullptr, frame->data.data(),
			    m_width * SDL_BYTESPERPIXEL(SDL_PIXELFORMAT_IYUV));

	  ++numFrames;
	  numBytes += frame->numBytes;
	  auto elapsed = now() - start;
	  if (numFrames > 30 || elapsed > 5.) {
	    double fps = numFrames / elapsed;
	    double bytesPerFrame = numBytes / static_cast<double>(numFrames);
	    double bytesPerSec = numBytes / elapsed;
	    std::cout << "fps=" << fps
		      << " bytes/frame=" << bytesPerFrame
		      << " bytes/sec=" << bytesPerSec << std::endl;
	    numFrames = 0;
	    numBytes = 0;
	    start = now();
	  }
        }
      }

      SDL_SetRenderDrawColor(m_renderer, 0, 0, 0, 0);
      SDL_RenderClear(m_renderer);

      SDL_Rect src{0, 0, m_width, m_height};
      SDL_Rect s = ScaleAspect(src, m_winRect);

      SDL_SetRenderDrawColor(m_renderer, 255, 0, 0, 0);
      SDL_RenderFillRect(m_renderer, &s);

      SDL_RenderCopy(m_renderer, m_texture, nullptr, &s);

      SDL_RenderPresent(m_renderer);

      SDL_Delay(10);
    }
  }
}

int
main( int argc, char *argv[] )
{
  uint16_t cameraW = 320;
  uint16_t cameraH = 240;
  uint16_t port = 12345;

  UDPSocket socket{port};
  Decoder   decoder{cameraW, cameraH};
  
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
    THROW( "Couldn't initialize SDL: " << SDL_GetError() );

  const auto frameEventNumber{SDL_RegisterEvents(1)};
  FrameQueue frameQueue;
    
  Reader reader{socket, decoder, frameEventNumber, frameQueue};
  UI ui{cameraW, cameraH, frameEventNumber, frameQueue};

  SDL_Thread* ft = SDL_CreateThread(Reader::ThreadFunc,
				    "ReadThread",
				    &reader );

  ui.run();

  SDL_Quit();

  return 0;
}
