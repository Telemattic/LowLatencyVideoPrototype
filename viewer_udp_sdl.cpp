#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif

#include <sys/socket.h> /* for socket(), connect(), sendto(), and recvfrom() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_addr() */

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

#include "config.h"
#include "data_source.h"
#include "x264_destreamer.h"


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
  Decoder();

  std::unique_ptr<Frame> operator()(uint8_t *data, size_t len);

private:
  AVCodec*        m_codec    = nullptr;
  AVCodecContext* m_codecCtx = nullptr;
  AVFrame*        m_frame    = nullptr;
};

Decoder::Decoder()
{
  m_codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!m_codec)
    throw std::runtime_error("avcodec_find_decoder");

  m_codecCtx = avcodec_alloc_context3(m_codec);
  if (!m_codecCtx)
    throw std::runtime_error("avcodec_alloc_context3");

  m_codecCtx->pix_fmt = AV_PIX_FMT_YUV420P;
  m_codecCtx->width = WIDTH;
  m_codecCtx->height = HEIGHT;
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

class ReadThread {

public:
  ReadThread(uint32_t frameEventNumber, FrameQueue& frameQueue);

  void run();

private:
  const uint32_t m_frameEventNumber;
  FrameQueue&    m_frameQueue;
  int            m_socket = -1;
  Decoder        m_decoder;
};

ReadThread::ReadThread(uint32_t frameEventNumber, FrameQueue& frameQueue)
  : m_frameEventNumber(frameEventNumber)
  , m_frameQueue(frameQueue)
{
  m_socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (m_socket < 0)
      throw std::runtime_error("socket");

  /* Construct bind structure */
  struct sockaddr_in addr;
  uint16_t broadcastPort = UDP_PORT_NUMBER;
    
  memset(&addr, 0, sizeof addr);
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(broadcastPort);
    
  /* Bind to the broadcast port */
  if (bind(m_socket, reinterpret_cast<const sockaddr *>(&addr), sizeof addr) < 0)
    throw std::runtime_error("bind");
}

void
ReadThread::run()
{
  for (;;) {

    std::array<uint8_t, 2550> buf;
    auto len = ::recvfrom(m_socket, buf.data(), buf.size(), 0, nullptr, nullptr);
    if (len < 0)
      throw std::runtime_error("recvfrom");

    auto frame = m_decoder(buf.data(), len);
    if (frame) {
      m_frameQueue.push(std::move(frame));
      
      SDL_Event event;
      event.type = m_frameEventNumber;
      SDL_PushEvent(&event);
    }
  }
}

int
main( int argc, char *argv[] )
{
    if( SDL_Init(SDL_INIT_VIDEO) < 0 )
        THROW( "Couldn't initialize SDL: " << SDL_GetError() );

    SDL_Rect winRect;
    winRect.w = 640;
    winRect.h = 360;

    SDL_Window* window = SDL_CreateWindow
        (
        "SDL",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        winRect.w, winRect.h,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
        );
    if( !window )
        THROW( "Couldn't create window: " << SDL_GetError() );

    SDL_Renderer* renderer = SDL_CreateRenderer( window, -1, 0 );
    if( !renderer )
        THROW( "Couldn't create renderer: " << SDL_GetError() );

    SDL_RendererInfo info;
    SDL_GetRendererInfo(renderer, &info);
    cout << "Using renderer: " << info.name << endl;

    SDL_Texture* tex = SDL_CreateTexture
        (
        renderer,
        SDL_PIXELFORMAT_IYUV,
        SDL_TEXTUREACCESS_STREAMING,
        WIDTH, HEIGHT
        );
    if( !tex )
        THROW( "Couldn't create texture: " << SDL_GetError() );

    const auto frameEventNumber{SDL_RegisterEvents(1)};
    FrameQueue frameQueue;
    ReadThread readThread{frameEventNumber, frameQueue};
    
    SDL_Thread* ft = SDL_CreateThread(
				      [](void* arg) {
					reinterpret_cast<ReadThread*>(arg)->run();
					return 0; },
				      "ReadThread",
				      &readThread );

    bool running = true;
    while( running )
    {
        SDL_Event event;
        while( SDL_PollEvent( &event ) )
        {
            switch ( event.type )
            {
            case SDL_QUIT:
                running = false;
                break;

            case SDL_KEYUP:
                if( event.key.keysym.sym == SDLK_ESCAPE )
                    running = false;
                if( event.key.keysym.sym == SDLK_f )
                {
                }
                break;

            case SDL_WINDOWEVENT:
                if( event.window.event == SDL_WINDOWEVENT_RESIZED )
                {
                    winRect.w = event.window.data1;
                    winRect.h = event.window.data2;
                    SDL_RenderSetViewport( renderer, NULL );
                }
                if( event.window.event == SDL_WINDOWEVENT_MOVED )
                {
                    winRect.x = event.window.data1;
                    winRect.y = event.window.data2;
                }
                break;
            }

            if (event.type == frameEventNumber) {

	      auto frame = frameQueue.pop();
	      SDL_UpdateTexture(tex, nullptr, frame->data.data(),
				WIDTH * SDL_BYTESPERPIXEL(SDL_PIXELFORMAT_IYUV) );
            }
        }

        SDL_SetRenderDrawColor( renderer, 0, 0, 0, 0 );
        SDL_RenderClear( renderer );

        SDL_Rect src;
        src.w = WIDTH;
        src.h = HEIGHT;
        SDL_Rect s = ScaleAspect(src, winRect );

        SDL_SetRenderDrawColor( renderer, 255, 0, 0, 0 );
        SDL_RenderFillRect( renderer, &s );

        SDL_RenderCopy( renderer, tex, NULL, &s );

        SDL_RenderPresent( renderer );

        SDL_Delay( 10 );
    }

    SDL_DestroyRenderer( renderer );
    SDL_DestroyWindow( window );

    SDL_Quit();

    return 0;
}
