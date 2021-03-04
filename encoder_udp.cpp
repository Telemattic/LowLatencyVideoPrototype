#include "v4l2device.hpp"

extern "C" {
#include <libswscale/swscale.h>
#include <libavutil/pixfmt.h>
#include <x264.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
}

#include <cmath>
#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>
#include <map>
#include <numeric>
#include <deque>
#include <algorithm>

#include "config.h"

using namespace std;


__u32 string_to_fourcc( const string& fourcc )
{
    return v4l2_fourcc
        (
        fourcc[ 0 ],
        fourcc[ 1 ],
        fourcc[ 2 ],
        fourcc[ 3 ]
        );
}

// TODO: little-endian (x86) specific?
string fourcc_to_string( const __u32 fourcc )
{
    string ret = "0000";
    ret[0] = static_cast<char>( (fourcc & 0x000000FF) >>  0 );
    ret[1] = static_cast<char>( (fourcc & 0x0000FF00) >>  8 );
    ret[2] = static_cast<char>( (fourcc & 0x00FF0000) >> 16 );
    ret[3] = static_cast<char>( (fourcc & 0xFF000000) >> 24 );
    return ret;
}

ostream& operator<<( ostream& os, const v4l2_fmtdesc& desc )
{
    os << fourcc_to_string( desc.pixelformat ) << " ";

    string flags = "";
    if( desc.flags & V4L2_FMT_FLAG_COMPRESSED )
        flags += "C";
    if( desc.flags & V4L2_FMT_FLAG_EMULATED )
        flags += "E";
    else
        flags += "N";
    os << "(" << flags << ") ";

    os << "[" << desc.description << "]";
    return os;
}

ostream& operator<<( ostream& os, const v4l2_frmsizeenum& size )
{
    if( size.type == V4L2_FRMSIZE_TYPE_DISCRETE )
    {
        os << size.discrete.width << "x" << size.discrete.height;
    }
    else
    {
        os << size.stepwise.min_width << "-" << size.stepwise.max_width;
        os << "," << size.stepwise.step_width;

        os << "x";

        os << size.stepwise.min_height << "-" << size.stepwise.max_height;
        os << "," << size.stepwise.step_height;
    }
    return os;
}

ostream& operator<<( ostream& os, const v4l2_fract& frac )
{
    os << frac.numerator << "/" << frac.denominator;
    return os;
}

ostream& operator<<( ostream& os, const v4l2_frmivalenum& interval )
{
    if( interval.type == V4L2_FRMIVAL_TYPE_DISCRETE )
    {
        os << interval.discrete;
    }
    else
    {
        os << interval.stepwise.min;
        os << "-";
        os << interval.stepwise.max;
        os << ",";
        os << interval.stepwise.step;
    }
    return os;
}


template< typename T >
string TS( const T& val )
{
    ostringstream oss;
    oss << val;
    return oss.str();
}

class Scaler {

public:
  Scaler(const v4l2_pix_format& srcFormat, uint32_t dstWidth, uint32_t dstHeight);
  ~Scaler();

  void operator()(const uint8_t* src, x264_image_t& dst);

private:
  const uint32_t    m_srcWidth;
  const uint32_t    m_srcHeight;
  const uint32_t    m_dstWidth;
  const uint32_t    m_dstHeight;
  std::vector<int>  m_srcOffsets;
  std::vector<int>  m_srcStrides;
  SwsContext*       m_swsCtx = nullptr;
};

Scaler::Scaler(const v4l2_pix_format& src, uint32_t dstWidth, uint32_t dstHeight)
  : m_srcWidth(src.width)
  , m_srcHeight(src.height)
  , m_dstWidth(dstWidth)
  , m_dstHeight(dstHeight)
{
  if (src.pixelformat == V4L2_PIX_FMT_YUV420) {

    // planar format
    m_srcOffsets.push_back( 0 );
    m_srcOffsets.push_back( m_srcOffsets.back() + ( src.height * src.bytesperline ) );
    m_srcOffsets.push_back( m_srcOffsets.back() + ( (src.height/2) * (src.bytesperline/2) ) );

    m_srcStrides.push_back( src.bytesperline );
    m_srcStrides.push_back( src.bytesperline/2 );
    m_srcStrides.push_back( src.bytesperline/2 );
  }
  else {

    // assume packed format
    m_srcOffsets.push_back( 0 );
    m_srcStrides.push_back( src.bytesperline );
  }

  AVPixelFormat srcFormat;

  switch (src.pixelformat) {
  case V4L2_PIX_FMT_YUYV:
    srcFormat = AV_PIX_FMT_YUYV422;
    break;
  case V4L2_PIX_FMT_YUV420:
    srcFormat = AV_PIX_FMT_YUV420P;
    break;
  case V4L2_PIX_FMT_RGB24:
    srcFormat = AV_PIX_FMT_RGB24;
    break;
  case V4L2_PIX_FMT_BGR24:
    srcFormat =  AV_PIX_FMT_BGR24;
    break;
  default:
    throw std::runtime_error("unknown pixel format");
  }

  AVPixelFormat dstFormat = AV_PIX_FMT_YUV420P;

  std::cout << "Scaler: src=" << m_srcWidth << 'x' << m_srcHeight << " (format="
	    << srcFormat << ") dst=" << m_dstWidth << 'x' << m_dstHeight
	    << " (format=" << dstFormat << ')' << std::endl;

  m_swsCtx = sws_getContext(m_srcWidth, m_srcHeight, srcFormat,
			    m_dstWidth, m_dstHeight, dstFormat,
			    SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
  if (!m_swsCtx)
    throw std::runtime_error("sws_getContext");
}

Scaler::~Scaler()
{
  sws_freeContext(m_swsCtx);
}

void
Scaler::operator()(const uint8_t* src, x264_image_t& dst)
{
  std::vector<const uint8_t*> srcPlanes;
  srcPlanes.reserve(m_srcOffsets.size());
  for (auto offset : m_srcOffsets)
    srcPlanes.push_back(src + offset);
  
  sws_scale(m_swsCtx,
	    srcPlanes.data(), m_srcStrides.data(), 0, m_srcHeight,
	    dst.plane, dst.i_stride);
}

double now()
{
    timespec temp;
    clock_gettime( CLOCK_MONOTONIC, &temp );

    return (double)temp.tv_sec + ( (double)temp.tv_nsec / 1e9 );
}


template< typename Cont >
double median( const Cont& arr )
{
    Cont v( arr );

    size_t n = v.size() / 2;
    nth_element( v.begin(), v.begin() + n, v.end() );
    if( n % 2 == 0 )
        return 0.5 * ( v[n] + v[n-1] );
    else
        return v[n];
}


template< typename Cont >
double stdev( const Cont& v )
{
    double sum = accumulate( v.begin(), v.end(), 0.0 );
    double mean = sum / v.size();

    vector<double> diff( v.size() );
    transform( v.begin(), v.end(), diff.begin(), bind2nd(minus<double>(), mean) );

    double sq_sum = inner_product( diff.begin(), diff.end(), diff.begin(), 0.0 );
    double stdev = sqrt( sq_sum / v.size() );

    return stdev;
}

class Encoder {

public:
  Encoder(uint16_t width, uint16_t height, const v4l2_fract& fps);
  ~Encoder();

  void operator()(x264_picture_t& input, std::vector<uint8_t>& output);

private:
  const uint16_t  m_width;
  const uint16_t  m_height;
  x264_t*         m_encoder = nullptr;
};

Encoder::Encoder(uint16_t width, uint16_t height, const v4l2_fract& fps)
  : m_width(width)
  , m_height(height)
{
    // --slice-max-size A
    // --vbv-maxrate B
    // --vbv-bufsize C
    // --crf D
    // --intra-refresh
    // --tune zerolatency

    // A is your packet size
    // B is your connection speed
    // C is (B / FPS)
    // D is a number from 18-30 or so (quality level, lower is better but higher bitrate).

    // Equally, you can do constant bitrate instead of capped constant quality,
    // by replacing CRF with --bitrate B, where B is the maxrate above.

    int packetsize = 1200; // bytes
    int maxrate = 400; // kbps
    int f =  fps.denominator / fps.numerator;
    int C = maxrate / f;

    x264_param_t param;

    x264_param_default_preset( &param, "superfast", "zerolatency" );

    param.i_width   = width;
    param.i_height  = height;
    param.i_fps_num = fps.denominator;
    param.i_fps_den = fps.numerator;
    param.b_repeat_headers = 1;

    x264_param_parse( &param, "slice-max-size", TS(packetsize).c_str() );
    x264_param_parse( &param, "vbv-maxrate", TS(maxrate).c_str() );
    x264_param_parse( &param, "vbv-bufsize", TS(C).c_str() );
    x264_param_parse( &param, "bitrate", TS(maxrate).c_str() );

    x264_param_parse( &param, "intra-refresh", NULL );
    param.i_frame_reference = 1;
    param.b_annexb = 1;

    x264_param_apply_profile(&param, "high");
    
    m_encoder = x264_encoder_open(&param);
}

Encoder::~Encoder()
{
  x264_encoder_close(m_encoder);
}

void
Encoder::operator()(x264_picture_t& input, std::vector<uint8_t>& output)
{
  x264_nal_t* nals = nullptr;
  int num_nals = 0;
  x264_picture_t pic_out;
  if (x264_encoder_encode(m_encoder, &nals, &num_nals, &input, &pic_out) < 0) {
    throw std::runtime_error("x264_encoder_encode");
  }

  size_t numBytes = 0;
  for (int i = 0; i < num_nals; ++i) {
    numBytes += nals[i].i_payload;
  }

  output.clear();
  output.reserve(numBytes);

  for (int i = 0; i < num_nals; ++i) {
    
    auto begin = nals[i].p_payload;
    auto end   = begin + nals[i].i_payload;
    output.insert(output.end(), begin, end);
  }
}

int main( int argc, char** argv )
{
    string device = "/dev/video0";
    string ip = "192.168.0.255";
    unsigned short port = UDP_PORT_NUMBER;
    if( argc >= 2 )
        device = argv[1];
    if( argc >= 3 )
        ip = argv[2];
    if( argc >= 4 )
        port = atoi( argv[3] );

    VideoCapture dev( device );

    cerr << "IO Methods:" << endl;
    vector< VideoCapture::IO > ios = dev.SupportedIO();
    for( size_t i = 0; i < ios.size(); ++i )
    {
        string name;
        if( ios[i] == VideoCapture::READ )    name = "READ";
        if( ios[i] == VideoCapture::USERPTR ) name = "USERPTR";
        if( ios[i] == VideoCapture::MMAP )    name = "MMAP";
        if( ios[i] == dev.SelectedIO() )      name += " *";
        cerr << name << endl;
    }
    cerr << endl;

    // dump supported formats/sizes/fps
    cerr << "Supported formats:" << endl;
    vector< v4l2_fmtdesc > fmts = dev.GetFormats();
    for( size_t i = 0; i < fmts.size(); ++i )
    {
        const v4l2_fmtdesc& fmt = fmts[i];
        cerr << fmt << endl;

        vector< v4l2_frmsizeenum > sizes = dev.GetSizes( fmt );
        for( size_t j = 0; j < sizes.size(); ++j )
        {
            const v4l2_frmsizeenum& size = sizes[j];
            cerr << "\t" << size << ":";

            vector< v4l2_frmivalenum > intervals = dev.GetIntervals( fmt, size );
            for( size_t k = 0; k < intervals.size(); ++k )
            {
                const v4l2_frmivalenum& interval = intervals[k];
                cerr << " (" << interval << ")";
            }
            cerr << endl;
        }
    }
    cerr << endl;

    v4l2_pix_format fmt = dev.GetFormat();
    if (fmt.width != WIDTH || fmt.height != HEIGHT) {

      cerr << "Format size: " << fmt.width << 'x' << fmt.height
	   << ", changing to " << WIDTH << 'x' << HEIGHT << endl;

      fmt.width = WIDTH;
      fmt.height = HEIGHT;
      dev.SetFormat(fmt);

      fmt = dev.GetFormat();
    }
    
    v4l2_fract fps = dev.GetInterval();
    cerr << "Frame info: " << endl;
    cerr << "  Fourcc: " << fourcc_to_string( fmt.pixelformat ) << endl;
    cerr << "    Size: " << fmt.width << "x" << fmt.height << endl;
    cerr << "Interval: " << fps << endl;
    cerr << endl;


    bool formatFound = false;
    
    switch (fmt.pixelformat) {
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_YUV420:
    case V4L2_PIX_FMT_RGB24:
    case V4L2_PIX_FMT_BGR24:
      formatFound = true;
      break;
    }

    if( !formatFound )
    {
        __u32 fallback = V4L2_PIX_FMT_YUV420;

        cerr << fourcc_to_string( fmt.pixelformat ) << " not supported!" << endl;
        cerr << "Switching to " << fourcc_to_string( fallback ) << "...";

        fmt.pixelformat = fallback;
        if( dev.SetFormat(fmt) )
        {
            cerr << "success!" << endl;
        }
        else
        {
            cerr << "failed!" << endl;
            exit( EXIT_FAILURE );
        }
        cerr << endl;

        fmt = dev.GetFormat();
        fps = dev.GetInterval();
        cerr << "Frame info: " << endl;
        cerr << "  Fourcc: " << fourcc_to_string( fmt.pixelformat ) << endl;
        cerr << "    Size: " << fmt.width << "x" << fmt.height << endl;
        cerr << "Interval: " << fps << endl;
        cerr << endl;
    }

    // Allocate conversion context
    unsigned int outputWidth = WIDTH;
    unsigned int outputHeight = HEIGHT;

    Scaler scaler(fmt, outputWidth, outputHeight);

    Encoder encoder(outputWidth, outputHeight, fps);

      // Allocate I420 picture
    x264_picture_t pic_in;
    if( x264_picture_alloc( &pic_in, X264_CSP_I420, outputWidth, outputHeight ) != 0 )
    {
        cerr << "x264 pic alloc fail" << endl;
        exit( EXIT_FAILURE );
    }

    typedef map< string, deque<double> > Acc;
    Acc acc;

    double prv = 0;

    /* Create socket for sending/receiving datagrams */
    int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if( sock < 0 )
    {
        cerr << "cannot open socket" <<endl;
        exit( EXIT_FAILURE );
    }


    /* Set socket to allow broadcast */
    int broadcastPermission = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (void *) &broadcastPermission, 
          sizeof(broadcastPermission)) < 0)
    {
        cerr << "failed to setsockopt(SO_BROADCAST)" <<endl;
        exit( EXIT_FAILURE );
    }

    /* Construct local address structure */
    struct sockaddr_in broadcastAddr;
    memset(&broadcastAddr, 0, sizeof(broadcastAddr));     /* Zero out structure */
    broadcastAddr.sin_family = AF_INET;                   /* Internet address family */
    broadcastAddr.sin_addr.s_addr = inet_addr(ip.c_str());/* Broadcast IP address */
    broadcastAddr.sin_port = htons(port);                 /* Broadcast port */


    dev.StartCapture();
    while( true )
    {
        prv = now();

        const VideoCapture::Buffer& b = dev.LockFrame();
        uint8_t* ptr = reinterpret_cast< unsigned char* >( const_cast< char* >( b.start ) );

        acc["1 - capture(ms):    "].push_back( ( now() - prv ) * 1000.0 );


        prv = now();

	scaler(ptr, pic_in.img);

        acc["2 - scale(ms):     "].push_back( ( now() - prv ) * 1000.0 );

        dev.UnlockFrame();

        prv = now();

	std::vector<uint8_t> buf;
	encoder(pic_in, buf);

        acc["3 - encode(ms):    "].push_back( ( now() - prv ) * 1000.0 );

		//send everything except the first NAL header, which we already sent
                 /* Broadcast sendString in datagram to clients every 3 seconds*/
        //int txSize = buf.size()-4;
        //if (sendto(sock, (const char*)&buf[4], txSize, 0, (struct sockaddr *) 
        int txSize = buf.size();
        if (sendto(sock, (const char*)&buf[0], txSize, 0, (struct sockaddr *) 
               &broadcastAddr, sizeof(broadcastAddr)) != txSize)
        {
            cerr << "failed to sendto" <<endl;
            exit( EXIT_FAILURE );
        }
        cerr <<"Sent "<<txSize<<" bytes"<<endl;

        acc["4 - bytes/frame:   "].push_back( buf.size() );

        static double start = now();
        if( now() - start > 5.0 )
        {
            // print times
            for( Acc::iterator i = acc.begin(); i != acc.end(); ++i )
            {
                deque<double>& arr = i->second;
                while( arr.size() > 300 )
                    arr.pop_front();

                cerr << i->first;
                cerr << "\t" << "Median: " << ( median( arr ) );
                cerr << "\t" << "Stdev: " << ( stdev( arr ) );
                cerr << endl;
            }
            cerr << endl;

            start = now();
        }
    }

    dev.StopCapture();

    return 0;
}
