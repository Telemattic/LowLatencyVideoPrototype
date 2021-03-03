#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <libv4l2.h>

#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

struct Control {
  int   cid;
  const char* name;
};

Control controls[] =
  {
   {V4L2_CID_BRIGHTNESS, "BRIGHTNESS"},
   {V4L2_CID_CONTRAST, "CONTRAST"},
   {V4L2_CID_SATURATION, "SATURATION"},
   {V4L2_CID_HUE, "HUE"},
   {V4L2_CID_BLACK_LEVEL, "BLACK_LEVEL"},
   {V4L2_CID_AUTO_WHITE_BALANCE, "AUTO_WHITE_BALANCE"},
   {V4L2_CID_DO_WHITE_BALANCE, "DO_WHITE_BALANCE"},
   {V4L2_CID_RED_BALANCE, "RED_BALANCE"},
   {V4L2_CID_BLUE_BALANCE, "BLUE_BALANCE"},   
   {V4L2_CID_GAMMA, "GAMMA"},   
   {V4L2_CID_WHITENESS, "WHITENESS"},   
   {V4L2_CID_EXPOSURE, "EXPOSURE"},
   {V4L2_CID_AUTOGAIN, "AUTOGAIN"},
   {V4L2_CID_GAIN, "GAIN"},
   {V4L2_CID_HFLIP, "HFLIP"},
   {V4L2_CID_VFLIP, "VFLIP"},
   {V4L2_CID_POWER_LINE_FREQUENCY, "POWER_LINE_FREQUENCY"},
   {V4L2_CID_HUE_AUTO, "HUE_AUTO"},
   {V4L2_CID_WHITE_BALANCE_TEMPERATURE, "WHITE_BALANCE_TEMPERATURE"},
   {V4L2_CID_SHARPNESS, "SHARPNESS"},
   {V4L2_CID_BACKLIGHT_COMPENSATION, "BACKLIGHT_COMPENSATION"},
   {V4L2_CID_CHROMA_AGC, "CHROMA_AGC"},
   {V4L2_CID_COLOR_KILLER, "COLOR_KILLER"},
   {V4L2_CID_COLORFX, "COLORFX"},
   {V4L2_CID_AUTOBRIGHTNESS, "AUTOBRIGHTNESS"},
   {V4L2_CID_BAND_STOP_FILTER, "BAND_STOP_FILTER"},
   {V4L2_CID_ROTATE, "ROTATE"},
   {V4L2_CID_BG_COLOR, "BG_COLOR"},
   {V4L2_CID_CHROMA_GAIN, "CHROMA_GAIN"},
   {V4L2_CID_ILLUMINATORS_1, "ILLUMINATORS_1"},
   {V4L2_CID_ILLUMINATORS_2, "ILLUMINATORS_2"},
   {V4L2_CID_MIN_BUFFERS_FOR_CAPTURE, "MIN_BUFFERS_FOR_CAPTURE"},
   {V4L2_CID_MIN_BUFFERS_FOR_OUTPUT, "MIN_BUFFERS_FOR_OUTPUT"},
   {V4L2_CID_ALPHA_COMPONENT, "ALPHA_COMPONENT"},
   {V4L2_CID_COLORFX_CBCR, "COLORFX_CBCR"},
   {V4L2_CID_EXPOSURE_AUTO, "EXPOSURE_AUTO"},
   {V4L2_CID_EXPOSURE_ABSOLUTE, "EXPOSURE_ABSOLUTE"},   
   {V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE, "AUTO_N_PRESET_WHITE_BALANCE"},
   {V4L2_CID_ISO_SENSITIVITY, "ISO_SENSITIVITY"},
  };

void
get_controls(int fd)
{
  for (const auto& i : controls) {
    int v = v4l2_get_control(fd, i.cid);
    std::cout << std::setw(2) << (i.cid - V4L2_CID_BASE) << std::setw(28) << i.name << " = " << v << std::endl;
  }
}

std::ostream&
operator<<(std::ostream& os, v4l2_ctrl_type x)
{
  switch (x) {
  case V4L2_CTRL_TYPE_INTEGER:
    os << "INTEGER";
    break;
  case V4L2_CTRL_TYPE_BOOLEAN:
    os << "BOOLEAN";
    break;
  case V4L2_CTRL_TYPE_MENU:
    os << "MENU";
    break;
  case V4L2_CTRL_TYPE_INTEGER_MENU:
    os << "INTEGER_MENU";
    break;
  case V4L2_CTRL_TYPE_BITMASK:
    os << "BITMASK";
    break;
  case V4L2_CTRL_TYPE_BUTTON:
    os << "BUTTON";
    break;
  case V4L2_CTRL_TYPE_INTEGER64:
    os << "INTEGER64";
    break;
  case V4L2_CTRL_TYPE_STRING:
    os << "STRING";
    break;
  case V4L2_CTRL_TYPE_CTRL_CLASS:
    os << "CTRL_CLASS";
    break;
  case V4L2_CTRL_TYPE_U8:
    os << "U8";
    break;
  case V4L2_CTRL_TYPE_U16:
    os << "U16";
    break;
  case V4L2_CTRL_TYPE_U32:
    os << "U32";
    break;
  default:
    os << "UNKNOWN";
    break;
  }

  os << '(' << static_cast<uint32_t>(x) << ')';
  
  return os;
}

void
discoverMenu(int fd, const v4l2_query_ext_ctrl& ctrl)
{
  struct v4l2_querymenu menu;
  
  menu.id = ctrl.id;
  for (menu.index = ctrl.minimum; menu.index <= ctrl.maximum; menu.index += 1) {

    if (0 == ioctl(fd, VIDIOC_QUERYMENU, &menu)) {

      std::cout << "  " << std::setw(2) << menu.index << ". ";

      switch (ctrl.type) {
      case V4L2_CTRL_TYPE_MENU:
	std::cout << '"' << menu.name << '"';
	break;
      case V4L2_CTRL_TYPE_INTEGER_MENU:
	std::cout << menu.value;
	break;
      }
      
      std::cout << std::endl;
    }
  }
}

struct Flags {
  explicit Flags(uint32_t x) : value(x) {}
  const uint32_t value;
};

std::ostream&
operator<<(std::ostream& os, const Flags& flags)
{
  struct Decode { uint32_t flag; const char* desc; };
  const Decode decode[] = {
			   {V4L2_CTRL_FLAG_DISABLED, "disabled"},
			   {V4L2_CTRL_FLAG_GRABBED, "grabbed"},
			   {V4L2_CTRL_FLAG_READ_ONLY, "read-only"},
			   {V4L2_CTRL_FLAG_UPDATE, "update"},
			   {V4L2_CTRL_FLAG_INACTIVE, "inactive"},
			   {V4L2_CTRL_FLAG_SLIDER, "slider"},
			   {V4L2_CTRL_FLAG_WRITE_ONLY, "write-only"},
			   {V4L2_CTRL_FLAG_VOLATILE, "volatile"},
			   {V4L2_CTRL_FLAG_HAS_PAYLOAD, "payload"},
			   {V4L2_CTRL_FLAG_EXECUTE_ON_WRITE, "execute-on-write"}};
  
  uint32_t x = flags.value;
  int n = 0;
  for (const auto& d : decode) {
    if ((x & d.flag) == d.flag) {
      x -= d.flag;
      if (n)
	os << ',';
      os << d.desc;
      ++n;
    }
  }
  if (x) {
    if (n)
      os << ',';
    os << x;
  }
  return os;
}

void
discover(int fd)
{
  std::map<uint32_t, std::vector<uint32_t>> classToControls;
    
  struct v4l2_query_ext_ctrl q;

  uint32_t currentClass = 0;
  
  q.id = V4L2_CTRL_FLAG_NEXT_CTRL|V4L2_CTRL_FLAG_NEXT_COMPOUND;
  while (0 ==ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &q)) {

    if (q.type == V4L2_CTRL_TYPE_CTRL_CLASS) {
      currentClass = q.id;
    }
    else {
      classToControls[currentClass].push_back(q.id);
    }
    
    // whatever
    std::cout << "id=" << q.id << " type=" << static_cast<v4l2_ctrl_type>(q.type) << " name=\"" << q.name << '"';
    switch (q.type) {
    case V4L2_CTRL_TYPE_INTEGER:
    case V4L2_CTRL_TYPE_INTEGER64:
    case V4L2_CTRL_TYPE_BOOLEAN:
    case V4L2_CTRL_TYPE_BITMASK:
    case V4L2_CTRL_TYPE_MENU:
    case V4L2_CTRL_TYPE_INTEGER_MENU:
    case V4L2_CTRL_TYPE_U8:
    case V4L2_CTRL_TYPE_U16:
      std::cout << " default=" << q.default_value;
      break;
    }
    std::cout << std::endl;

    if (q.flags && q.type != V4L2_CTRL_TYPE_CTRL_CLASS)
      std::cout << "  flags=" << Flags{q.flags} << std::endl;

    switch (q.type)
      {
      case V4L2_CTRL_TYPE_INTEGER:
	std::cout << "  minimum=" << q.minimum << " maximum=" << q.maximum << " step=" << q.step << std::endl;
	break;
      case V4L2_CTRL_TYPE_MENU:
      case V4L2_CTRL_TYPE_INTEGER_MENU:
	std::cout << "  minimum=" << q.minimum << " maximum=" << q.maximum << " step=" << q.step << std::endl;
	discoverMenu(fd, q);
	break;
      case V4L2_CTRL_TYPE_BOOLEAN:
	// nop
	break;
      case V4L2_CTRL_TYPE_CTRL_CLASS:
	// not actually a control
	break;
      default:
	std::cout << "  ???" << std::endl;
      }
    q.id |= V4L2_CTRL_FLAG_NEXT_CTRL|V4L2_CTRL_FLAG_NEXT_COMPOUND;
  }

  for (const auto& i : classToControls) {

    std::vector<v4l2_ext_control> controls(i.second.size());
    for(size_t j = 0; j != i.second.size(); ++j) {
      memset(&controls[j], 0, sizeof(v4l2_ext_control));
      controls[j].id = i.second[j];
    }

    struct v4l2_ext_controls q;
    memset(&q, 0, sizeof q);
    
    q.ctrl_class = i.first;
    q.which = V4L2_CTRL_WHICH_CUR_VAL;
    q.count = i.second.size();
    q.error_idx = q.count;
    q.controls = controls.data();

    std::cout << "CLASS=" << i.first << " count=" << q.count << std::endl;
    if (0 == ioctl(fd, VIDIOC_G_EXT_CTRLS, &q)) {
      for (const auto& j : controls) {
	std::cout << "  CID=" << j.id << " value=" << j.value << std::endl;
      }
    }
    else {
      std::cout << "ioctl failed: " << errno << std::endl;
      if (q.error_idx != q.count)
	std::cout << "  error at index " << q.error_idx << std::endl;
    }
  }
}

int main(int argc, char* argv[])
{
  std::string device("/dev/video0");
  
  int fd = v4l2_open(device.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (fd < 0) {
    std::cerr << "can't open " << device << ": " << errno << std::endl;
    return -1;
  }

  get_controls(fd);

  v4l2_set_control(fd, V4L2_CID_HFLIP, 0);
  v4l2_set_control(fd, V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE, 0);
  v4l2_set_control(fd, V4L2_CID_RED_BALANCE, 8186);
  v4l2_set_control(fd, V4L2_CID_BLUE_BALANCE, 8186);

  v4l2_set_control(fd, V4L2_CID_EXPOSURE_AUTO, (V4L2_EXPOSURE_MANUAL << 16) / 3);

  discover(fd);

  {
    v4l2_ext_control controls[2];
    memset(controls, 0, sizeof controls);
    controls[0].id = V4L2_CID_EXPOSURE_AUTO;
    controls[0].value = 1;
    controls[1].id = V4L2_CID_EXPOSURE_ABSOLUTE;
    controls[1].value = 999;

    v4l2_ext_controls arg;
    memset(&arg, 0, sizeof arg);
    arg.ctrl_class = V4L2_CID_CAMERA_CLASS;
    arg.which = V4L2_CTRL_WHICH_CUR_VAL;
    arg.count = 2;
    arg.error_idx = 2;
    arg.controls = controls;

    if (0 == ioctl(fd, VIDIOC_S_EXT_CTRLS, &arg)) {
      std::cout << "ioctl: ok" << std::endl;
    }
    else {
      std::cout << "ioctl: error=" << errno << std::endl;
    }
  }

  v4l2_close(fd);

  return 0;
}
