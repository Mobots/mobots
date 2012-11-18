/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#define __STDC_CONSTANT_MACROS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <asm/types.h>          /* for videodev2.h */

extern "C" {
#include <linux/videodev2.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

#include "usb_cam.h"

#define CLEAR(x) memset (&(x), 0, sizeof (x))

struct buffer {
  void * start;
  size_t length;
};

static char *camera_dev;
static int fd = -1;
struct buffer * buffers = NULL;
static unsigned int n_buffers = 0;
static AVFrame *avframe_camera = NULL;
static AVFrame *avframe_rgb = NULL;
static AVCodec *avcodec = NULL;
static AVCodecContext *avcodec_context = NULL;
static int avframe_camera_size = 0;
static int avframe_rgb_size = 0;

struct SwsContext *video_sws = NULL;
static void (*handler)(const char*);

static void errno_exit(const char * s)
{
  fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
	handler(s);
  //exit(EXIT_FAILURE);
}

void usb_cam_setErrorHandler(void (*func)(const char*)){
	handler = func;
}

static int xioctl(int fd, int request, void * arg)
{
  int r;

  do
    r = ioctl(fd, request, arg); while (-1==r&&EINTR==errno);

  return r;
}

const unsigned char uchar_clipping_table[] = {
  0,0,0,0,0,0,0,0, // -128 - -121
  0,0,0,0,0,0,0,0, // -120 - -113
  0,0,0,0,0,0,0,0, // -112 - -105
  0,0,0,0,0,0,0,0, // -104 -  -97
  0,0,0,0,0,0,0,0, //  -96 -  -89
  0,0,0,0,0,0,0,0, //  -88 -  -81
  0,0,0,0,0,0,0,0, //  -80 -  -73
  0,0,0,0,0,0,0,0, //  -72 -  -65
  0,0,0,0,0,0,0,0, //  -64 -  -57
  0,0,0,0,0,0,0,0, //  -56 -  -49
  0,0,0,0,0,0,0,0, //  -48 -  -41
  0,0,0,0,0,0,0,0, //  -40 -  -33
  0,0,0,0,0,0,0,0, //  -32 -  -25
  0,0,0,0,0,0,0,0, //  -24 -  -17
  0,0,0,0,0,0,0,0, //  -16 -   -9
  0,0,0,0,0,0,0,0, //   -8 -   -1
  0,1,2,3,4,5,6,7,
  8,9,10,11,12,13,14,15,
  16,17,18,19,20,21,22,23,
  24,25,26,27,28,29,30,31,
  32,33,34,35,36,37,38,39,
  40,41,42,43,44,45,46,47,
  48,49,50,51,52,53,54,55,
  56,57,58,59,60,61,62,63,
  64,65,66,67,68,69,70,71,
  72,73,74,75,76,77,78,79,
  80,81,82,83,84,85,86,87,
  88,89,90,91,92,93,94,95,
  96,97,98,99,100,101,102,103,
  104,105,106,107,108,109,110,111,
  112,113,114,115,116,117,118,119,
  120,121,122,123,124,125,126,127,
  128,129,130,131,132,133,134,135,
  136,137,138,139,140,141,142,143,
  144,145,146,147,148,149,150,151,
  152,153,154,155,156,157,158,159,
  160,161,162,163,164,165,166,167,
  168,169,170,171,172,173,174,175,
  176,177,178,179,180,181,182,183,
  184,185,186,187,188,189,190,191,
  192,193,194,195,196,197,198,199,
  200,201,202,203,204,205,206,207,
  208,209,210,211,212,213,214,215,
  216,217,218,219,220,221,222,223,
  224,225,226,227,228,229,230,231,
  232,233,234,235,236,237,238,239,
  240,241,242,243,244,245,246,247,
  248,249,250,251,252,253,254,255,
  255,255,255,255,255,255,255,255, // 256-263
  255,255,255,255,255,255,255,255, // 264-271
  255,255,255,255,255,255,255,255, // 272-279
  255,255,255,255,255,255,255,255, // 280-287
  255,255,255,255,255,255,255,255, // 288-295
  255,255,255,255,255,255,255,255, // 296-303
  255,255,255,255,255,255,255,255, // 304-311
  255,255,255,255,255,255,255,255, // 312-319
  255,255,255,255,255,255,255,255, // 320-327
  255,255,255,255,255,255,255,255, // 328-335
  255,255,255,255,255,255,255,255, // 336-343
  255,255,255,255,255,255,255,255, // 344-351
  255,255,255,255,255,255,255,255, // 352-359
  255,255,255,255,255,255,255,255, // 360-367
  255,255,255,255,255,255,255,255, // 368-375
  255,255,255,255,255,255,255,255, // 376-383
};
const int clipping_table_offset = 128;

/** Clip a value to the range 0<val<255. For speed this is done using an
 * array, so can only cope with numbers in the range -128<val<383.
 */
static unsigned char
CLIPVALUE(int val)
{
  // Old method (if)
/*   val = val < 0 ? 0 : val; */
/*   return val > 255 ? 255 : val; */

  // New method (array)
  return uchar_clipping_table[val+clipping_table_offset];
}

/**
 * Conversion from YUV to RGB.
 * The normal conversion matrix is due to Julien (surname unknown):
 *
 * [ R ]   [  1.0   0.0     1.403 ] [ Y ]
 * [ G ] = [  1.0  -0.344  -0.714 ] [ U ]
 * [ B ]   [  1.0   1.770   0.0   ] [ V ]
 *
 * and the firewire one is similar:
 *
 * [ R ]   [  1.0   0.0     0.700 ] [ Y ]
 * [ G ] = [  1.0  -0.198  -0.291 ] [ U ]
 * [ B ]   [  1.0   1.015   0.0   ] [ V ]
 *
 * Corrected by BJT (coriander's transforms RGB->YUV and YUV->RGB
 *                   do not get you back to the same RGB!)
 * [ R ]   [  1.0   0.0     1.136 ] [ Y ]
 * [ G ] = [  1.0  -0.396  -0.578 ] [ U ]
 * [ B ]   [  1.0   2.041   0.002 ] [ V ]
 *
 */
static void
YUV2RGB(const unsigned char y,
        const unsigned char u,
        const unsigned char v,
        unsigned char* r,
        unsigned char* g,
        unsigned char* b)
{
  const int y2=(int)y;
  const int u2=(int)u-128;
  const int v2=(int)v-128;
  //std::cerr << "YUV=("<<y2<<","<<u2<<","<<v2<<")"<<std::endl;


  // This is the normal YUV conversion, but
  // appears to be incorrect for the firewire cameras
  //   int r2 = y2 + ( (v2*91947) >> 16);
  //   int g2 = y2 - ( ((u2*22544) + (v2*46793)) >> 16 );
  //   int b2 = y2 + ( (u2*115999) >> 16);
  // This is an adjusted version (UV spread out a bit)
  int r2 = y2 + ( (v2*37221) >> 15);
  int g2 = y2 - ( ((u2*12975) + (v2*18949)) >> 15 );
  int b2 = y2 + ( (u2*66883) >> 15);
  //std::cerr << "   RGB=("<<r2<<","<<g2<<","<<b2<<")"<<std::endl;


  // Cap the values.
  *r=CLIPVALUE(r2);
  *g=CLIPVALUE(g2);
  *b=CLIPVALUE(b2);
}

void
uyvy2rgb (char *YUV, char *RGB, int NumPixels) {
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;
  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
    {
      u = (unsigned char) YUV[i + 0];
      y0 = (unsigned char) YUV[i + 1];
      v = (unsigned char) YUV[i + 2];
      y1 = (unsigned char) YUV[i + 3];
      YUV2RGB (y0, u, v, &r, &g, &b);
      RGB[j + 0] = r;
      RGB[j + 1] = g;
      RGB[j + 2] = b;
      YUV2RGB (y1, u, v, &r, &g, &b);
      RGB[j + 3] = r;
      RGB[j + 4] = g;
      RGB[j + 5] = b;
    }
}

static void
yuyv2rgb(char *YUV, char *RGB, int NumPixels) {
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;

  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
    {
      y0 = (unsigned char) YUV[i + 0];
      u = (unsigned char) YUV[i + 1];
      y1 = (unsigned char) YUV[i + 2];
      v = (unsigned char) YUV[i + 3];
      YUV2RGB (y0, u, v, &r, &g, &b);
      RGB[j + 0] = r;
      RGB[j + 1] = g;
      RGB[j + 2] = b;
      YUV2RGB (y1, u, v, &r, &g, &b);
      RGB[j + 3] = r;
      RGB[j + 4] = g;
      RGB[j + 5] = b;
    }
}

static void process_image(const void * src, int len, usb_cam_camera_image_t *dest)
{
    yuyv2rgb((char*)src, dest->image, dest->width*dest->height);
}

static int read_frame(usb_cam_camera_image_t *image)
{
  struct v4l2_buffer buf;
  unsigned int i;
  int len;

    CLEAR (buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1==xioctl(fd, VIDIOC_DQBUF, &buf)) {
      switch (errno) {
      case EAGAIN:
        return 0;

      case EIO:
        /* Could ignore EIO, see spec. */

        /* fall through */

      default:
        errno_exit("VIDIOC_DQBUF");
      }
    }

    assert (buf.index < n_buffers);
    len = buf.bytesused;
    process_image(buffers[buf.index].start, len, image);

    if (-1==xioctl(fd, VIDIOC_QBUF, &buf))
      errno_exit("VIDIOC_QBUF");


  return 1;
}

static void stop_capturing(void)
{
  enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1==xioctl(fd, VIDIOC_STREAMOFF, &type))
      errno_exit("VIDIOC_STREAMOFF");
}

static void start_capturing(void)
{
  unsigned int i;
  enum v4l2_buf_type type;

    for(i = 0; i<n_buffers; ++i) {
      struct v4l2_buffer buf;

      CLEAR (buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;

      if (-1==xioctl(fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1==xioctl(fd, VIDIOC_STREAMON, &type))
      errno_exit("VIDIOC_STREAMON");
}

static void uninit_device(void)
{
  unsigned int i;

    for(i = 0; i<n_buffers; ++i)
      if (-1==munmap(buffers[i].start, buffers[i].length))
        errno_exit("munmap");

  free(buffers);
}

static void init_mmap(void)
{
  struct v4l2_requestbuffers req;

  CLEAR (req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1==xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL==errno) {
      fprintf(stderr, "%s does not support memory mapping\n", camera_dev);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }

  if (req.count<2) {
    fprintf(stderr, "Insufficient buffer memory on %s\n", camera_dev);
    exit(EXIT_FAILURE);
  }

  buffers = (buffer*)calloc(req.count, sizeof(*buffers));

  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }

  for(n_buffers = 0; n_buffers<req.count; ++n_buffers) {
    struct v4l2_buffer buf;

    CLEAR (buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;

    if (-1==xioctl(fd, VIDIOC_QUERYBUF, &buf))
      errno_exit("VIDIOC_QUERYBUF");

    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start = mmap(NULL /* start anywhere */, buf.length, PROT_READ|PROT_WRITE /* required */, MAP_SHARED /* recommended */, fd, buf.m.offset);

    if (MAP_FAILED==buffers[n_buffers].start)
      errno_exit("mmap");
  }
}

static void init_device(int image_width, int image_height)
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;

  if (-1==xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL==errno) {
      fprintf(stderr, "%s is no V4L2 device\n", camera_dev);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_QUERYCAP");
    }
  }

  if (!(cap.capabilities&V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "%s is no video capture device\n", camera_dev);
    exit(EXIT_FAILURE);
  }
  
    if (!(cap.capabilities&V4L2_CAP_STREAMING)) {
      fprintf(stderr, "%s does not support streaming i/o\n", camera_dev);
      exit(EXIT_FAILURE);
    }

  /* Select video input, video standard and tune here. */

  CLEAR (cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0==xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1==xioctl(fd, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
      case EINVAL:
        /* Cropping not supported. */
        break;
      default:
        /* Errors ignored. */
        break;
      }
    }
  } else {
    /* Errors ignored. */
  }

  CLEAR (fmt);

//  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//  fmt.fmt.pix.width = 640;
//  fmt.fmt.pix.height = 480;
//  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
//  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = image_width;
  fmt.fmt.pix.height = image_height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;


  if (-1==xioctl(fd, VIDIOC_S_FMT, &fmt))
    errno_exit("VIDIOC_S_FMT");

  /* Note VIDIOC_S_FMT may change width and height. */

  /* Buggy driver paranoia. */
  min = fmt.fmt.pix.width*2;
  if (fmt.fmt.pix.bytesperline<min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline*fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage<min)
    fmt.fmt.pix.sizeimage = min;

  image_width = fmt.fmt.pix.width;
  image_height = fmt.fmt.pix.height;

    init_mmap();
}

static void close_device(void)
{
  if (-1==close(fd))
    errno_exit("close");

  fd = -1;
}

static void open_device(void)
{
  struct stat st;

  if (-1==stat(camera_dev, &st)) {
    fprintf(stderr, "Cannot identify '%s': %d, %s\n", camera_dev, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }

  if (!S_ISCHR (st.st_mode)) {
    fprintf(stderr, "%s is no device\n", camera_dev);
    exit(EXIT_FAILURE);
  }

  fd = open(camera_dev, O_RDWR /* required */|O_NONBLOCK, 0);

  if (-1==fd) {
    fprintf(stderr, "Cannot open '%s': %d, %s\n", camera_dev, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
}

usb_cam_camera_image_t *usb_cam_camera_start(const char* dev, int image_width, int image_height)
{
  camera_dev = (char*)calloc(1,strlen(dev)+1);
  strcpy(camera_dev,dev);

  usb_cam_camera_image_t *image;

  open_device();
  init_device(image_width, image_height);
  start_capturing();

  image = (usb_cam_camera_image_t *) calloc(1, sizeof(usb_cam_camera_image_t));

  image->width = image_width;
  image->height = image_height;
  image->bytes_per_pixel = 24;

  image->image_size = image->width*image->height*image->bytes_per_pixel;
  image->is_new = 0;
  image->image = (char *) calloc(image->image_size, sizeof(char));
  memset(image->image, 0, image->image_size*sizeof(char));

  return image;
}

void usb_cam_camera_shutdown(void)
{
  stop_capturing();
  uninit_device();
  close_device();

  if (avcodec_context) {
    avcodec_close(avcodec_context);
    av_free(avcodec_context);
    avcodec_context = NULL;
  }
  if (avframe_camera)
    av_free(avframe_camera);
  avframe_camera = NULL;
  if (avframe_rgb)
    av_free(avframe_rgb);
  avframe_rgb = NULL;
}

void usb_cam_camera_grab_image(usb_cam_camera_image_t *image)
{
  fd_set fds;
  struct timeval tv;
  int r;

  FD_ZERO (&fds);
  FD_SET (fd, &fds);

  /* Timeout. */
  tv.tv_sec = 5;
  tv.tv_usec = 0;

  r = select(fd+1, &fds, NULL, NULL, &tv);

  if (-1==r) {
    if (EINTR==errno)
      return;

    errno_exit("select");
  }

  if (0==r) {
    //fprintf(stderr, "select timeout\n");
    errno_exit("select");
    //exit(EXIT_FAILURE);
  }

  read_frame(image);
  image->is_new = 1;
}

// enables/disables auto focus
void usb_cam_camera_set_auto_focus(int value)
{
  struct v4l2_queryctrl queryctrl;
  struct v4l2_ext_control control;

  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = V4L2_CID_FOCUS_AUTO;

  if (-1 == xioctl(fd, VIDIOC_QUERYCTRL, &queryctrl)) {
    if (errno != EINVAL) {
      perror("VIDIOC_QUERYCTRL");
      return;
    } else {
      printf("V4L2_CID_FOCUS_AUTO is not supported\n");
      return;
    }
  } else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    printf("V4L2_CID_FOCUS_AUTO is not supported\n");
    return;
  } else {
    memset(&control, 0, sizeof(control));
    control.id = V4L2_CID_FOCUS_AUTO;
    control.value = value;

    if (-1 == xioctl(fd, VIDIOC_S_CTRL, &control)) {
      perror("VIDIOC_S_CTRL");
      return;
    }
  }
}



