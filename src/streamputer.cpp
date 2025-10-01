/*
 * Copyright (c) 2013, Zhiwei Chu
 * All rights reserved.
 *
 * 文件名称/File name: streamputer.cpp.
 *
 * 摘要/Summary：
 *    将ROS图像消息发送到一个虚拟摄像头上
 *    push ROS image messages into a virtual video device
 *
 * 当前版本/Version：1.0
 *
 */

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>

#include "rclcpp/rclcpp.hpp"

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define ROUND_UP_2(num)  (((num)+1)&~1)
#define ROUND_UP_4(num)  (((num)+3)&~3)
#define ROUND_UP_8(num)  (((num)+7)&~7)
#define ROUND_UP_16(num) (((num)+15)&~15)
#define ROUND_UP_32(num) (((num)+31)&~31)
#define ROUND_UP_64(num) (((num)+63)&~63)

#if 0
# define CHECK_REREAD
#endif

#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480

#define FRAME_FORMAT V4L2_PIX_FMT_YUYV

#define logger rclcpp::get_logger("virtual_camera")

namespace enc = sensor_msgs::image_encodings;

static int debug=0;


int format_properties(const unsigned int format,
    const unsigned int width,
    const unsigned int height,
    size_t*linewidth,
    size_t*framewidth)
{
  size_t lw, fw;
  switch (format)
  {
    case V4L2_PIX_FMT_YUV420: case V4L2_PIX_FMT_YVU420:
      lw = width; /* ??? */
      fw = ROUND_UP_4 (width) * ROUND_UP_2 (height);
      fw += 2 * ((ROUND_UP_8 (width) / 2) * (ROUND_UP_2 (height) / 2));
      break;

    case V4L2_PIX_FMT_UYVY: case V4L2_PIX_FMT_Y41P: case V4L2_PIX_FMT_YUYV: case V4L2_PIX_FMT_YVYU:
      lw = (ROUND_UP_2 (width) * 2);
      fw = lw * height;
      break;

    default:
      return 0;
  }

  if (linewidth)
  {
    *linewidth=lw;
  }
  if (framewidth)
  {
    *framewidth=fw;
  }

  return 1;
}


void print_format(struct v4l2_format*vid_format)
{
  RCLCPP_INFO(logger, "vid_format->type                =%d\n",	vid_format->type );
  RCLCPP_INFO(logger, "vid_format->fmt.pix.width       =%d\n",	vid_format->fmt.pix.width );
  RCLCPP_INFO(logger, "vid_format->fmt.pix.height      =%d\n",	vid_format->fmt.pix.height );
  RCLCPP_INFO(logger, "vid_format->fmt.pix.pixelformat =%d\n",	vid_format->fmt.pix.pixelformat);
  RCLCPP_INFO(logger, "vid_format->fmt.pix.sizeimage   =%d\n",	vid_format->fmt.pix.sizeimage );
  RCLCPP_INFO(logger, "vid_format->fmt.pix.field       =%d\n",	vid_format->fmt.pix.field );
  RCLCPP_INFO(logger, "vid_format->fmt.pix.bytesperline=%d\n",	vid_format->fmt.pix.bytesperline );
  RCLCPP_INFO(logger, "vid_format->fmt.pix.colorspace  =%d\n",	vid_format->fmt.pix.colorspace );
}

class ImageConverter : public rclcpp::Node
{
    int fdwr;
    size_t imageSize;
    size_t lineSize;

    __u8 *buffer;

    cv::Mat yuv_img;
    cv::Size yuv_img_sz;

  public:
    ImageConverter(int fd,  size_t imgsz, size_t linesz) : Node("virtual_camera")
    {
      fdwr = fd;
      imageSize = imgsz;
      lineSize = linesz;

      yuv_img_sz = cv::Size(FRAME_WIDTH, FRAME_HEIGHT);
      yuv_img = cv::Mat(yuv_img_sz, CV_8U);

      //YUV buffer
      buffer = (__u8*)malloc(sizeof(__u8)*imageSize);
      memset(buffer, 0, imageSize);
    }

    ~ImageConverter()
    {
      free(buffer);
    }

    void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
      size_t nx;
      size_t ny;
      unsigned int i = 0;
      __u8 r0,g0,b0,r1,g1,b1;
      __u8 y0,y1,u,v;

      unsigned int ncol;
      uchar* pcol;

      cv_bridge::CvImagePtr cv_ptr;

      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        assert(0);
      }

      try
      {
        cv::resize(cv_ptr->image, yuv_img, yuv_img_sz);
      }
      catch (cv::Exception& e)
      {
        RCLCPP_ERROR(get_logger(), "cvResize error");
        assert(0);
      }

      ncol = yuv_img.cols * yuv_img.channels();
      for (ny=0; ny < yuv_img.rows; ny++)
      {
        pcol = yuv_img.ptr<uchar>(ny);
        for (nx=0; nx < ncol; nx=nx+6)
        {
          r0 = (__u8)pcol[nx];
          g0 = (__u8)pcol[nx+1];
          b0 = (__u8)pcol[nx+2];
          r1 = (__u8)pcol[nx+3];
          g1 = (__u8)pcol[nx+4];
          b1 = (__u8)pcol[nx+5];

          y0 = (__u8)( 0.299*r0 + 0.587*g0 + 0.114*b0);
          y1 = (__u8)( 0.299*r1 + 0.587*g1 + 0.114*b1);
          u  = (__u8)( 0.436*(b0 - y0)/(1-0.114) + 128);
          v  = (__u8)( 0.615*(r0 - y0)/(1-0.299) + 128);

          buffer[i++] = y0;
          buffer[i++] = v;
          buffer[i++] = y1;
          buffer[i++] = u;

        }
        if (i > imageSize)
        {
          RCLCPP_ERROR(get_logger(), "The size of image exceed!");
          return;
        }
      }
      write(fdwr, buffer, imageSize);
    }
};


int main(int argc, char**argv)
{
  struct v4l2_capability vid_caps;
  struct v4l2_format vid_format;

  size_t framesize;
  size_t linewidth;

  char video_device[13];
  int fdwr = 0;
  int ret_code = 0;

  rclcpp::init(argc, argv);

  if (argc > 1)
  {
    strcpy(video_device, argv[1]);
  }
  else
  {
    char *stach = strchr(argv[0], '/');
    char *endch = strrchr(argv[0], '/');
    char dir[200];
    strncpy(dir, argv[0], endch-stach+1);
    dir[endch-stach+1] = 0;
    strncat(dir, "../data/yuyv_camera.txt", 23);
    RCLCPP_INFO(logger, "DIR: %s\n", dir);
    FILE *fp = fopen(dir,"r");
    if (NULL == fp)
    {
      RCLCPP_ERROR(logger, "Open data/yuyv_camera.txt failed!");
      //return -1;
    }
    else
    {
      fscanf(fp,"%s",video_device);
      fclose(fp);
    }

  }

  RCLCPP_INFO(logger, "Using output device: %s\n", video_device);

  //Config camera
  fdwr = open(video_device, O_RDWR);
  assert(fdwr >= 0);

  ret_code = ioctl(fdwr, VIDIOC_QUERYCAP, &vid_caps);
  assert(ret_code != -1);

  memset(&vid_format, 0, sizeof(vid_format));

  ret_code = ioctl(fdwr, VIDIOC_G_FMT, &vid_format);
  if (debug)
  {
    print_format(&vid_format);
  }

  //defurt format
  vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  vid_format.fmt.pix.width = FRAME_WIDTH;
  vid_format.fmt.pix.height = FRAME_HEIGHT;
  vid_format.fmt.pix.pixelformat = FRAME_FORMAT;
  vid_format.fmt.pix.sizeimage = framesize;
  vid_format.fmt.pix.field = V4L2_FIELD_NONE;
  vid_format.fmt.pix.bytesperline = linewidth;
  vid_format.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

  if (debug)
  {
    print_format(&vid_format);
  }
  ret_code = ioctl(fdwr, VIDIOC_S_FMT, &vid_format);

  assert(ret_code != -1);

  if (debug)
  {
    RCLCPP_INFO(logger, "frame: format=%d\tsize=%ld\n", FRAME_FORMAT, framesize);
  }
  print_format(&vid_format);

  if (!format_properties(vid_format.fmt.pix.pixelformat,
                         vid_format.fmt.pix.width,
                         vid_format.fmt.pix.height,
                         &linewidth,
                         &framesize))
  {
    RCLCPP_ERROR(logger, "unable to guess correct settings for format '%d'\n", FRAME_FORMAT);
  }

  auto node = std::make_shared<ImageConverter>(fdwr, framesize, linewidth);

  image_transport::ImageTransport it_(node);
  image_transport::Subscriber image_sub_ = it_.subscribe(
      "image",
      1,
      std::bind(&ImageConverter::imageCb, node, std::placeholders::_1)
  );

  if (image_sub_.getTopic() == "image")
  {
    RCLCPP_WARN(logger, "Topic 'image' has not been remapped! Typical command-line usage:\n"
        "\t$ ros2 run virtual_camera streamputer --ros-args -r image:=<image topic> [\\dev\\video<id>]");
  }

  rclcpp::spin(node);
  rclcpp::shutdown();

  close(fdwr);

  return 0;
}
