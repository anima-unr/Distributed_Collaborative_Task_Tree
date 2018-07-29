/*
 * color_blob_finder
 * Copyright (c) 2018, David Feil-Seifer
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Nevada, Reno nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "ros/ros.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "image_transport/image_transport.h"
//#include "image_geometry/pinhole_camera_model.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/fill_image.h"

//#include "contour_fcns.h"
#include "blob_vision/color_finder.h"

//#include "ray.h"

using namespace std;

class ImageSplitter
{
  public:
    ros::NodeHandle n;
  private:
    ros::Publisher hsv_pub;
    ros::Publisher foreground_pub;

    sensor_msgs::Image img_;
    cv::Mat hsv_img_, disp_;
    bool first;
    int display;

    //IplConvKernel *kernel_;
    std::vector<cv::Vec4i> storage_;
    vector<ColorFinder> color_finders_;
    vector<cv::Scalar> colors_;

    //int vmin, vmax, smin;
  public:

  void image_cb( const sensor_msgs::ImageConstPtr& msg )
  {
    cv_bridge::CvImagePtr cv_ptr;
    /* get frame */
    try {
      cv_ptr = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
    }
    catch( cv_bridge::Exception &e ) 
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  
    cv::Mat frame = cv_ptr->image;
    ros::Time img_time = msg->header.stamp;
    
    if( frame.rows != hsv_img_.rows )
    {
      ROS_WARN( "resizing images to: [%d,%d]", frame.cols, frame.rows );
      //resize images
      hsv_img_.create(cv::Size(frame.rows,frame.cols), CV_8UC3 );
      disp_.create(cv::Size(frame.rows,frame.cols), CV_8UC3 );
    }
    disp_ = frame.clone();
    cv::cvtColor( frame, hsv_img_, CV_BGR2HSV );
      
    // color finders
    int c = 0;
    for( vector<ColorFinder>::iterator i = color_finders_.begin();
         i != color_finders_.end(); i++ )
    {
      // color probability images
      i->image_cb( hsv_img_ );
      // find contours
      i->find_blobs(msg->header.stamp);
      // publish blobs

      // render blobs
      //vector<oit_msgs::Blob> blobs = i->get_blobs();
      /*
      for( unsigned int j = 0; j < blobs.size(); j++ )
      {

        oit_msgs::Blob b = blobs[j];
        cvRectangle( disp_, cvPoint( b.x, b.y ), 
                            cvPoint( b.x+b.width, b.y+b.height ),
                            colors_[c], 1 );         
      }
      */
      c++;
    }

    // TODO: fill for foreground color
    if( display > 0 )
    {
      cv::imshow( "output", disp_ );
      cv::waitKey(10);
    }
  }

  void init()
  {
    //hsv_pub = n.advertise<sensor_msgs::Image>("image_hsv",1000);
    //foreground_pub = n.advertise<oit_msgs::BlobArray>("foreground_blobs",1000);
    n = ros::NodeHandle("~");
    std::string colorfile, irfile, childfile;
    n.param("parent_hist", colorfile, std::string(""));
    n.param("child_hist", childfile, std::string(""));
    n.param("ir_hist", irfile, std::string(""));

    hsv_img_ = cvCreateImage( cvSize( 320,240), 8, 3 );
    disp_ = cvCreateImage( cvSize( 320,240), 8, 3 );
    //foreground_img_ = cvCreateImage( cvSize( 320,240), 8, 1 );

    // colors
    color_finders_.clear();
    
    if( irfile != std::string("") )
    {
      ColorFinder c;
      c.init( irfile.c_str(), "ir" );
      color_finders_.push_back( c );
      colors_.push_back( CV_RGB( 0, 0, 255 ) );
    }

		if( childfile != std::string( "" ) )
    {
      ColorFinder p;
      p.init( childfile.c_str(), "child" );
      color_finders_.push_back( p );
      colors_.push_back( CV_RGB( 255, 128, 0 ) );
    }

    if( colorfile != std::string( "" ) )
    {
      ColorFinder p;
      p.init( colorfile.c_str(), "parent" );
      color_finders_.push_back( p );
      colors_.push_back( CV_RGB( 128, 255, 0 ) );
    }
    //std::string background_file;
    //n.param( "background", background_file, std::string("") );
    n.param( "display", display, 1 );
    if( display > 0 )
    {
      cvNamedWindow( "output", 1 );
			cvCreateTrackbar( "Vmin", "output", color_finders_[1].vmin(), 256, 0);
			cvCreateTrackbar( "Vmax", "output", color_finders_[1].vmax(), 256, 0);
			cvCreateTrackbar( "Smin", "output", color_finders_[1].smin(), 256, 0);
    }

    //kernel_ = cvCreateStructuringElementEx( 15, 15, 8, 8, CV_SHAPE_RECT );
    //storage_ = cvCreateMemStorage(0);

		first = true;
  }

  void cleanup()
  {
  }
};

int main( int argc, char* argv[] )
{
  ros::init(argc,argv,"color_blob_finder" );
  ImageSplitter* i = new ImageSplitter();
  boost::shared_ptr<ImageSplitter> foo_object(i);
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe("image_raw", 1, &ImageSplitter::image_cb, foo_object );
  i->init();
  ros::spin();
  ROS_INFO( "image_splitter quitting..." );
  i->cleanup();
  return 0;
}
