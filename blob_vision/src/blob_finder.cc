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
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/fill_image.h"
#include "vision_manip_pipeline/GetObjLoc.h"

#include "blob_vision/color_finder.h"

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
    vector<string> color_hist_files_;
    vector<string> color_names_;
    map<string,Blob> current_location_;
    //int vmin, vmax, smin;
  public:

  bool blob_lookup( vision_manip_pipeline::GetObjLoc::Request  &req,
                    vision_manip_pipeline::GetObjLoc::Response &res)
  {
    Blob b = current_location_[req.obj_name];
    res.Class = req.obj_name;
    res.probability = 1;
    res.xmin = b.x;
    res.ymin = b.y;
    res.xmax = b.x+b.width;
    res.ymax = b.y+b.height;
    return true;
  }

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
    for( int i = 0; i < color_finders_.size(); i++ )
    {
      // color probability images
      color_finders_[i].image_cb( hsv_img_ );
      // find contours
      color_finders_[i].find_blobs(msg->header.stamp);
      // publish blobs

      // render blobs
      vector<Blob> blobs = color_finders_[i].get_blobs();
      int idx = -1;
      int ma = 0;
      for( int j = 0; j < blobs.size(); j++ )
      {
        //printf( "blobs[%u].size = %d <> %d (%u)\n", j, blobs[j].size, ma, idx );
        if( blobs[j].size > ma )
        {
          idx = j;
          ma = blobs[j].size;
        }
      }      

      if( idx > -1 )
      {
        Blob b = blobs[idx];
        cv::rectangle( disp_, cv::Point( b.x, b.y ), 
                              cv::Point( b.x+b.width, b.y+b.height ),
                              colors_[i], 1 );
        current_location_[color_names_[i]] = b;
      }

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
    n = ros::NodeHandle("~");
    hsv_img_ = cvCreateImage( cvSize( 320,240), 8, 3 );
    disp_ = cvCreateImage( cvSize( 320,240), 8, 3 );
    std::string colordir;
    n.param("color_dir", colordir, std::string(""));

    color_hist_files_.clear();
    n.getParam("histfiles", color_hist_files_);
    color_names_.clear();
    n.getParam("color_names", color_names_);

    colors_.push_back( CV_RGB( 128, 0, 255 ) );
    colors_.push_back( CV_RGB( 255, 128, 0 ) );
    colors_.push_back( CV_RGB( 0, 128, 0 ) );
    colors_.push_back( CV_RGB( 0, 0, 255 ) );
    colors_.push_back( CV_RGB( 255, 0, 0 ) );
    colors_.push_back( CV_RGB( 128, 255, 0 ) );

    for( int i = 0; i < color_hist_files_.size(); i++ )
    {
      std::string colorfilename = colordir + std::string("/") + color_hist_files_[i];
      ROS_DEBUG( "creating color finder: [%s:%s]", color_names_[i].c_str(), colorfilename.c_str() );
      ColorFinder p;
      if( p.init( colorfilename.c_str(), color_hist_files_[i] ) == 0 )
        color_finders_.push_back( p );
    }

    n.param( "display", display, 1 );
    if( display > 0 )
    {
      cvNamedWindow( "output", 1 );
    }

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
  ros::ServiceServer service = nh.advertiseService("return_blobs", &ImageSplitter::blob_lookup, foo_object);

  i->init();
  ros::spin();
  ROS_INFO( "image_splitter quitting..." );
  i->cleanup();
  return 0;
}
