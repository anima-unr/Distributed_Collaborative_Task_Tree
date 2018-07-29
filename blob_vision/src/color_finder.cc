/*
 * color_finder
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
#include "blob_vision/color_finder.h"
#include "opencv2/core/core.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"

ColorFinder::ColorFinder()
{
  /*
  hranges_arr[0] = 0;
  hranges_arr[1] = 1;
  hranges = hranges_arr;
  */
}

void
ColorFinder::read_file()
{
  printf( "read file: [%s]\n", color_histfile_.c_str() );
  // create histogram; read in hdims
  FILE* hist_dump = fopen( color_histfile_.c_str(), "r" );
  fscanf( hist_dump, "%d\n", &hdims_ );
  fscanf( hist_dump, "%d\n", &smin_ );
  fscanf( hist_dump, "%d\n", &vmin_ );
  fscanf( hist_dump, "%d\n", &vmax_ );
  printf( "hist read: [%d,%d] [%d,%lu]\n", hist_.cols, hist_.rows, hist_.type(), hist_.elemSize() );

  float x = 0.0;

  for( int i = 0; i < hdims_; i++ )
  {
    fscanf( hist_dump, "%f\n", &x );
    printf( "%f\n", x);
    hist_.at<float>(i) = x;
  }
  cv::normalize(hist_, hist_, 0, 255, cv::NORM_MINMAX);
  fclose( hist_dump );
}

void 
ColorFinder::image_cb( cv::Mat hsv )
{
  float hranges_arr[] = {0,1};
  const float* hranges = hranges_arr;


  ROS_INFO("[%s]: image_cb", color_histfile_.c_str() );
  if( hsv.cols != mask_.cols )
  {
    hue_.create(hsv.size(), hsv.depth());
    //hist_.create(hdims_, 1, 5);
    mask_.create( hsv.size(), CV_8UC1 );
    backproject_img_.create( hsv.rows,hsv.cols, CV_8UC1 );

    cv::Rect selection;
    selection.x = 1;
    selection.y = 1;
    selection.width = 5;
    selection.height = 5;
    cv::inRange( hsv, cv::Scalar(0,smin_,MIN(vmin_,vmax_),0),
                 cv::Scalar(180,256,MAX(vmax_,vmin_),0),mask_);
    
    hue_.create(hsv.size(), hsv.depth());
    int ch[] = {0,0};
    cv::mixChannels( &hsv, 1, &hue_, 1, ch, 1);

    cv::Mat roi( hue_, selection ), maskroi(mask_, selection);
    printf( "about to read hist [%d]\n", hdims_);
    printf( "hist not read: [%d,%d] [%d,%lu]\n", hue_.cols, hue_.rows, hue_.type(), hue_.elemSize() );
    cv::calcHist( &roi, 1, 0, maskroi, hist_, 1, &hdims_, &hranges );
    read_file();
    cv::normalize(hist_, hist_, 0, 255, cv::NORM_MINMAX);
  }

  cv::inRange( hsv, cv::Scalar(0,smin_,MIN(vmin_,vmax_),0),
              cv::Scalar(180,256,MAX(vmax_,vmin_),0),mask_);
  
  hue_.create(hsv.size(), hsv.depth());
  int ch[] = {0,0};
  cv::mixChannels( &hsv, 1, &hue_, 1, ch, 1);

  cv::calcBackProject( &hue_, 1, 0, hist_, backproject_img_, &hranges );
  backproject_img_ &= mask_;

  int erosion_type = cv::MORPH_RECT;
  int erosion_size = 3;
  cv::Mat element = cv::getStructuringElement( erosion_type,
                                               cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                               cv::Point( erosion_size, erosion_size ) );


  cv::Mat dilate_dst;
  dilate_dst.create(hsv.size(), hsv.depth());
  cv::dilate( backproject_img_, dilate_dst, element );
  cv::Mat erode_dst;
  erode_dst.create(hsv.size(), hsv.depth());
  cv::erode( dilate_dst, erode_dst, element );

  element = cv::getStructuringElement( erosion_type,
                                               cv::Size( 2*erosion_size + 5, 2*erosion_size+5 ),
                                               cv::Point( erosion_size, erosion_size ) );

  cv::dilate( erode_dst, backproject_img_, element );
  //cvDilate( backproject_img_, backproject_img_, kernel_, 1 );

  cv::imshow( color_histfile_.c_str(), backproject_img_ );
  cv::waitKey(10);

}

void
ColorFinder::find_blobs( ros::Time t )
{
  ROS_INFO("[%s]: find_blobs", color_histfile_.c_str() );
  std::vector< std::vector<cv::Point> > contours;
  storage_.clear();
  cv::findContours( backproject_img_, contours, storage_, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );

  //blobs_.blobs.resize(0);

  // for each contour
  for( int i = 0; i < contours.size(); i++ )
  {
    // if above size threshold
    double area = fabs( cv::contourArea( contours[i] ) );
    if( area > min_area_ )
    {
      //bounding box
      cv::Rect bb = cv::boundingRect( contours[i] );
/*
      oit_msgs::Blob b;
      b.x = bb.x;
      b.y = bb.y;
      b.width = bb.width;
      b.height = bb.height;
      b.size = area;

      // add to blob list
      blobs_.blobs.push_back( b );
*/
      printf( "(%d,%d) [%d, %d]: %0.2f\n", bb.x, bb.y, bb.width, bb.height, area);
    }
  }
  //blobs_.header.stamp = t;
  //blobs_pub.publish( blobs_ );
}

void 
ColorFinder::init( std::string color_file, std::string name, int min_area )
{

  color_histfile_ = color_file;
  min_area_ = min_area;
  printf( "filename: [%s]\n", color_histfile_.c_str() );
  FILE* hist_dump = fopen( color_file.c_str(), "r" );
  fscanf( hist_dump, "%d\n", &hdims_ );
  fscanf( hist_dump, "%d\n", &smin_ );
  fscanf( hist_dump, "%d\n", &vmin_ );
  fscanf( hist_dump, "%d\n", &vmax_ );
  fclose( hist_dump);
  //storage_ = cvCreateMemStorage(0);
  //mask_ = cvCreateImage( cvSize(1,1),8,1);

  // read in kernel size
  int kernel_size = 5;

  //kernel_ = cvCreateStructuringElementEx( kernel_size, kernel_size, 
  //                                       kernel_size/2, kernel_size/2, 
  //                                       CV_SHAPE_RECT );

  //cvNamedWindow( color_histfile_.c_str(), 1 );

  // declare publisher
  ros::NodeHandle n;
  std::string pub_name = name + "_blobs";
  std::string world_name = name + "_world";
  //blobs_pub = n.advertise<oit_msgs::BlobArray>( pub_name, 1000 );
}

