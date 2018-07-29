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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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
//#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/videoio/videoio.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>

#include <unistd.h>
#include <ctype.h>

cv::Mat image, hsv, hue, mask, backproject, hist, histimg = cv::Mat::zeros( 240, 360, CV_8UC3 );
//cv::Histogram *hist = 0;
int backproject_mode = 0;
int select_object = 0;
int color_selected = 0;
int show_hist = 1;
cv::Point origin;
cv::Rect selection;
cv::Rect track_window;
cv::RotatedRect track_box;
int hdims = 60;
float hranges_arr[] = {0,180};
const float* hranges = hranges_arr;
int vmin = 10, vmax = 256, smin = 30;

std::string prefix;
void image_cb( const sensor_msgs::ImageConstPtr& image_msg );


void process_key( int key )
{
  FILE* hist_out;
  char name[1024];
  name[0] = 0;
  //prefix[0] = 0;
  time_t t = ::time(NULL);
  struct tm *tms = localtime(&t);


  switch( (char) key )
  {
    case 'b':
      backproject_mode ^= 1;
      break;
    case 'c':
      color_selected = 0;
      histimg = cv::Scalar::all(0);
      break;
    case 'h':
       show_hist ^= 1;
       if( !show_hist )
           cv::destroyWindow( "Histogram" );
       else
           cv::namedWindow( "Histogram", 1 );
       break;
    case 's':
      if( prefix != "" )
      {
        snprintf(name, sizeof(name), "%s/%d-%02d-%02d-%02d-%02d-%02d-dump.txt",
                 prefix.c_str(),
                 tms->tm_year+1900, tms->tm_mon+1, tms->tm_mday,
                 tms->tm_hour     , tms->tm_min  , tms->tm_sec);
      }
      else 
      {
        snprintf(name, sizeof(name), "%d-%02d-%02d-%02d-%02d-%02d-dump.txt",
                 tms->tm_year+1900, tms->tm_mon+1, tms->tm_mday,
                 tms->tm_hour     , tms->tm_min  , tms->tm_sec);
      }
      ROS_INFO( "saving color: [%s]", name );
      hist_out = fopen( name, "w" );
      fprintf( hist_out, "%d\n%d\n%d\n%d\n", hdims, smin, vmin, vmax );
      for( int i = 0; i < hdims; i++ )
        fprintf( hist_out, "%f\n", hist.at<float>(i));
      fclose( hist_out );
      break;
    default:
       ;
  }
}

void on_mouse( int event, int x, int y, int flags, void* param )
{
    if( select_object )
    {
        selection.x = MIN(x,origin.x);
        selection.y = MIN(y,origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);
        
        selection &= cv::Rect(0,0,image.cols,image.rows);
    }

    switch( event )
    {
    case cv::EVENT_LBUTTONDOWN:
        origin = cv::Point(x,y);
        selection = cv::Rect(x,y,0,0);
        select_object = 1;
        break;
    case cv::EVENT_LBUTTONUP:
        select_object = 0;
        if( selection.width > 0 && selection.height > 0 )
            color_selected = -1;
        break;
    }
}

/*
CvScalar hsv2rgb( float hue )
{
    int rgb[3], p, sector;
    static const int sector_data[][3]=
        {{0,2,1}, {1,2,0}, {1,0,2}, {2,0,1}, {2,1,0}, {0,1,2}};
    hue *= 0.033333333333333333333333333333333f;
    sector = cv::Floor(hue);
    p = cv::Round(255*(hue - sector));
    p ^= sector & 1 ? 255 : 0;

    rgb[sector_data[sector][0]] = 255;
    rgb[sector_data[sector][1]] = 0;
    rgb[sector_data[sector][2]] = p;

    return cvScalar(rgb[2], rgb[1], rgb[0],0);
}
*/

void image_cb( const sensor_msgs::ImageConstPtr& img_msg )
{
  int i, bin_w;
  cv_bridge::CvImagePtr cv_ptr;
	/* get frame */
  try {
    cv_ptr = cv_bridge::toCvCopy( img_msg, sensor_msgs::image_encodings::BGR8 );
  }
  catch( cv_bridge::Exception &e ) 
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  cv::Mat frame = cv_ptr->image;

	/* allocate all the buffers */
	  
  image = frame.clone();
  hsv	= cv::Mat::zeros( frame.rows, frame.cols, CV_8UC3 );
	hue = cv::Mat::zeros( frame.rows, frame.cols, CV_8UC3 );
	mask = cv::Mat::zeros( frame.rows, frame.cols, CV_8UC3 );
	backproject = cv::Mat::zeros( frame.rows, frame.cols, CV_8UC3 );
	
  cv::cvtColor( image, hsv, CV_BGR2HSV );

  if( color_selected )
  {
	  int _vmin = vmin, _vmax = vmax;
    cv::inRange( hsv, cv::Scalar(0,smin,MIN(_vmin,_vmax),0),
                cv::Scalar(180,256,MAX(_vmin,_vmax),0), mask );
    hue.create(hsv.size(), hsv.depth());
    int ch[] = {0,0};
    cv::mixChannels( &hsv, 1, &hue, 1, ch, 1 );

    if( color_selected < 0 )
    {    
	  	float max_val = 0.f;
      cv::Mat roi( hue, selection ), maskroi(mask, selection);
      cv::calcHist( &roi, 1, 0, maskroi, hist, 1, &hdims, &hranges );
      cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX);
      
      track_window = selection;
      color_selected = 1;

      histimg = cv::Scalar::all(0);
      bin_w = histimg.cols / hdims;
      cv::Mat buf(1, hdims, CV_8UC3);
      for( int i = 0; i < hdims; i++ )
        buf.at<cv::Vec3b>(i) = cv::Vec3b(cv::saturate_cast<uchar>(i*180./hdims), 255, 255);
      cv::cvtColor(buf, buf, cv::COLOR_HSV2BGR);
      for( int i = 0; i < hdims; i++ )
      {
 	    	int val = cv::saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
        cv::rectangle( histimg, cv::Point(i*bin_w,histimg.rows),
                    cv::Point((i+1)*bin_w,histimg.rows - val),
                    cv::Scalar(buf.at<cv::Vec3b>(i)), -1, 8 );

        /*
        int val = cvRound( cvGetReal1D(hist->bins,i)*histimg->height/255 );
        CvScalar color = hsv2rgb(i*180.f/hdims);
        cvRectangle( histimg, cvPoint(i*bin_w,histimg->height),
                     cvPoint((i+1)*bin_w,histimg->height - val),
                     color, -1, 8, 0 );
        */
      }
		  printf( "hist dump: [%d,%d] [%d,%d]\n", hist.cols, hist.rows, hist.type(), hist.elemSize() );
    }

    cv::calcBackProject( &hue, 1, 0, hist, backproject, &hranges );
    backproject &= mask;
    track_box = cv::CamShift( backproject, track_window,
    						cv::TermCriteria( cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1 ) );
    //track_window = track_comp.rect;
    /*if( !image->origin )
      track_box.angle = -track_box.angle;*/
    if( track_window.area() <=1 )
    {
      int cols = backproject.cols, rows = backproject.rows, r = (MIN(cols,rows) + 5)/6;
      track_window = cv::Rect(track_window.x - r, track_window.y - r,
                              track_window.x + r, track_window.y + r) &
                     cv::Rect(0,0,cols,rows);
    }
    //printf( "(%f,%f) %f:%f\n", track_box.center.x, track_box.center.y, track_box.size.width, track_box.size.height );
    if( backproject_mode )
      cv::cvtColor( backproject, image, cv::COLOR_GRAY2BGR );
    cv::ellipse( image, track_box, cv::Scalar(255,0,0), 1, CV_AA );
  }
        
  if( select_object && selection.width > 0 && selection.height > 0 )
  {
  	cv::Mat roi( image, selection );
    cv::bitwise_not( roi, roi );
  }

	cv::imshow( "CamShiftDemo", image );
  cv::imshow( "Histogram", histimg );
		
}

int main( int argc, char** argv )
{
  ros::init(argc,argv,"color_picker");
  ros::NodeHandle n;
  ros::NodeHandle n_priv("~");
  n_priv.param( "prefix", prefix, std::string(""));
  ros::Rate loop_rate(10);
  ros::Subscriber image_sub = n.subscribe("image",1,image_cb);
  printf( "Hot keys: \n"
      "\tESC - quit the program\n"
      "\tc - stop the tracking\n"
      "\tb - switch to/from backprojection view\n"
      "\th - show/hide object histogram\n"
      "\ts - save current color to filename\n"
      "To initialize tracking, select the object with mouse\n" );

  /* create windows */
  cvNamedWindow( "Histogram", 1 );
  cvNamedWindow( "CamShiftDemo", 1 );
  cvSetMouseCallback( "CamShiftDemo", on_mouse, 0 );
  cvCreateTrackbar( "Vmin", "CamShiftDemo", &vmin, 256, 0 );
  cvCreateTrackbar( "Vmax", "CamShiftDemo", &vmax, 256, 0 );
  cvCreateTrackbar( "Smin", "CamShiftDemo", &smin, 256, 0 );
            

  while( ros::ok() )
  {

    int c = cvWaitKey(10);
    process_key(c);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
} 

