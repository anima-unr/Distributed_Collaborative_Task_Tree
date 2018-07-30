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


#ifndef _COLOR_FINDER_H_
#define _COLOR_FINDER_H_

#include "ros/ros.h"
#include "opencv2/core/core.hpp"

class Blob 
{
  public:
    int x, y, width, height, size;
};

class ColorFinder
{
  ros::Publisher blobs_pub;
  ros::Publisher world_pub;
  cv::Mat backproject_img_, mask_, hue_, histimg_ = cv::Mat::zeros( 240, 360, CV_8UC3 );

  std::vector<Blob> blobs_;
  //oit_msgs::BlobArray blobs_;

  // color histogram settings
  std::string color_histfile_;
  int smin_, vmin_, vmax_, hdims_;
  int min_area_;
  cv::Mat hist_;
  std::vector<cv::Vec4i> storage_;

  public:
    ColorFinder();
    void read_file();
    void init( std::string, std::string, int min_area = 10 );
    void image_cb( cv::Mat img );
    void find_blobs(ros::Time t);

		int* smin() {return &smin_;}
		int* vmax() {return &vmax_;}
		int* vmin() {return &vmin_;}

    std::vector<Blob> get_blobs() { return blobs_; }
    
};

#endif
