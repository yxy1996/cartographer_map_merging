/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Zhi Yan.
 *  Copyright (c) 2015-2016, Jiri Horner.
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
 *   * Neither the name of the Jiri Horner nor the names of its
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
#ifndef MERGING_PIPELINE_H_
#define MERGING_PIPELINE_H_

#include <vector>

#include <geometry_msgs/Transform.h>
#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/core/utility.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace combine_grids
{
enum class FeatureType { AKAZE, ORB, SURF };

/**
 * @brief Pipeline for merging overlapping occupancy grids
 * @details Pipeline works on internally stored grids.
 */
class MergingPipeline
{
public:
  template <typename InputIt>
  void feed(InputIt grids_begin, InputIt grids_end);
  nav_msgs::OccupancyGrid::Ptr composeGrids();

  std::vector<geometry_msgs::Transform> getTransforms() const;
  template <typename InputIt>
  bool setTransforms(InputIt transforms_begin, InputIt transforms_end);  
private:
  std::vector<nav_msgs::OccupancyGrid::ConstPtr> grids_;
  std::vector<cv::Mat> images_;
  std::vector<cv::Mat> transforms_;
  
  std::vector<double> origin_x_;
  std::vector<double> origin_y_;
  std::vector<double> resolution_;
};

template <typename InputIt>
void MergingPipeline::feed(InputIt grids_begin, InputIt grids_end)
{
  static_assert(std::is_assignable<nav_msgs::OccupancyGrid::ConstPtr&,
                                   decltype(*grids_begin)>::value,
                "grids_begin must point to nav_msgs::OccupancyGrid::ConstPtr "
                "data");

  // we can't reserve anything, because we want to support just InputIt and
  // their guarantee validity for only single-pass algos
  images_.clear();
  grids_.clear();
  origin_x_.clear();
  origin_y_.clear();	
  resolution_.clear();//big ++
//to do 
   std::vector<cv::Mat> images_fake;
   

  for (InputIt it = grids_begin; it != grids_end; ++it) {
    if (*it && !(*it)->data.empty()) {
      grids_.push_back(*it);
      /* convert to opencv images. it creates only a view for opencv and does
       * not copy or own actual data. */
      images_fake.emplace_back((*it)->info.height, (*it)->info.width, CV_8UC1,
                           const_cast<signed char*>((*it)->data.data()));
      resolution_.emplace_back((*it)->info.resolution);
      origin_x_.emplace_back(((*it)->info.origin.position.x)/(*it)->info.resolution);
      origin_y_.emplace_back(((*it)->info.origin.position.y)/(*it)->info.resolution);
      
    //  std::cout<<(*it)->header.frame_id<<std::endl;
    //  std::cout<<origin_y_.back()<<std::endl;
    //  std::cout<<origin_y_.size()<<std::endl;
    } else {
      grids_.emplace_back();
      images_fake.emplace_back();
      origin_x_.emplace_back();
      origin_y_.emplace_back();
      resolution_.emplace_back();
    }
  }
  for (int i=0;i<images_fake.size();i++){
	if(!images_fake[i].empty()){

     cv::Mat img(1000,1000,CV_8UC1);
     cv::Mat imgROI;
     img.setTo(255);	
     imgROI = img(cv::Rect(img.cols/2 + origin_x_[i],img.rows/2 + origin_y_[i] ,images_fake[i].cols,images_fake[i].rows));
     images_fake[i].copyTo(imgROI);  
     images_.emplace_back(img);
}
}
//to do
}

template <typename InputIt>
bool MergingPipeline::setTransforms(InputIt transforms_begin,
                                    InputIt transforms_end)
{
  static_assert(std::is_assignable<geometry_msgs::Transform&,
                                   decltype(*transforms_begin)>::value,
                "transforms_begin must point to geometry_msgs::Transform "
                "data");

  decltype(transforms_) transforms_buf;
  int i=0;
  for (InputIt it = transforms_begin; it != transforms_end; ++it) {
    const geometry_msgs::Quaternion& q = it->rotation;
    if ((q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) <
        std::numeric_limits<double>::epsilon()) {
      // represents invalid transform
      transforms_buf.emplace_back();
      continue;
    }
    double s = 2.0 / (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    double a = 1 - q.y * q.y * s - q.z * q.z * s;
    double b = q.x * q.y * s + q.z * q.w * s;
    double tx = it->translation.x;
    double ty = it->translation.y;
    cv::Mat transform = cv::Mat::eye(3, 3, CV_64F);
    transform.at<double>(0, 0) = transform.at<double>(1, 1) = a;
    transform.at<double>(1, 0) = b;
    transform.at<double>(0, 1) = -b;
    transform.at<double>(0, 2) = tx / resolution_[i];					//translation in grids
    transform.at<double>(1, 2) = ty / resolution_[i];

    transforms_buf.emplace_back(std::move(transform));
    i++;
  }

  if (transforms_buf.size() != images_.size()) {
    return false;
  }
  std::swap(transforms_, transforms_buf);

  return true;
}

}  // namespace combine_grids

#endif  // MERGING_PIPELINE_H_
