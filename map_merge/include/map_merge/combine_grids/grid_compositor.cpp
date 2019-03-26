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
#include <combine_grids/grid_compositor.h>

#include <opencv2/stitching/detail/util.hpp>

#include <ros/assert.h>

namespace combine_grids
{
namespace internal
{
nav_msgs::OccupancyGrid::Ptr GridCompositor::compose(
    const std::vector<cv::Mat>& grids, const std::vector<cv::Rect>& rois)
{
  ROS_ASSERT(grids.size() == rois.size());

  nav_msgs::OccupancyGrid::Ptr result_grid(new nav_msgs::OccupancyGrid());

  std::vector<cv::Point> corners;
  corners.reserve(grids.size());
  std::vector<cv::Size> sizes;
  sizes.reserve(grids.size());

  for (auto& roi : rois) {

    corners.push_back(roi.tl());
	//std::cout<<roi.tl()<<std::endl;
    sizes.push_back(roi.size());
	//std::cout<<roi.size()<<std::endl;
  }
  cv::Rect dst_roi = cv::detail::resultRoi(corners, sizes);

  result_grid->info.width = static_cast<uint>(dst_roi.width);
  result_grid->info.height = static_cast<uint>(dst_roi.height);
  result_grid->data.resize(static_cast<size_t>(dst_roi.area()), -1);
  // create view for opencv pointing to newly allocated grid
  cv::Mat result(dst_roi.size(), CV_8S, result_grid->data.data());

  
  //cartographer融合两个地图方法：
  // 1.P = P1*P2/(1-P1-P2+2*P1*P2)
  // 2.若P1、P2中有负的，则100*P<0
  // 3.令100*P小于0的置为0 (能否置为 -1 ?)
  // 4.恢复到 OccupancyGrid 中。
  for (size_t i = 0; i < grids.size(); ++i) {								
    // we need to compensate global offset
    cv::Rect roi = cv::Rect(corners[i] - dst_roi.tl(), sizes[i]);
    cv::Mat result_roi(result, roi);								//last global grid  : result_roi
    // reinterpret warped matrix as signed
    // we will not change this matrix, but opencv does not support const matrices
    //cv::Mat warped_signed (grids[i].size(), CV_8S, const_cast<uchar*>(grids[i].ptr()));		//current input grid : warped_signed
    // compose img into result matrix
    
    for (int k=0;k<result_roi.size().height;k++)
    {
      signed char* indata10 = result_roi.ptr<signed char>(k);
      const signed char* indata20 = grids[i].ptr<signed char>(k);
      for (int j=0;j<result_roi.size().width;j++)
      {
	if (indata10[j]>=0 && indata20[j]>=0)
	  {
	    float p1 = (float)indata10[j]/100;
	    float p2 = (float)indata20[j]/100;
	    
	    double medi = (p1)*(p2)/(1-p1-p2+2*p1*p2);
	    indata10[j] = (int)(medi*100);
	    continue;
	  }
	  if (indata10[j]==-1 && indata20[j]==-1)
	  {
	    indata10[j]=-1;
	    continue;
	  }
	   if (indata10[j]>=0 && indata20[j]==-1)
	  {
	   continue;    
	  }
	  if (indata10[j]==-1 && indata20[j]>=0)
	  {
	    indata10[j] = indata20[j];
	  }
      }     
    }
  }

  return result_grid;
}

}  // namespace internal

}  // namespace combine_grids
