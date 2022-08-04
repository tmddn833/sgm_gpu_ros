/***********************************************************************
  Copyright (C) 2020 Hironori Fujimoto

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
***********************************************************************/

#include "sgm_gpu_node.h"
#include <message_filters/synchronizer.h>

namespace sgm_gpu {

    SgmGpuNode::SgmGpuNode() {
        node_handle_.reset(new ros::NodeHandle());
        private_node_handle_.reset(new ros::NodeHandle("~"));
        private_node_handle_->getParam("rgb_fov", rgb_fov_deg_);
        private_node_handle_->getParam("base_line",stereo_baseline_);
        private_node_handle_->getParam("use_gt", use_gt_);

        image_transport_.reset(new image_transport::ImageTransport(*node_handle_));
        sgm_.reset(new SgmGpu(*private_node_handle_));

        disparity_pub_ = private_node_handle_->advertise<sensor_msgs::Image>("disparity", 1);


        // Subscribe left and right Image topic or GT depth
        if (!use_gt_){
            std::string left_base_topic = node_handle_->resolveName("left_image");
            std::string right_base_topic = node_handle_->resolveName("right_image");
            left_image_sub_.subscribe(*image_transport_, left_base_topic, 10);
            right_image_sub_.subscribe(*image_transport_, right_base_topic, 10);
            stereo_synchronizer_.reset(
                    new StereoSynchronizer(StereoSynchronize_policies(10), left_image_sub_, right_image_sub_)
            );
            ROS_INFO("[SGM NODE]sgm started");
            stereo_synchronizer_->registerCallback(&SgmGpuNode::stereoCallback, this);
        }
        else{
            std::string gt_depth_topic = node_handle_->resolveName("gt_depth_image");
            gt_depth_sub_ = image_transport_->subscribe(gt_depth_topic, 10, &SgmGpuNode::gtDepthCallback, this);
            ROS_INFO("[SGM NODE] use GT depth instead sgm");
        }
    }

    void SgmGpuNode::stereoCallback(
            const sensor_msgs::ImageConstPtr &left_image,
            const sensor_msgs::ImageConstPtr &right_image
    ) {
//        if (disparity_pub_.getNumSubscribers() == 0) {
//            ROS_INFO("getNumSubscribers");
//            return;
//        }

//        ROS_INFO("%f",left_image->header.stamp.toSec()-right_image->header.stamp.toSec());
        cv_bridge::CvImageConstPtr left_cv_ptr;
        cv_bridge::CvImageConstPtr right_cv_ptr;
        try
        {
            left_cv_ptr = cv_bridge::toCvShare(left_image, sensor_msgs::image_encodings::RGB8);
            right_cv_ptr = cv_bridge::toCvShare(right_image, sensor_msgs::image_encodings::RGB8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        int img_rows_ = left_image->height;
        int img_cols_ = left_image->width;

        // compute disparity image
        cv::Mat depth_uint16(img_rows_, img_cols_, CV_16UC1);
        cv::Mat disparity(img_rows_, img_cols_, CV_8UC1);
        sgm_->computeDisparity(left_cv_ptr->image, right_cv_ptr->image, &disparity);
        disparity.convertTo(disparity, CV_32FC1);


        float f = (img_cols_ / 2.0) / std::tan((M_PI * (rgb_fov_deg_ / 180.0)) / 2.0);
        //  depth = static_cast<float>(stereo_baseline_) * f / disparity;
        for (int r = 0; r < img_rows_; ++r) {
            for (int c = 0; c < img_cols_; ++c) {
                if (disparity.at<float>(r, c) == 0.0f) {
//                    depth_float.at<float>(r, c) = 0.0f;
                    depth_uint16.at<unsigned short>(r, c) = 0;
                } else if (disparity.at<float>(r, c) == 255.0f) {
//                    depth_float.at<float>(r, c) = 0.0f;
                    depth_uint16.at<unsigned short>(r, c) = 255;
                } else {
//                    depth_float.at<float>(r, c) = static_cast<float>(stereo_baseline_) * f /
//                                                  disparity.at<float>(r, c);
                    depth_uint16.at<unsigned short>(r, c) = static_cast<unsigned short>(
                            1000.0 * static_cast<float>(stereo_baseline_) * f /
                            disparity.at<float>(r, c));
                }
            }
        }

        sensor_msgs::ImagePtr sgm_depth_msg =
                cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_uint16).toImageMsg();
        sgm_depth_msg->header = left_image->header;
        disparity_pub_.publish(sgm_depth_msg);
    }


    void SgmGpuNode::gtDepthCallback(
            const sensor_msgs::ImageConstPtr &gt_depth
    ) {
        cv_bridge::CvImageConstPtr depth_cv_ptr;
        try
        {
            depth_cv_ptr = cv_bridge::toCvShare(gt_depth, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        int img_rows_ = gt_depth->height;
        int img_cols_ = gt_depth->width;

        cv::Mat depth_uint16(img_rows_, img_cols_, CV_16UC1);
        cv::Mat gt_depth_mat(img_rows_, img_cols_, CV_32FC1);
        gt_depth_mat = depth_cv_ptr->image;
        gt_depth_mat.convertTo(depth_uint16,CV_16UC1);
        for (int r = 0; r < img_rows_; ++r) {
            for (int c = 0; c < img_cols_; ++c) {
                depth_uint16.at<unsigned short>(r, c) = depth_uint16.at<unsigned short>(r, c)*2;
            }
        }

        sensor_msgs::ImagePtr sgm_depth_msg =
                cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_uint16).toImageMsg();
        sgm_depth_msg->header = gt_depth->header;
        disparity_pub_.publish(sgm_depth_msg);
    }

} // namespace sgm_gpu
