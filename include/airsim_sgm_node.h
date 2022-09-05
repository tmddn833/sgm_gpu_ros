//
// Created by seungwooubuntu on 22. 8. 17..
//

#ifndef SGM_GPU_ROS_AIRSIM_SGM_NODE_H
#define SGM_GPU_ROS_AIRSIM_SGM_NODE_H
#include <stdio.h>

#include "sgm_gpu/sgm_gpu.h"

#include <image_transport/camera_common.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include "common/Common.hpp"
#include "common/common_utils/ProsumerQueue.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "common/ClockFactory.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "api/VehicleApiBase.hpp"
#include <opencv2/stereo.hpp>


namespace airsim_sgm_node{
    class AirsimSgmNode {
    private:
        std::shared_ptr<ros::NodeHandle> node_handle_;
        std::shared_ptr<ros::NodeHandle> private_node_handle_;
        std::shared_ptr<image_transport::ImageTransport> image_transport_;
        std::shared_ptr<sgm_gpu::SgmGpu> sgm_;
        cv::Ptr<cv::StereoSGBM> cv_sgm_;
        std::shared_ptr<msr::airlib::RpcLibClientBase> airsim_client_images_;
        std::shared_ptr<msr::airlib::ClockBase> clock_;

        image_transport::Publisher right_image_pub_;
        image_transport::Publisher left_image_pub_;
        image_transport::Publisher disparity_pub_;
        image_transport::Publisher cv2_disparity_pub_;



        typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::Image, sensor_msgs::Image> StereoSynchronize_policies;
        using StereoSynchronizer = message_filters::Synchronizer<StereoSynchronize_policies>;
        std::shared_ptr<StereoSynchronizer> stereo_synchronizer_;
        ros::Timer timer_;

        typedef boost::shared_ptr<cv_bridge::CvImage> CvImagePtr;
        typedef boost::shared_ptr<cv_bridge::CvImage const> CvImageConstPtr;
        typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
        typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
        typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;

        void timerCallback(const ros::TimerEvent&);
        void printCameraInfo(void);
        ros::Time airsim_timestamp_to_ros(const msr::airlib::TTimePoint& stamp) const;
        ros::Time chrono_timestamp_to_ros(const std::chrono::system_clock::time_point& stamp) const;


    public:
        AirsimSgmNode();
        float rgb_fov_deg_;
        float stereo_baseline_;
        float sgm_period_;
        bool USE_CV_SGM = true;

    };

}
#endif //SGM_GPU_ROS_AIRSIM_SGM_NODE_H
