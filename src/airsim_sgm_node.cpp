//
// Created by seungwooubuntu on 22. 8. 17..
//

#include "airsim_sgm_node.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
//#include <opencv2/stereo.hpp>

// Defining callback functions for mouse events
void mouseEvent(int evt, int x, int y, int flags, void *param) {
    cv::Mat *depth = (cv::Mat *) param;
    if (evt == cv::EVENT_LBUTTONDOWN) {
        printf("depth is %d at %d, %d \n",
               (int) (*depth).at<unsigned short>(y, x),
               x, y);
    }
}

namespace airsim_sgm_node {
    AirsimSgmNode::AirsimSgmNode() {
        node_handle_.reset(new ros::NodeHandle());
        private_node_handle_.reset(new ros::NodeHandle("~"));
        private_node_handle_->getParam("rgb_fov", rgb_fov_deg_);
        private_node_handle_->getParam("base_line", stereo_baseline_);
        private_node_handle_->getParam("sgm_period", sgm_period_);
        private_node_handle_->getParam("use_cv_sgm", USE_CV_SGM);

        std::string host_ip = "localhost";
        private_node_handle_->getParam("host_ip", host_ip);

        image_transport_.reset(new image_transport::ImageTransport(*node_handle_));
        sgm_.reset(new sgm_gpu::SgmGpu(*private_node_handle_));

        cv_sgm_ = cv::StereoSGBM::create();
        cv_sgm_->setP1(6);
        cv_sgm_->setP2(96);
//        cv_sgm_->setP1(0.0);
//        cv_sgm_->setP2(0.0);
        cv_sgm_->setMinDisparity(0);
//        cv_sgm_->setNumDisparities(64);
        cv_sgm_->setBlockSize(15);
        cv_sgm_->setPreFilterCap(1);
        cv_sgm_->setUniquenessRatio(30);
        cv_sgm_->setSpeckleWindowSize(100);
        cv_sgm_->setSpeckleRange(32);
        cv_sgm_->setDisp12MaxDiff(1);
//        cv_sgm_->setMode(cv::StereoSGBM::MODE_HH);

        cv::namedWindow("depth", cv::WINDOW_NORMAL);
        cv::resizeWindow("depth", 600, 600);


        disparity_pub_ = image_transport_->advertise("cuda_depth", 1);
        cv2_disparity_pub_ = image_transport_->advertise("cv2_depth", 1);
        right_image_pub_ = image_transport_->advertise("right_image", 1);
        left_image_pub_ = image_transport_->advertise("left_image", 1);


        timer_ = node_handle_->createTimer(ros::Duration(sgm_period_), &AirsimSgmNode::timerCallback, this);
        airsim_client_images_.reset(new msr::airlib::RpcLibClientBase);
        airsim_client_images_->confirmConnection();

        clock_.reset(msr::airlib::ClockFactory::get());

        ROS_INFO("[SGM NODE]sgm started");
        printCameraInfo();
    }

    void AirsimSgmNode::printCameraInfo() {
        std::vector<std::string> camera_names(2);
        camera_names.emplace_back("left");
        camera_names.emplace_back("right");
        for (int i = 0; i < camera_names.size(); i++) {
            msr::airlib::CameraInfo camera_info_i = airsim_client_images_->simGetCameraInfo(camera_names.at(i));
            float pos_x = camera_info_i.pose.position.x();
            float pos_y = camera_info_i.pose.position.y();
            float pos_z = camera_info_i.pose.position.z();
            std::cout << "Camera #" << camera_names.at(i) << "'s pose : x " << pos_x << " y " << pos_y << " z " << pos_z
                      << std::endl;
        }
    }

    void AirsimSgmNode::timerCallback(const ros::TimerEvent &) {
        std::vector<ImageRequest> request = {
                ImageRequest("right", ImageType::Scene, false, false),
                ImageRequest("left", ImageType::Scene, false, false)
        };
        const std::vector<ImageResponse> &response = airsim_client_images_->simGetImages(request);
//        ros::Duration(0.1).sleep();
        if (response.size() != 2) {
            std::cout << "Images were not received!" << response.at(0).image_data_uint8.size() << std::endl;
            return;
        } else if (int(response.at(1).time_stamp - response.at(0).time_stamp) > 3000064) {
            std::cout << "Image time stamp not synced : time diff "
                      << response.at(1).time_stamp - response.at(0).time_stamp
                      << "  data len : " << response.at(1).image_data_uint8.size() << std::endl;
            return;
        } else {
            sensor_msgs::ImagePtr left_img_msg_ptr = boost::make_shared<sensor_msgs::Image>();
            sensor_msgs::ImagePtr right_img_msg_ptr = boost::make_shared<sensor_msgs::Image>();

            right_img_msg_ptr->data = response.at(0).image_data_uint8;
            left_img_msg_ptr->data = response.at(1).image_data_uint8;

            left_img_msg_ptr->step = right_img_msg_ptr->step =
                    response.at(1).width * 3;
            left_img_msg_ptr->header.stamp = right_img_msg_ptr->header.stamp = airsim_timestamp_to_ros(
                    response.at(1).time_stamp);

            right_img_msg_ptr->header.frame_id = response.at(0).camera_name;
            left_img_msg_ptr->header.frame_id = response.at(1).camera_name;

            left_img_msg_ptr->height = right_img_msg_ptr->height = response.at(1).height;
            left_img_msg_ptr->width = right_img_msg_ptr->width = response.at(1).width;
            left_img_msg_ptr->encoding = right_img_msg_ptr->encoding = "bgr8";

            right_image_pub_.publish(right_img_msg_ptr);
            left_image_pub_.publish(left_img_msg_ptr);
//            std::cout << "Image published  data len : " << response.at(1).image_data_uint8.size() << std::endl;

            /////RGB image check?////

            cv_bridge::CvImageConstPtr left_cv_ptr;
            cv_bridge::CvImageConstPtr right_cv_ptr;
            try {
                left_cv_ptr = cv_bridge::toCvShare(left_img_msg_ptr, sensor_msgs::image_encodings::BGR8);
                right_cv_ptr = cv_bridge::toCvShare(right_img_msg_ptr, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            cv::Mat left_mono8, right_mono8;
            if (left_cv_ptr->image.channels() > 1) {
                cv::cvtColor(left_cv_ptr->image, left_mono8, CV_BGR2GRAY);
            }
            if (right_cv_ptr->image.channels() > 1) {
                cv::cvtColor(right_cv_ptr->image, right_mono8, CV_BGR2GRAY);
            }
            double min_d, max_d;
            cv::minMaxLoc(left_cv_ptr->image, &min_d, &max_d);
            int img_rows_ = left_img_msg_ptr->height;
            int img_cols_ = left_img_msg_ptr->width;

            //////////SGM DEPTH//////////////
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
                        depth_uint16.at<unsigned short>(r, c) = 0;
                    } else if (disparity.at<float>(r, c) == 255.0f) {
                        depth_uint16.at<unsigned short>(r, c) = 0;
                    } else {
                        depth_uint16.at<unsigned short>(r, c) = static_cast<unsigned short>(
                                1000.0 * static_cast<float>(stereo_baseline_) * f /
                                disparity.at<float>(r, c));
                    }
                }
            }

            sensor_msgs::ImagePtr sgm_depth_msg =
                    cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_uint16).toImageMsg();
//            sensor_msgs::ImagePtr sgm_depth_msg =
//                    cv_bridge::CvImage(std_msgs::Header(), "mono", depth_float).toImageMsg();

            ///visualize
//            cv::Mat normalized_image;
//            cv::normalize(depth_uint16, normalized_image, 0, 255, cv::NORM_MINMAX);
//            normalized_image.convertTo(normalized_image, CV_8U);
//            cv::imshow("depth", normalized_image);
//            cv::setMouseCallback("depth", mouseEvent, &depth_uint16);
//            cv::waitKey(1);

            sgm_depth_msg->header = left_img_msg_ptr->header;
            disparity_pub_.publish(sgm_depth_msg);

            /// CV SGM ///
            if (USE_CV_SGM) {
                cv::Mat disparity_cv2;
                cv::Mat cv2_depth_uint16(img_rows_, img_cols_, CV_16UC1);
//            cv::Mat cv2_depth_float(img_rows_, img_cols_, CV_16FC1);

                cv_sgm_->compute(left_cv_ptr->image, right_cv_ptr->image, disparity_cv2);


                ///===============================
                cv::Mat XYZ(disparity_cv2.size(), CV_32F);
                float dataQ[] = {1.0, 0.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, f,
                                 0.0, 0.0, -1.0f / stereo_baseline_, 0.0};
                cv::Mat Qmat = cv::Mat(4, 4, CV_32F, dataQ).clone();
                cv::reprojectImageTo3D(disparity_cv2, XYZ, Qmat.clone(), false, CV_32F);
                ///=============================
                disparity_cv2.convertTo(disparity_cv2, CV_32FC1);
                disparity_cv2 = (disparity_cv2 / 16.0f);
                //  depth = static_cast<float>(stereo_baseline_) * f / disparity;
                for (int r = 0; r < img_rows_; ++r) {
                    for (int c = 0; c < img_cols_; ++c) {
                        if (disparity_cv2.at<float>(r, c) == 0.0f) {
                            cv2_depth_uint16.at<uint16_t>(r, c) = 0;
                        } else if (disparity_cv2.at<float>(r, c) < 0) {
                            cv2_depth_uint16.at<uint16_t>(r, c) = 0;
                        } else {
                            float depth_float = -16.0 * XYZ.at<cv::Vec3f>(r, c)[2];
//                        cv2_depth_uint16.at<uint16_t>(r, c) = static_cast<uint16_t>(
//                                1000.0f * static_cast<float>(stereo_baseline_) * f / disparity_cv2.at<float>(r, c));
                            if (1000.0f * depth_float > 60000) {
                                cv2_depth_uint16.at<uint16_t>(r, c) = static_cast<uint16_t>(60000);
                            }
                            else
                            {
                                cv2_depth_uint16.at<uint16_t>(r, c) = static_cast<uint16_t>(1000.0f * depth_float);
                            }
//                        std::cout<<cv2_depth_float.at<float>(r, c) << " " << cv2_depth_uint16.at<unsigned short>(r, c)<<" " <<disparity_cv2.at<float>(r, c)<<std::endl;
                        }
                    }
                }

                sensor_msgs::ImagePtr cv_sgm_depth_msg =
                        cv_bridge::CvImage(std_msgs::Header(), "mono16", cv2_depth_uint16).toImageMsg();
                cv_sgm_depth_msg->header = left_img_msg_ptr->header;
                cv::Mat normalized_image;
                cv::normalize(cv2_depth_uint16, normalized_image, 0, 255, cv::NORM_MINMAX);
                normalized_image.convertTo(normalized_image, CV_8U);
                cv::imshow("depth", normalized_image);
                cv::setMouseCallback("depth", mouseEvent, &cv2_depth_uint16);
                cv::waitKey(1);
                cv2_disparity_pub_.publish(cv_sgm_depth_msg);
            }
        }
    }

    ros::Time AirsimSgmNode::airsim_timestamp_to_ros(const msr::airlib::TTimePoint &stamp) const {
        // airsim appears to use chrono::system_clock with nanosecond precision
        std::chrono::nanoseconds dur(stamp);
        std::chrono::time_point<std::chrono::system_clock> tp(dur);
        ros::Time cur_time = chrono_timestamp_to_ros(tp);
        return cur_time;
    }

    ros::Time AirsimSgmNode::chrono_timestamp_to_ros(const std::chrono::system_clock::time_point &stamp) const {
        auto dur = std::chrono::duration<double>(stamp.time_since_epoch());
        ros::Time cur_time;
        cur_time.fromSec(dur.count());
        return cur_time;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "airsim_sgm_gpu_node");
    airsim_sgm_node::AirsimSgmNode airsim_sgm_gpu;
    ros::spin();

    return 0;
}
