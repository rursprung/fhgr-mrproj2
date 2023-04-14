#include <ros/ros.h>
#include <raspicam/raspicam_cv.h>
#include <cv_bridge/cv_bridge.h>
#include <smt_camera_controller/smt_camera_controller.hpp>

namespace smt{
    cameraController::cameraController(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle)
    {
        if (!this->nodeHandle_.getParam("/smt_camera_controller/img_pub_topic_name", imgTopic_)) {
            ROS_ERROR("failed to load the `img_pub_topic_name` parameter!");
            ros::requestShutdown();
        }

        if (!this->nodeHandle_.getParam("/smt_camera_controller/img_pub_queue_size", imgQueueSize_)) {
            ROS_ERROR("failed to load the `img_pub_queue_size` parameter!");
            ros::requestShutdown();
        }

        if (!this->nodeHandle_.getParam("/smt_camera_controller/chip_height", chipHeight_)) {
            ROS_ERROR("failed to load the `chip_height` parameter!");
            ros::requestShutdown();
        }

        if (!this->nodeHandle_.getParam("/smt_camera_controller/chip_width", chipWidth_)) {
            ROS_ERROR("failed to load the `chip_width` parameter!");
            ros::requestShutdown();
        }


        camera_.set(cv::CAP_PROP_FORMAT, CV_8UC3);
        camera_.set(cv::CAP_PROP_FRAME_WIDTH, chipWidth_);
        camera_.set(cv::CAP_PROP_FRAME_HEIGHT, chipHeight_);
        camera_.open();

        if (!camera_.isOpened())
        {
            ROS_ERROR("Failed to open camera");
        }


        imgPublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(imgTopic_, imgQueueSize_);
    }

    cameraController::~cameraController(){
        camera_.release();
    }


    void smt::cameraController::updateImg(){
        cv::Mat image;
        sensor_msgs::ImagePtr msg;

        camera_.grab();
        camera_.retrieve(image);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        imgPublisher_.publish(msg);
    }
}
