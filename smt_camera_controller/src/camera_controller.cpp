#include <cv_bridge/cv_bridge.h>
#include <raspicam/raspicam_cv.h>
#include <ros/ros.h>

#include <smt_camera_controller/camera_controller.hpp>

namespace smt {
    namespace camera_controller {
        CameraController::CameraController(ros::NodeHandle& nodeHandle)
            : nodeHandle(nodeHandle) {
            int chipWidth;
            int chipHeight;
            int imgQueueSize;
            std::string imgTopic;

            if (!this->nodeHandle.getParam("img_pub_topic/topic", imgTopic)) {
                ROS_ERROR("failed to load the `img_pub_topic/topic` parameter!");
                ros::requestShutdown();
            }

            if (!this->nodeHandle.getParam("img_pub_topic/queue_size", imgQueueSize)) {
                ROS_ERROR("failed to load the `img_pub_topic/queue_size` parameter!");
                ros::requestShutdown();
            }

            if (!this->nodeHandle.getParam("camera/chip/height", chipHeight)) {
                ROS_ERROR("failed to load the `camera/chip/height` parameter!");
                ros::requestShutdown();
            }

            if (!this->nodeHandle.getParam("camera/chip/width", chipWidth)) {
                ROS_ERROR("failed to load the `camera/chip/width` parameter!");
                ros::requestShutdown();
            }

            if (!this->nodeHandle.getParam("camera/fps", fps)) {
                ROS_ERROR("failed to load the `camera/fps` parameter!");
                ros::requestShutdown();
            }

            camera.set(cv::CAP_PROP_FORMAT, CV_8UC3);
            camera.set(cv::CAP_PROP_FRAME_WIDTH, chipWidth);
            camera.set(cv::CAP_PROP_FRAME_HEIGHT, chipHeight);
            camera.open();

            if (!camera.isOpened()) {
                ROS_ERROR("Failed to open camera");
                ros::requestShutdown();
            }

            imgPublisher = nodeHandle.advertise<sensor_msgs::Image>(imgTopic, imgQueueSize);
        }

        CameraController::~CameraController() {
            camera.release();
        }

        void CameraController::updateImg() {
            cv::Mat image;
            cv::Mat flippedImage;

            camera.grab();
            camera.retrieve(image);
            cv::flip(image, flippedImage, -1);
            const auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", flippedImage).toImageMsg();
            imgPublisher.publish(msg);
        }

        void CameraController::spin() {
            ros::Rate loop_rate(fps);
            while (nodeHandle.ok()) {
                updateImg();
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

    }  // namespace camera_controller
}  // namespace smt

