#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "smt_qr_detection/qr_detector.hpp"
#include <std_msgs/Int32.h>


namespace smt {
    namespace qr_detector {
        /* `QRDetector` is a class that initializes subscribers and publishers for image topics and gun
        topic. It loads parameters from the ROS parameter server and sets up the subscribers and
        publishers accordingly. */
        QRDetector::QRDetector(ros::NodeHandle& nodeHandle) {
            std::string imgSubTopic;
            int imgSubQueueSize;
            std::string imgPubTopic;
            int imgPubQueueSize;
            std::string gunPubTopic;
            int gunPubQueueSize;

            if (!nodeHandle.getParam("img_sub_topic/topic", imgSubTopic)) {
                ROS_ERROR("failed to load the `img_sub_topic/topic` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("img_sub_topic/queue_size", imgSubQueueSize)) {
                ROS_ERROR("failed to load the `img_sub_topic/queue_size` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("img_pub_topic/topic", imgPubTopic)) {
                ROS_ERROR("failed to load the `img_pub_topic/topic` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("img_pub_topic/queue_size", imgPubQueueSize)) {
                ROS_ERROR("failed to load the `img_pub_topic/queue_size` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("gun_topic/topic", gunPubTopic)) {
                ROS_ERROR("failed to load the `gun_topic/topic` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("gun_topic/queue_size", gunPubQueueSize)) {
                ROS_ERROR("failed to load the `gun_topic/queue_size` parameter!");
                ros::requestShutdown();
            }
            imgSubscriber = nodeHandle.subscribe(imgSubTopic, imgSubQueueSize, &QRDetector::imgCallback, this);
            ROS_INFO("starting subscriber for %s with queue size %i", imgSubTopic.c_str(), imgSubQueueSize);

            imgPublisher = nodeHandle.advertise<sensor_msgs::Image>(imgPubTopic, imgPubQueueSize);
            ROS_INFO("starting publisher for %s with queue size %i", imgPubTopic.c_str(), imgPubQueueSize);

            gunPublisher = nodeHandle.advertise<std_msgs::Int32>(gunPubTopic, gunPubQueueSize);
            ROS_INFO("starting publisher for %s with queue size %i", gunPubTopic.c_str(), gunPubQueueSize);

            ROS_INFO("init QR_detector done");
        }

        void QRDetector::imgCallback(const sensor_msgs::Image imageRaw) const {
            const auto cv_ptr = cv_bridge::toCvCopy(imageRaw, sensor_msgs::image_encodings::BGR8);
            const cv::Mat image = cv_ptr->image;
            searchForQrCodes(image);
        }

        void QRDetector::searchForQrCodes(const cv::Mat& img) const {
            cv::Mat imgGrey, points, rectangles;
            cv::cvtColor(img, imgGrey, cv::COLOR_BGR2GRAY);
            std::string data = qrDet.detectAndDecode(imgGrey, points, rectImage);

            if (data.length() > 0) {
                generateNewImage(img, points);
            }

            const auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
            imgPublisher.publish(msg);
        }

        void QRDetector::generateNewImage(const cv::Mat& img, cv::Mat& points) const {
            const uint x1 = static_cast<uint>(points.at<float>(0, 0));
            const uint y1 = static_cast<uint>(points.at<float>(0, 1));
            const uint x2 = static_cast<uint>(points.at<float>(0, 2));
            const uint y2 = static_cast<uint>(points.at<float>(0, 3));
            const uint x3 = static_cast<uint>(points.at<float>(0, 4));
            const uint y3 = static_cast<uint>(points.at<float>(0, 5));
            const uint x4 = static_cast<uint>(points.at<float>(0, 6));
            const uint y4 = static_cast<uint>(points.at<float>(0, 7));
            cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 4);
            cv::line(img, cv::Point(x2, y2), cv::Point(x3, y3), cv::Scalar(0, 255, 0), 4);
            cv::line(img, cv::Point(x3, y3), cv::Point(x4, y4), cv::Scalar(0, 255, 0), 4);
            cv::line(img, cv::Point(x4, y4), cv::Point(x1, y1), cv::Scalar(0, 255, 0), 4);

            const auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
            imgPublisher.publish(msg);

        }

    } // namesspace qr_detector
}  // namespace smt
