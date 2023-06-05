#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <smt_qr_msgs/PoseWithString.h>

#include "smt_qr_finder/QRFinder.hpp"


namespace smt {
    namespace qr_finder {
        QRFinder::QRFinder(ros::NodeHandle& nodeHandle) {
            std::string imgSubTopic;
            int imgSubQueueSize;
            std::string imgPubTopic;
            int imgPubQueueSize;
            std::string qrCodePubTopic;
            int qrCodePubQueueSize;

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

            if (!nodeHandle.getParam("qr_code_topic/topic", qrCodePubTopic)) {
                ROS_ERROR("failed to load the `gun_topic/topic` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("qr_code_topic/queue_size", qrCodePubQueueSize)) {
                ROS_ERROR("failed to load the `gun_topic/queue_size` parameter!");
                ros::requestShutdown();
            }
            imgSubscriber = nodeHandle.subscribe(imgSubTopic, imgSubQueueSize, &QRFinder::imgCallback, this);
            ROS_INFO("starting subscriber for %s with queue size %i", imgSubTopic.c_str(), imgSubQueueSize);

            imgPublisher = nodeHandle.advertise<sensor_msgs::Image>(imgPubTopic, imgPubQueueSize);
            ROS_INFO("starting publisher for %s with queue size %i", imgPubTopic.c_str(), imgPubQueueSize);

            qrCodePublisher = nodeHandle.advertise<smt_qr_msgs::PoseWithString>(qrCodePubTopic, qrCodePubQueueSize);
            ROS_INFO("starting publisher for %s with queue size %i", qrCodePubTopic.c_str(), qrCodePubQueueSize);

            zbarScanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

            ROS_INFO("init QR_detector done");
        }

        void QRFinder::imgCallback(sensor_msgs::Image const& imageRaw) {
            auto const& cv_ptr = cv_bridge::toCvCopy(imageRaw, sensor_msgs::image_encodings::BGR8);
            auto const& image = cv_ptr->image;

            auto const& qrCodes = searchForQrCodes(image);

            if(qrCodes.empty()) {
                return;
            }

            cv::Mat taggedImage = image.clone();
            for (auto const& qrCode : qrCodes) {
                ROS_INFO_STREAM("found QR code: " << qrCode.text << " located at " << qrCode.polygon);

                // tag QR code on image
                cv::Scalar const COLOUR_GREEN = {0, 255, 0};
                cv::polylines(taggedImage, qrCode.polygon, true, COLOUR_GREEN);

                // calculate & publish QR code position
                smt_qr_msgs::PoseWithString positionMsg;
                positionMsg.header.frame_id = "odom";
                positionMsg.text = qrCode.text;
                positionMsg.pose = this->calculateQrCodePose(qrCode);
                qrCodePublisher.publish(positionMsg);
            }

            // publish tagged image
            const auto taggedImageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", taggedImage).toImageMsg();
            imgPublisher.publish(taggedImageMsg);
        }

        std::vector<QRFinder::QRCode> QRFinder::searchForQrCodes(cv::Mat const& img) {
            cv::Mat grayImg, points, rectImage;
            cv::cvtColor(img, grayImg, cv::COLOR_BGR2GRAY);

            const auto width = grayImg.cols;
            const auto height = grayImg.rows;

            zbar::Image zbarImg(width, height, "Y800", grayImg.data, width * height);
            zbarScanner.scan(zbarImg);

            std::vector<QRFinder::QRCode> qrCodes;
            for (auto s = zbarImg.symbol_begin(); s != zbarImg.symbol_end(); ++s)
            {
                std::vector<cv::Point> polygon;
                for(int i = 0; i < s->get_location_size(); i++) {
                    polygon.emplace_back(s->get_location_x(i), s->get_location_y(i));
                }
                qrCodes.push_back({s->get_data(), polygon});
            }

            return qrCodes;
        }

        geometry_msgs::Pose QRFinder::calculateQrCodePose(QRFinder::QRCode) const {
            // TODO: 1. identify QR code size & orientation
            // TODO: 2. calculate vector from camera to QR code
            // TODO: 3. create pose (point + orientation) with this information
            // TODO: 4. transform pose to `odom` frame using tf
            return {};
        }

    } // namesspace qr_detector
}  // namespace smt
