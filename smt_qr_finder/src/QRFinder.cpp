#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <smt_msgs/PoseWithString.h>
#include <geometry_msgs/Pose.h>

#include "smt_qr_finder/QRFinder.hpp"


namespace smt {
    namespace qr_finder {
        QRFinder::QRFinder(ros::NodeHandle& nodeHandle) :tfListener(tfBuffer) {
            std::string imgSubTopic;
            int imgSubQueueSize;
            std::string imgPubTopic;
            int imgPubQueueSize;
            std::string qrCodePubTopic;
            int qrCodePubQueueSize;
            std::string qrCodePoseTopic;
            int qrCodePoseQueueSize;

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

            if (!nodeHandle.getParam("qr_code_pose_topic/topic", qrCodePoseTopic)) {
                ROS_ERROR("failed to load the `gun_topic/topic` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("qr_code_pose_topic/queue_size", qrCodePoseQueueSize)) {
                ROS_ERROR("failed to load the `gun_topic/queue_size` parameter!");
                ros::requestShutdown();
            }

            imgSubscriber = nodeHandle.subscribe(imgSubTopic, imgSubQueueSize, &QRFinder::imgCallback, this);
            ROS_INFO("starting subscriber for %s with queue size %i", imgSubTopic.c_str(), imgSubQueueSize);

            imgPublisher = nodeHandle.advertise<sensor_msgs::Image>(imgPubTopic, imgPubQueueSize);
            ROS_INFO("starting publisher for %s with queue size %i", imgPubTopic.c_str(), imgPubQueueSize);

            qrCodePublisher = nodeHandle.advertise<smt_msgs::PoseWithString>(qrCodePubTopic, qrCodePubQueueSize);
            ROS_INFO("starting publisher for %s with queue size %i", qrCodePubTopic.c_str(), qrCodePubQueueSize);

            qrCodePosePublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>(qrCodePoseTopic, qrCodePoseQueueSize);
            ROS_INFO("starting publisher for %s with queue size %i", qrCodePoseTopic.c_str(), qrCodePoseQueueSize);

            zbarScanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

            ROS_INFO("init QR_detector done");
        }

        void QRFinder::imgCallback(sensor_msgs::Image const& imageRaw) {
            auto const& cv_ptr = cv_bridge::toCvCopy(imageRaw, sensor_msgs::image_encodings::BGR8);
            auto const& image = cv_ptr->image;

            auto const& qrCodes = searchForQrCodes(image);

            if (qrCodes.empty()) {
                return;
            }

            cv::Mat taggedImage = image.clone();
            for (auto const& qrCode : qrCodes) {
                ROS_DEBUG_STREAM("found QR code: " << qrCode.text << " located at " << qrCode.polygon);

                // tag QR code on image
                cv::Scalar const COLOUR_GREEN = { 0, 255, 0 };
                cv::polylines(taggedImage, qrCode.polygon, true, COLOUR_GREEN, 2);

                // calculate & publish QR code position
                smt_msgs::PoseWithString positionMsg;
                positionMsg.header.frame_id = "odom";
                positionMsg.text = qrCode.text;
                positionMsg.pose = this->calculateQrCodePose(qrCode, image.cols, image.rows);
                qrCodePublisher.publish(positionMsg);

                // publish QR code pose
                geometry_msgs::PoseStamped poseMsg;
                poseMsg.header.frame_id = "odom";
                poseMsg.pose = positionMsg.pose;
                qrCodePosePublisher.publish(poseMsg);
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
                for (int i = 0; i < s->get_location_size(); i++) {
                    polygon.emplace_back(s->get_location_x(i), s->get_location_y(i));
                }
                qrCodes.push_back({ s->get_data(), polygon });
            }

            return qrCodes;
        }


        geometry_msgs::Pose QRFinder::calculateQrCodePose(QRFinder::QRCode const& qrCode, int const width, int const height) const {
            auto points = qrCode.polygon;
            const auto distanceFactor = 93.09; //!< must be calculated one time with the cam. [pixele * meter] (messured distance * pixel)
            const auto horizontalDistaneToQrCodeInMeter = distanceFactor / (points.at(0).y - points.at(1).y);
            const auto qrCodeHeight = 0.145; //!< Height of QR-Code in m
            const auto factorMPerPixel = qrCodeHeight / (points.at(0).x - points.at(3).x);

            const auto midXAxisQrCode = (points.at(0).x - points.at(3).x) / 2 + points.at(3).x;
            const auto midXAxisImg = width / 2;
            const auto qrDistanceToMidAxisInMeter = (midXAxisImg - midXAxisQrCode) * factorMPerPixel;

            // Erstelle eine Instanz der Pose-Nachricht
            geometry_msgs::Pose poseInCameraFrame;

            poseInCameraFrame.position.x = horizontalDistaneToQrCodeInMeter;
            poseInCameraFrame.position.y = qrDistanceToMidAxisInMeter;
            poseInCameraFrame.position.z = 0.02;

            poseInCameraFrame.orientation.x = 0.0;
            poseInCameraFrame.orientation.y = 0.0;
            poseInCameraFrame.orientation.z = 0.0;
            poseInCameraFrame.orientation.w = 1.0;


            geometry_msgs::TransformStamped transformStamped;
            transformStamped = this->tfBuffer.lookupTransform("odom", "cam_1", ros::Time(0));

            geometry_msgs::Pose poseInOdomFrame;
            tf2::doTransform(poseInCameraFrame, poseInOdomFrame, transformStamped);

            return poseInOdomFrame;
        }

    } // namesspace qr_detector
}  // namespace smt
