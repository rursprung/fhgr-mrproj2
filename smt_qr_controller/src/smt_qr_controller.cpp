#include <algorithm>
#include <cmath>

#include <geometry_msgs/Twist.h>

#include <smt_svcs/FireGun.h>

#include "smt_qr_controller/smt_qr_controller.hpp"

namespace smt {
    namespace qr_controller {

        qr_controller::QrController::QrController(ros::NodeHandle &nodeHandle) : tfListener(tfBuffer) {
            std::string qrCodeTopic;
            int qrCodeQueueSize;
            std::string gunControllerService;
            std::string velTopic;
            int velTopicQueueSize;

            if (!nodeHandle.getParam("qr_code_topic/topic", qrCodeTopic)) {
                ROS_ERROR("failed to load the `gun_topic/topic` parameter!");
                ros::requestShutdown();
            }
            if (!nodeHandle.getParam("qr_code_topic/queue_size", qrCodeQueueSize)) {
                ROS_ERROR("failed to load the `gun_topic/queue_size` parameter!");
                ros::requestShutdown();
            }
            if (!nodeHandle.getParam("gun_controller_service", gunControllerService)) {
                ROS_ERROR("failed to load the `gun_controller_service` parameter!");
                ros::requestShutdown();
            }

            if (!nodeHandle.getParam("cmd_vel_topic/topic", velTopic)) {
                ROS_ERROR("failed to load the `cmd_vel_topic/topic` parameter!");
                ros::requestShutdown();
            }
            if (!nodeHandle.getParam("cmd_vel_topic/queue_size", velTopicQueueSize)) {
                ROS_ERROR("failed to load the `cmd_vel_topic/queue_size` parameter!");
                ros::requestShutdown();
            }

            this->qrCodeSubscriber = nodeHandle.subscribe(qrCodeTopic, qrCodeQueueSize, &QrController::qrCodeCallback, this);
            this->gunControllerClient = nodeHandle.serviceClient<smt_svcs::FireGun>(gunControllerService);
            this->velocityCommandPublisher = nodeHandle.advertise<geometry_msgs::Twist>(velTopic, velTopicQueueSize)
        }

        void QrController::qrCodeCallback(smt_msgs::PoseWithString const& qrCode) {
            if (!this->qrCodeAlreadyHandled(qrCode.text)) {
                this->handledQrCodes.push_back(qrCode.text);
                ROS_INFO_STREAM("received new QR code " << qrCode.text << " with pose " << qrCode.pose);

                geometry_msgs::PoseStamped stampedQrCodePose;
                stampedQrCodePose.header = qrCode.header;
                stampedQrCodePose.pose = qrCode.pose;
                this->handleQrCode(stampedQrCodePose);
            }
        }

        bool QrController::qrCodeAlreadyHandled(const std::string &qrCode) const {
            return std::find(this->handledQrCodes.begin(), this->handledQrCodes.end(), qrCode) != this->handledQrCodes.end();
        }

        void QrController::handleQrCode(geometry_msgs::PoseStamped const& qrCodePose) {
            geometry_msgs::TransformStamped const transformStamped = this->tfBuffer.lookupTransform("base_link", qrCodePose.header.frame_id, ros::Time(0));

            geometry_msgs::PoseStamped qrCodePoseFromRobot;
            tf2::doTransform(qrCodePose, qrCodePoseFromRobot, transformStamped);

            auto const turnAngleInDegrees = std::atan2(qrCodePoseFromRobot.pose.position.y, qrCodePoseFromRobot.pose.position.x) * 180 / M_PI;
            this->turnRobotByAngle(turnAngleInDegrees);

            // we currently don't calculate the height of the QR code
            this->fireGun(10);
        }

        void QrController::turnRobotByAngle(const double turnAngle) {
            // for now this is pretty simple:
            // we turn at a fixed velocity and calculate the time we need to give it until we stop the rotation again.
            geometry_msgs::Twist twistMessage;
            twistMessage.angular.z = 2;
            this->velocityCommandPublisher.publish(twistMessage);
            ros::Duration(2).sleep(); // FIXME: calculate time based on turn angle!
            twistMessage.angular.z = 0;
            this->velocityCommandPublisher.publish(twistMessage);
        }

        void QrController::fireGun(const int gunAngle) {
            smt_svcs::FireGun fireGunCall;
            fireGunCall.request.angle = gunAngle;
            if (this->gunControllerClient.call(fireGunCall)) {
                ROS_INFO("successfully fired gun");
            } else {
                ROS_ERROR("failed to fire the gun");
            }
        }
    }
}
