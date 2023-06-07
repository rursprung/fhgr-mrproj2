#include <algorithm>

#include <smt_svcs/FireGun.h>

#include "smt_qr_controller/smt_qr_controller.hpp"

namespace smt {
    namespace qr_controller {

        qr_controller::QrController::QrController(ros::NodeHandle &nodeHandle) {
            std::string qrCodeTopic;
            int qrCodeQueueSize;
            std::string gunControllerService;

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

            this->qrCodeSubscriber = nodeHandle.subscribe(qrCodeTopic, qrCodeQueueSize, &QrController::qrCodeCallback, this);
            this->gunControllerClient = nodeHandle.serviceClient<smt_svcs::FireGun>(gunControllerService);
        }

        void QrController::qrCodeCallback(smt_msgs::PoseWithString const& qrCode) {
            if (!this->qrCodeAlreadyHandled(qrCode.text)) {
                this->handledQrCodes.push_back(qrCode.text);
                ROS_INFO_STREAM("received new QR code " << qrCode.text << " with pose " << qrCode.pose);

                this->handleQrCode(qrCode.pose);
            }
        }

        bool QrController::qrCodeAlreadyHandled(const std::string &qrCode) const {
            return std::find(this->handledQrCodes.begin(), this->handledQrCodes.end(), qrCode) != this->handledQrCodes.end();
        }

        void QrController::handleQrCode(const geometry_msgs::Pose &qrCodePose) {
            // TODO: properly handle all of this
            smt_svcs::FireGun fireGunCall;
            fireGunCall.request.angle = 10;
            if (this->gunControllerClient.call(fireGunCall)) {
                ROS_INFO("successfully fired gun");
            } else {
                ROS_ERROR("failed to fire the gun");
            }
        }
    }
}
