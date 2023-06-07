#ifndef SMT_QR_CONTROLLER_SMT_QR_CONTROLLER_HPP
#define SMT_QR_CONTROLLER_SMT_QR_CONTROLLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <smt_msgs/PoseWithString.h>

namespace smt {
    namespace qr_controller {
        class QrController {
        public:
            explicit QrController(ros::NodeHandle& nodeHandle);

            void qrCodeCallback(smt_msgs::PoseWithString const& qrCode);

        private:
            ros::Subscriber qrCodeSubscriber;
            ros::ServiceClient gunControllerClient;

            std::vector<std::string> handledQrCodes;

            bool qrCodeAlreadyHandled(std::string const& qrCode) const;
            void handleQrCode(geometry_msgs::Pose const& qrCodePose);
        };
    }
}

#endif // SMT_QR_CONTROLLER_SMT_QR_CONTROLLER_HPP
