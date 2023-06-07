#ifndef SMT_QR_CONTROLLER_SMT_QR_CONTROLLER_HPP
#define SMT_QR_CONTROLLER_SMT_QR_CONTROLLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

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
            ros::Publisher velocityCommandPublisher;

            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener;

            std::vector<std::string> handledQrCodes;

            bool qrCodeAlreadyHandled(std::string const& qrCode) const;
            void handleQrCode(geometry_msgs::PoseStamped const& qrCodePose);
            void turnRobotByAngle(double turnAngle);
            void fireGun(int gunAngle);
        };
    }
}

#endif // SMT_QR_CONTROLLER_SMT_QR_CONTROLLER_HPP
