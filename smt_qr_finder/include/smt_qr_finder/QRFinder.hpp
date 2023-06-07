#ifndef SMT_QR_FINDER_QRFINDER_HPP
#define SMT_QR_FINDER_QRFINDER_HPP

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.h>

#include <zbar.h>

namespace smt {

    namespace qr_finder {

        class QRFinder {
        public:
            explicit QRFinder(ros::NodeHandle& nodeHandle);
            void imgCallback(sensor_msgs::Image const& imageRaw);

        private:
            struct QRCode {
                std::string text;
                std::vector<cv::Point> const polygon;
            };

            ros::Subscriber imgSubscriber;
            ros::Publisher imgPublisher;
            ros::Publisher qrCodePublisher;
            ros::Publisher qrCodePosePublisher;
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener;

            zbar::ImageScanner zbarScanner;

            std::vector<QRFinder::QRCode> searchForQrCodes(cv::Mat const& img);

            geometry_msgs::Pose calculateQrCodePose(QRFinder::QRCode const& qrCode, int const width, int const height) const;
        };
    }  // namespace qr_detector

}  // namespace smt

#endif  // SMT_QR_FINDER_QRFINDER_HPP
