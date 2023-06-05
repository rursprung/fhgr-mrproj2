#ifndef SMT_QR_DETECTION_QR_DETECTOR_HPP
#define SMT_QR_DETECTION_QR_DETECTOR_HPP

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>

namespace smt {

    namespace qr_detector {

        class QRDetector {
        public:
            QRDetector(ros::NodeHandle& nodeHandle);
            void imgCallback(const sensor_msgs::Image imageRaw) const;


        private:
            ros::Subscriber imgSubscriber;
            ros::Publisher imgPublisher;
            ros::Publisher gunPublisher;

            void searchForQrCodes(const cv::Mat& img) const;
            void generateNewImage(const cv::Mat& img, cv::Mat& points) const;

        };
    }  // namespace qr_detector

}  // namespace smt

#endif  // SMT_QR_DETECTION_QR_DETECTOR_HPP
