#ifndef SMT_QR_DETECTION_QR_DETECTOR_HPP
#define SMT_QR_DETECTION_QR_DETECTOR_HPP

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace smt {

    namespace qr_detector {

        class QRDetector {
        public:
            QRDetector(ros::NodeHandle& nodeHandle);
            void imgCallback(const sensor_msgs::Image imageRaw);


        private:
            ros::Subscriber imgSubscriber;
            ros::Publisher imgPublisher;
            ros::Publisher gunPublisher;

            void searchForQrCodes(cv::Mat& img);
            void generateNewImage(cv::Mat& img, cv::Mat& points);
            int computeDistance(int qrCodeHeight);

        };
    }  // namespace qr_detector

}  // namespace smt

#endif  // SMT_QR_DETECTION_QR_DETECTOR_HPP
