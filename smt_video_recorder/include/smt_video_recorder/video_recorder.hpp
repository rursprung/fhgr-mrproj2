#ifndef video_recorder_HPP
#define video_recorder_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

namespace smt{

    class videoController{
        public: 
            videoController(ros::NodeHandle &nodeHandle);
            ~videoController();
            
            void scanCallback(const sensor_msgs::Image::ConstPtr &img);

        private:
            ros::NodeHandle nodeHandle_;
            ros::Subscriber videoSubscriber_;
            int subscriberQueueSize_;
            std::string videoTopic_;

            cv::VideoWriter videoWriter_;

    };
}

#endif // video_recorder_HPP
