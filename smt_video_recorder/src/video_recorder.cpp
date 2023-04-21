#include <ros/ros.h>
#include <smt_video_recorder/video_recorder.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>


namespace smt{

    videoController::videoController(ros::NodeHandle &nodeHandle) :
        nodeHandle_(nodeHandle)
    {
        int fps = 30;
        int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); 
        cv::Size frameSize = cv::Size(640, 480);

        videoTopic_ = "/camera1/image_raw";
        subscriberQueueSize_ = 10;
        videoSubscriber_ = nodeHandle_.subscribe(videoTopic_,subscriberQueueSize_, &videoController::scanCallback, this);
        
        videoWriter_ = cv::VideoWriter("/home/ros/git/001.avi", fourcc, fps, frameSize, true);
        
        ROS_INFO("starting subscriber for %s with queue size %i", videoTopic_.c_str(), subscriberQueueSize_);

        ROS_INFO("init done");
    }

    videoController::~videoController()
    {

    }

    void videoController::scanCallback(const sensor_msgs::Image::ConstPtr &img)
    {

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat matImg = cv_ptr->image;
        
        videoWriter_ << matImg;
        ROS_INFO("Bild Kopiert!");

    }

}
