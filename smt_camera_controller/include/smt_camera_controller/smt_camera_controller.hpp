#ifndef smt_camera_controller_HPP
#define smt_camera_controller_HPP

#include <ros/ros.h>
#include <raspicam/raspicam_cv.h>
#include <sensor_msgs/Image.h>

namespace smt{
    
    class cameraController{
        public:
            cameraController(ros::NodeHandle &nodeHandle);
            ~cameraController();
            void updateImg();

        private:
            ros::NodeHandle nodeHandle_;
            ros::Publisher imgPublisher_;
            raspicam::RaspiCam_Cv camera_;
            int chipWidth_;
            int chipHeight_;
            int imgQueueSize_;
            std::string imgTopic_;

    };
}

#endif //smt_camera_controller_HPP
