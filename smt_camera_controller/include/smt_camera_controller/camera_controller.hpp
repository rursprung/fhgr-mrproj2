#ifndef SMT_CAMERA_CONTROLLER_CAMERACONTROLLER_HPP
#define SMT_CAMERA_CONTROLLER_CAMERACONTROLLER_HPP

#include <raspicam/raspicam_cv.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace smt {
namespace camera_controller {
class CameraController {
   public:
    CameraController(ros::NodeHandle &nodeHandle);
    ~CameraController();
    void updateImg();
    void spin();

   private:
    ros::NodeHandle nodeHandle;
    ros::Publisher imgPublisher;
    raspicam::RaspiCam_Cv camera;
    int fps;
};
}  // namespace camera_controller
}  // namespace smt

#endif  // SMT_CAMERA_CONTROLLER_CAMERACONTROLLER_HPP
