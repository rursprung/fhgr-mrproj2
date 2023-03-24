#include <ros/ros.h>
#include <wiringPi.h>
#include <smt_movement_controller/controller.hpp>

enum pin {pwm_left = 18,        //!< pwm motor Left
          pwm_right = 19,       //!< pwm motor right
          dir_left_1 = 1,       //!< direction pin left 1
          dir_left_2 = 2,       //!< direction pin left 2
          dir_rigt_1 = 3,       //!< direction pin right 1 
          dir_right_2 = 4,      //!< direction pin right 2
          };

const int PWM_FREQ = 20000;     //!< Hertz
const int PWM_RANGE = 100;      //!< Range for pwm 0% to 100%
const int PWM_CLOCK = 19.2e6;   //!< Pi PWM clock base frequency

namespace smt{

    gpioController::gpioController(ros::NodeHandle &nodeHandle) :
        nodeHandle_(nodeHandle)
    {

        if (!this->nodeHandle_.getParam("/smt_movement_controller/vel_sub_topic_name", velTopic_)) {
            ROS_ERROR("failed to load the `vel_sub_topic_name` parameter!");
            ros::requestShutdown();
        }

        if (!this->nodeHandle_.getParam("/smt_movement_controller/vel_sub_queue_size", subscriberQueueSize_)) {
            ROS_ERROR("failed to load the `vel_sub_queue_size` parameter!");
            ros::requestShutdown();
        }

        velSubscriber_ = nodeHandle_.subscribe(velTopic_,subscriberQueueSize_, &gpioController::scanCallback, this);
        ROS_INFO("starting subscriber for %s with queue size %i", velTopic_.c_str(), subscriberQueueSize_);

        PiInit();
        ROS_INFO("init done");
    }

    void gpioController::scanCallback(const geometry_msgs::Twist::ConstPtr& velocity){

        double linear_x = velocity->linear.x;
        double angular_z = velocity->angular.z;

        vel2motors(linear_x, angular_z);
    }

    void gpioController::vel2motors(const double &linear_x, const double &angular_z){
        double b = 0.201; //Wheelseperation
        double vel_mot_left = linear_x - (angular_z * b) / 2;
        double vel_mot_right = linear_x + (angular_z * b) / 2;

        applyMotorSpeed(vel_mot_left,vel_mot_right);
    }

    void gpioController::PiInit(){
        wiringPiSetup();

        //PWMs
        pinMode(pwm_left, PWM_OUTPUT);
        pinMode(pwm_right, PWM_OUTPUT);
        pwmSetMode(PWM_MODE_MS);

        pwmSetRange(PWM_RANGE); // set range to 100
        pwmSetClock((int)(PWM_CLOCK / (PWM_FREQ * PWM_RANGE)));
        
        //GPIOs
        pinMode(dir_left_1, OUTPUT);
        pinMode(dir_left_2, OUTPUT);
        pinMode(dir_rigt_1, OUTPUT);
        pinMode(dir_right_2, OUTPUT);

        digitalWrite(dir_left_1, LOW);
        digitalWrite(dir_left_2, LOW);
        digitalWrite(dir_rigt_1, LOW);
        digitalWrite(dir_right_2, LOW);
    }

    void gpioController::applyMotorSpeed(const double vLeft, const double vRight){
    if(vLeft == 0){
        digitalWrite(dir_left_1, LOW);
        digitalWrite(dir_left_2, LOW);
    }
    else if(vLeft > 0){
        digitalWrite(dir_left_1, LOW);
        digitalWrite(dir_left_2, HIGH);
    }
    else{
        digitalWrite(dir_left_1, HIGH);
        digitalWrite(dir_left_2, LOW);
    }

    if(vRight == 0){
        digitalWrite(dir_rigt_1, LOW);
        digitalWrite(dir_right_2, LOW);
    }
    else if(vRight > 0){
        digitalWrite(dir_rigt_1, LOW);
        digitalWrite(dir_right_2, HIGH);
    }
    else{
        digitalWrite(dir_rigt_1, HIGH);
        digitalWrite(dir_right_2, LOW);
    }

}

}




