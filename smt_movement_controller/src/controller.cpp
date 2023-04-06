#include <ros/ros.h>
#include <pigpio.h>
#include <smt_movement_controller/controller.hpp>


enum pin {pwm_left = 4,         //!< pwm motor Left (Pyhsical 7)
          pwm_right = 5,        //!< pwm motor right (Physical 29)
          dir_left_1 = 17,      //!< direction pin left 1 (Physical 11)
          dir_left_2 = 27,      //!< direction pin left 2 (Physical 13)
          dir_right_1 = 6,      //!< direction pin right 1 (Physical 31)
          dir_right_2 = 13,     //!< direction pin right 2 (Physical 33)
          };

const int PWM_FREQ = 20000;     //!< Hertz
const int PWM_RANGE = 100;      //!< Range for pwm 0% to 100%

const int MAX_SPEED = 5;        //!< Max Speed for the cmd_vel


double map(double value, double in_min, double in_max, double out_min, double out_max)
{
    double returnVal;
    if(value < in_min){
        returnVal = out_min;
    }
    else if(value > in_max){
        returnVal = out_max;
    }
    else{
        returnVal = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    return returnVal;
}

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

    gpioController::~gpioController()
    {
        gpioTerminate();
    }

    void gpioController::scanCallback(const geometry_msgs::Twist::ConstPtr &velocity)
    {

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
        gpioTerminate();
        setenv("PIGPIO_PORT", "8889", 1);
        if (gpioInitialise() < 0){
            ROS_ERROR("Fehler beim Initialisieren von pigpio!"); 
            return;
        }


        //PWMs
        gpioSetMode(pwm_left, PI_OUTPUT);
        gpioSetMode(pwm_right, PI_OUTPUT);

        gpioSetPWMfrequency(pwm_left, PWM_FREQ);
        gpioSetPWMfrequency(pwm_right, PWM_FREQ);

        gpioSetPWMrange(pwm_left, PWM_RANGE);
        gpioSetPWMrange(pwm_right, PWM_RANGE);

        //GPIOs
        gpioSetMode(dir_left_1, PI_OUTPUT);
        gpioSetMode(dir_left_2, PI_OUTPUT);
        gpioSetMode(dir_right_1, PI_OUTPUT);
        gpioSetMode(dir_right_2, PI_OUTPUT);
        
        gpioWrite(dir_left_1, 0);
        gpioWrite(dir_left_2, 0);
        gpioWrite(dir_right_1, 0);
        gpioWrite(dir_right_2, 0);

    }

    void gpioController::applyMotorSpeed(const double vLeft, const double vRight){
        int pwm_val_left = map(abs(vLeft),0,MAX_SPEED,0,PWM_RANGE);
        int pwm_val_right = map(abs(vRight),0,MAX_SPEED,0,PWM_RANGE);

        if(vLeft == 0){
            gpioWrite(dir_left_1, 0);
            gpioWrite(dir_left_2, 0);
        }
        else if(vLeft > 0){
            gpioWrite(dir_left_1, 0);
            gpioWrite(dir_left_2, 1);
        }
        else{
            gpioWrite(dir_left_1, 1);
            gpioWrite(dir_left_2, 0);
        }

        if(vRight == 0){
            gpioWrite(dir_right_1, 0);
            gpioWrite(dir_right_2, 0);
        }
        else if(vRight > 0){
            gpioWrite(dir_right_1, 0);
            gpioWrite(dir_right_2, 1);
        }
        else{
            gpioWrite(dir_right_1, 1);
            gpioWrite(dir_right_2, 0);
        }

        gpioPWM(pwm_left, pwm_val_left);
        gpioPWM(pwm_right, pwm_val_right);
    }
}
