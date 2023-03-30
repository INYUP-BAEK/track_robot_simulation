 #include <string>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class PS4_ROS {

public:

    /**
     * @brief      { PS4 to TWIST MESSAGES }
     *
     */
    PS4_ROS(ros::NodeHandle &n) {
        // get ros param
        ros::NodeHandle private_nh("~");
        private_nh.param("scale_linear", this->scale_linear, 1.0);
        private_nh.param("scale_angular", this->scale_angular, 1.0);
        private_nh.param<std::string>("pub_topic", this->pubName, "/searchbot/p3at/vel_cmd");

        this->chat = n.advertise<geometry_msgs::Twist>(pubName, 1000);
        this->sub = n.subscribe<sensor_msgs::Joy>("/joy", 10, &PS4_ROS::subscribePS4, this);

        /* set calibration counter to zero */
        this->calib1 = 0;
        this->calib2 = 0;
        this->calib = false;

        this->maxVel = this->scale_linear;
        this->maxVelR = this->scale_linear * -1;

        ROS_INFO("maxVelR: %f", this->maxVelR);

        ROS_INFO("scale_linear set to: %f", this->scale_linear);
        ROS_INFO("scale_angular set to: %f", this->scale_angular);
        ROS_INFO("PS4_ROS initialized");
    }

    ~PS4_ROS() {
        // std::cout << "Destroy the pointer" << std::endl;
    }

    void run() {
        if(this->calib) {
            this->publishTwistMsg();
        }
    }

    void prepareData()
    {
        // Normalize velocity between ]-1.0, 1.0[
        this->send_l2 = (-0.5 * this->l2) + 0.5;
        this->send_r2 = (this->r2 - 1.0) * 0.5;

        // Apply rosparam "scale_linear"
        this->send_l2 = this->scale_linear * this->send_l2;
        this->send_r2 = this->scale_linear * this->send_r2;

        // Apply rosparam "scale_angular"
//        this->send_leftStickX = this->scale_angular * this->leftStickX;
    }

    void publishTwistMsg() {
        geometry_msgs::Twist msg;
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;

        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;

        prepareData();

        //printRaw();
        if(!this->buttonTouch) {
            if ((this->send_l2 >= 0.1) && (this->send_l2 <= maxVel)) {
                msg.linear.x = this->send_l2;
            } else if ((this->send_r2 <= 0.0) && (this->send_r2 >= maxVelR)) {
                msg.linear.x = this->send_r2;
            }
            msg.angular.z = this->send_leftStickX;
        }
        else{
            //ROS_WARN("SENDING EMERGENCY STOP");

            /* To Do */
        }

        this->chat.publish(msg);
    }

    void subscribePS4(const sensor_msgs::Joy::ConstPtr &joy)
    {
      ///////////////////////Wired///////////////////////

//      this->buttonSq = joy->buttons[0];
//      this->buttonX = joy->buttons[1];
//      this->buttonO = joy->buttons[2];
//      this->buttonTr = joy->buttons[3];
//      this->buttonTouch = joy->buttons[13];
//      this->l1 = joy->buttons[4];
//      this->r1 = joy->buttons[5];

//      this->arrowsX = joy->axes[9];
//      this->arrowsY = joy->axes[10];
//      this->l2 = joy->axes[3];
//      this->r2 = joy->axes[4];

//      this->leftStickX = joy->axes[0];
//      this->leftStickY = joy->axes[1];
//      this->rightStickX = joy->axes[2];
//      this->rightStickY = joy->axes[5];

      //printRaw();

      ///////////////////////Wireless///////////////////////

      this->button_X = joy->buttons[0];
      this->button_O = joy->buttons[1];
      this->button_Triangle = joy->buttons[2];
      this->button_Square = joy->buttons[3];
      this->button_left_right = joy->axes[6];
      this->button_up_down = joy->axes[7];


      this->button_L1 = joy->buttons[4];
      this->button_R1 = joy->buttons[5];
      this->button_Option = joy->buttons[9];
      this->button_Share = joy->buttons[8];


      this->buttonTouch = joy->buttons[13];

      this->stick_X_Axis = joy->axes[0];
      this->stick_Y_Axis = joy->axes[1];
      this->stick_Z_Axis = joy->axes[4];



      //printRaw();

    }

    bool calibrate()
    {
        double progress = ((double) this->calib1 / this->calib_duration) * 100;
        if( (this->l2 == -1.0) && (this->r2 == -1.0) )
        {
            ROS_WARN("Press L2 and R2 to calibrate: %i%% ", (int) progress);
            this->calib1++;
            this->calib2++;
        }
        else{
            this->calib1 = 0;
            this->calib2 = 0;
        }

        if( (this->calib1 > this->calib_duration) && (this->calib2 > this->calib_duration))
        {
            this->calib = true;
            return true;
        }
        else
            return false;
    }

    bool waitForRelease()
    {
        if( (this->l2 == 1.0) && (this->r2 == 1.0) )
        {
            return true;
        }
        else{
            return false;
        }
    }

    void printSend()
    {

        ROS_INFO("#####################################");
        ROS_INFO("Send L2: %f", this->send_l2);
        ROS_INFO("Send R2: %f", this->send_r2);
        ROS_INFO("##################################### \n");
    }

    void printRaw()
    {

        ROS_INFO("#####################################");
        ROS_INFO("Squared Button pressed: %i", this->button_Square);
        ROS_INFO("X Button pressed: %i", this->button_X);
        ROS_INFO("O Button pressed: %i", this->button_O);
        ROS_INFO("Triangel Button pressed: %i", this->button_Triangle);
        ROS_INFO("Touch Button pressed: %i", this->buttonTouch);
        ROS_INFO("L1: %i", this->button_L1);
        ROS_INFO("R1: %i", this->button_R1);
        ROS_INFO("L2: %f", this->l2);
        ROS_INFO("R2: %f", this->r2);

        ROS_INFO("##################################### \n");
    }

private:
    ros::Publisher chat;
    ros::Subscriber sub;

    /* calibration variables */
    int calib_duration = 20; // 1/10 sec
    int calib1, calib2;
    bool calib;

    /* raw data */
    double l2, r2;
    int button_Square, button_X, button_O, button_Triangle,
        buttonTouch, button_L1, button_R1, stick_X_Axis, stick_Y_Axis, stick_Z_Axis, button_Share, button_Option;
    int button_left_right, button_up_down;

    /* rosparams */
    double scale_linear, scale_angular;
    std::string pubName;

    double maxVel, maxVelR;

    /* send data */
    double send_leftStickX, send_l2, send_r2;

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "PS4_ROS");
    ros::NodeHandle n;

    // create ps4_ros object
    PS4_ROS *ps4_ros = new PS4_ROS(n);

    // calibrate
    ROS_WARN("Press L2 and R2 to calibrate");
    bool ready = false;
    while(!ready)
    {
        ready = ps4_ros->calibrate();
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    ROS_WARN("Release L2 and R2");
    ros::Duration(2.0).sleep();
    ready = false;
    while(!ready)
    {
        ready = ps4_ros->waitForRelease();
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Calibrated - Ready to use");

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ps4_ros->run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete ps4_ros;
	return 0;
}
