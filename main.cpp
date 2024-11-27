#include <Arduino.h>
#include "PinMap.h"
#include <micro_ros_platformio.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>

// ROS Client Libraries
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS Message Libraries
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
// ...

// Other Libraries
#include <utilities/rcl_handler.h>
// ...

/* ----------------------------Global Variables----------------------------- */

// ROS Client Variables
rclc_support_t support;
rcl_allocator_t allocator;

// ROS Node Variables
rcl_node_t node;

// ROS Executor Variables (Can be more than one)
rclc_executor_t executor;

// ROS subscriber
rcl_subscription_t cmdvel_subscriber;
geometry_msgs_msg_Twist msg_cmdvel;

// ROS publisher
rcl_publisher_t odom_publisher;
nav_msgs_msg_Odometry msg_odom;

// Timer for BNO publisher
rcl_timer_t odom_timer;


int dir[3][4]={{1,1,1,1},{-1,1,-1,1},{-1,-1,1,1}};
int vxg, vyg; //Global Velocities
int direction[4]; //Initializing an array for direction
int pwm[4]; //Initializing an array for PWM
int mot_vel[4]; //Initializing an array for motor speeds
int vx,vy,vw; //Local Velocities
float odom_angle;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); //BNO initialization

// put function declarations here:
void cmdvel_callback(const void *msgin){
  // subscribes from cmd_vel topic and feeds it to motors
  vxg = (float) msg_cmdvel.linear.x;
  vyg = (float) msg_cmdvel.linear.y;
  vw = (float) msg_cmdvel.angular.z;
  vx=vxg; // If the velocity given through cmd_vel is already local, then these two lines
  vy=vyg; // of code come in play. If not, they get overwritten by the rotation function
}

void rotation(float theta){
  // Remember to comment out this function if calculation happens in ROS already
  theta = theta*PI/180;
  vx = vxg*cos(theta) + vyg*sin(theta);
  vy = vyg*cos(theta) - vxg*sin(theta);
  return;
}

void multiplication(int vx,int vy,int vw)
{
  //Motor velocity matrix
  mot_vel[0]=vx*dir[0][0]+vy*dir[1][0]+vw*dir[2][0];
  mot_vel[1]=vx*dir[0][1]+vy*dir[1][1]+vw*dir[2][1];
  mot_vel[2]=vx*dir[0][2]+vy*dir[1][2]+vw*dir[2][2];
  mot_vel[3]=vx*dir[0][3]+vy*dir[1][3]+vw*dir[2][3];
}

void direction_pwm()
{ 
  for (int i =0;i<4;i++)
  {
    if (mot_vel[i] >0) 
    {direction[i] = 0;}
    else{
      direction[i] = 1;
    }
    pwm[i] = map(abs(mot_vel[i]),0,10,0,100);
  }
}

void odom_callback(rcl_timer_t *timer, int64_t last_call_time){
    /* Get a new sensor event */ 
    sensors_event_t event; 
    bno.getEvent(&event);

    odom_angle = event.orientation.x;

    msg_odom.pose.pose.orientation.z = odom_angle;
    RCCHECK(rcl_publish(&odom_publisher, &msg_odom, NULL)); 
}

void setup() {
    // Setup scripts for hardware
    pinMode(MD1_DIR,OUTPUT);
    pinMode(MD2_DIR,OUTPUT);
    pinMode(MD3_DIR,OUTPUT);
    pinMode(MD4_DIR,OUTPUT);
    pinMode(MD1_PWM,OUTPUT);
    pinMode(MD2_PWM,OUTPUT);
    pinMode(MD3_PWM,OUTPUT);
    pinMode(MD4_PWM,OUTPUT);

      /* Initialise the sensor */
    if(!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1); 
    }

    //Setup scripts for microROS
    set_microros_serial_transports(Serial);
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create node
    RCCHECK(rclc_node_init_default(&node, "bot_mcu", "", &support));

    // Create executor (Here 1 is the number of handles. Change if required)
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    //Create subscriber
    const char *cmdvel_name = "/cmd_vel";
    const rosidl_message_type_support_t *Twist_type =
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);

    //Create publisher
    const char *odom_name = "/odom";
    const rosidl_message_type_support_t *Odometry_type = 
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry);

    // create cmd_vel subscriber
    RCCHECK(rclc_subscription_init_default(&cmdvel_subscriber, &node, Twist_type, cmdvel_name));
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmdvel_subscriber, &msg_cmdvel, &cmdvel_callback, ON_NEW_DATA));

    // create odom publisher
    RCCHECK(rclc_publisher_init_default(
      &odom_publisher,
      &node,
      Odometry_type, odom_name));


    float timer_period = RCL_MS_TO_NS(100);

    // timer for odom publisher
    RCCHECK(rclc_timer_init_default(&odom_timer, &support, timer_period, odom_callback));
    // RCCHECK(rclc_executor_init(&odom_executor, &support.context, 1, &allocator));
    rclc_executor_add_timer(&executor, &odom_timer);
    
    bno.setExtCrystalUse(true);    
}

void loop() {

    // Spin the executors
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

    // put your main code here, to run repeatedly:
    //rotation(event.orientation.x); // Remember to comment out this function if calculation happens in ROS already
    multiplication(vx,vy,vw);
    direction_pwm();
    digitalWrite(MD1_DIR,direction[0]);
    digitalWrite(MD2_DIR,direction[1]);
    digitalWrite(MD3_DIR,direction[2]);
    digitalWrite(MD4_DIR,direction[3]);
    analogWrite(MD1_PWM,pwm[0]);
    analogWrite(MD2_PWM,pwm[1]);
    analogWrite(MD3_PWM,pwm[2]);
    analogWrite(MD4_PWM,pwm[3]);
}