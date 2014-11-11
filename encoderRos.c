#include "udoo_motor_control.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"


ros::NodeHandle nh;
int sentSpeed;
float rec;
int spd;


void distMessageHandler(const geometry_msgs::Twist& cmd_dist) {
  
  //PID_target = 128;
  rec = cmd_dist.linear.x ;
  PID_target = convertSpeed(cmd_dist.linear.x);
  sentSpeed = PID_target;
}

ros::Subscriber<geometry_msgs::Twist> distSubscription("cmd_dist", distMessageHandler);

geometry_msgs::Twist encoder_data;
ros::Publisher encoderData("encoder_data", &encoder_data);

void setup() {
  // put your setup code here, to run once:
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  // Begin serial and serial 1, which is how we are talking to the motor driver
  Serial1.begin(9600);

  // turn the PID on
  speedPID.SetMode(AUTOMATIC);
  speedPID.SetOutputLimits(PID_output_lower, PID_output_upper);
  speedPID.SetSampleTime(20);
  
  // ros stuff
  nh.initNode();
  nh.advertise(encoderData);
  nh.subscribe(velocitySubscription);
  // Set baud rate to handle Twist messages
  nh.getHardware()->setBaud(115200);
  
  setSpeedBoth(128);
}


void loop() {
  // The target speed is set in the subscriber callback
  
  // Compute the desired output speed to be sent to the motors
  speedPID.Compute();
  // get the speed from the encoders
  spd = getSpeed(1);
  // Set the PID input to the speed gotten from the encoders
  PID_input = spd;
  
//  Serial.println(spd);
  
  // Set the speed of the motors to the PID controller output
  setSpeedBoth(PID_output);
//  setSpeedBoth(sentSpeed);
  
  // Report the velocity on the ROS publisher
  encoder_data.linear.x = spd;
  encoder_data.linear.y = PID_output;
  encoder_data.linear.z = sentSpeed;
  encoder_data.angular.x = rec;
  //encoder_data.linear.x = sentSpeed;
  encoderData.publish(&encoder_data);
  nh.spinOnce();
  delay(20);
}
