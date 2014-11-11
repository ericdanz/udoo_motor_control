#include "udoo_motor_control.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"


ros::NodeHandle nh;
int sentSpeed;
float rec;
int spd;
//std_msgs::Float64 linearx;

void velocityMessageHandler(const geometry_msgs::Twist& cmd_vel) {
  
  //PID_target = 128;
  rec = cmd_vel.linear.x - 1.0;
  PID_target = convertSpeed(cmd_vel.linear.x);
  sentSpeed = PID_target;
}

ros::Subscriber<geometry_msgs::Twist> velocitySubscription("cmd_vel", velocityMessageHandler);

geometry_msgs::Twist reported_velocity;
ros::Publisher velocityReporter("velocity_reporter", &reported_velocity);

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
  nh.advertise(velocityReporter);
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
  reported_velocity.linear.x = spd;
  reported_velocity.linear.y = PID_output;
  reported_velocity.linear.z = sentSpeed;
  reported_velocity.angular.x = rec;
  //reported_velocity.linear.x = sentSpeed;
  velocityReporter.publish(&reported_velocity);
  nh.spinOnce();
  delay(20);
}
