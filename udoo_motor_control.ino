#include "udoo_motor_control.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"

ros::NodeHandle nh;

void velocityMessageHandler(const geometry_msgs::Twist& cmd_vel) {
  PID_target = cmd_vel.linear.x;
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
  speedPID.SetOutputLimits(0, 255);
  speedPID.SetSampleTime(20);
  
  // ros stuff
  nh.initNode();
  nh.advertise(velocityReporter);
  nh.subscribe(velocitySubscription);
  // Set baud rate to handle Twist messages
  nh.getHardware()->setBaud(115200);
}


void loop() {
  // The target speed is set in the subscriber callback
  
  // Compute the desired output speed to be sent to the motors
  speedPID.Compute();
  // get the speed from the encoders
  int spd = getSpeed(1);
  // Set the PID input to the speed gotten from the encoders
  PID_input = spd;
  //Serial.println(spd);
  
  // Set the speed of the motors to the PID controller output
  setSpeedBoth(PID_output);
  
  // Report the velocity on the ROS publisher
  reported_velocity.linear.x = spd;
  velocityReporter.publish(&reported_velocity);
  nh.spinOnce();
  delay(20);
}
