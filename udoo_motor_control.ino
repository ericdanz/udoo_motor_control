#include "udoo_motor_control.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Float64.h>

ros::NodeHandle nh;
int sentSpeed;
float rec;
int spd;
int turnAmount;

long encoder1;
long encoder2;
long encoder1target;
long encoder2target;
int distanceForward = 0;
int degreesTurn = 0;


float mpe = 0.8722;
float turnRadiusConstant = 1;


//std_msgs::Float64 linearx;

void velocityMessageHandler(const geometry_msgs::Twist& cmd_vel) {
  
  //check if cmd_vel is sending velocities
  if(cmd_vel.linear.x != 128)
{
  	spd = cmd_vel.linear.x;
        //only go for one second
	setSpeedBoth(spd);
	
}
//if there is no forward velocity, check for turning
else if (cmd_vel.angular.z !=128)
{
	//do some kind of turn
	turnAmount = cmd_vel.angular.z;
	Turn(turnAmount);
}
}

ros::Subscriber<geometry_msgs::Twist> velocitySubscription("cmd_vel", velocityMessageHandler);

geometry_msgs::Twist reported_velocity;
ros::Publisher velocityReporter("velocity_reporter", &reported_velocity);

void distMessageHandler(const geometry_msgs::Twist& cmd_dist) {
	
	distanceForward = cmd_dist.linear.x;	
	degreesTurn = cmd_dist.angular.z;
  
}

ros::Subscriber<geometry_msgs::Twist> distSubscription("cmd_dist", distMessageHandler);

geometry_msgs::Twist reported_dist;
ros::Publisher distReporter("dist_reporter", &reported_dist);

void setup() {
  // put your setup code here, to run once:
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  // Begin serial and serial 1, which is how we are talking to the motor driver
  Serial1.begin(9600);

  // turn the PID on
  enc1PID.SetMode(AUTOMATIC);
  enc1PID.SetOutputLimits(PID_output_lower, PID_output_upper);
  enc1PID.SetSampleTime(20);
  
  enc2PID.SetMode(AUTOMATIC);
  enc2PID.SetOutputLimits(PID_output_lower, PID_output_upper);
  enc2PID.SetSampleTime(20);
  
  // ros stuff
  nh.initNode();
  nh.advertise(velocityReporter);
  nh.subscribe(velocitySubscription);

  nh.advertise(distReporter);
  nh.subscribe(distSubscription);
  // Set baud rate to handle Twist messages
  nh.getHardware()->setBaud(115200);
  
  setSpeedBoth(128);

  PID1_input = 0;
  PID1_output = 0;
  PID1_target = 0;

  PID2_input = 0;
  PID2_output = 0;
  PID2_target = 0;

  resetEncoders();
}


void loop() {
  reported_dist.linear.z = distanceForward;
  encoder1 = getEncoder(1);
  encoder2 = getEncoder(2);

   if(distanceForward != 0)
{
   //add the new forward distance to both encoder targets
   encoder1target = encoder1 + (distanceForward/mpe);
   encoder2target = encoder2 + (distanceForward/mpe);
   //reset distance forward
   distanceForward = 0;
   PID1_target = encoder1target;
   PID2_target = encoder2target;
}
if(degreesTurn != 0)
{
  //find a value to multiply 
  encoder1target = encoder1 + (degreesTurn/2)*turnRadiusConstant;
  encoder2target = encoder2 - (degreesTurn/2)*turnRadiusConstant;
  degreesTurn = 0;
  PID1_target = encoder1target;
  PID2_target = encoder2target;
}
   
   // The target speed is set in the subscriber callback
   
  // Compute the desired output speed to be sent to the motors
  enc1PID.Compute();
  enc2PID.Compute();
  // get the speed from the encoders
  //spd = getSpeed(1);
  // Set the PID input to the speed gotten from the encoders
  PID1_input = encoder1;
  PID2_input = encoder2;
//  Serial.println(spd);
  
  // Set the speed of the motors to the PID controller output
  setSpeed1(PID1_output);
  setSpeed1(PID2_output);
//  setSpeedBoth(sentSpeed);
  
  // Report the velocity on the ROS publisher
  reported_velocity.linear.x = getSpeed(1);
/*  reported_velocity.linear.y = PID_output;*/
/*  reported_velocity.linear.z = sentSpeed;*/
/*  reported_velocity.angular.x = rec;*/
  //reported_velocity.linear.x = sentSpeed;
  velocityReporter.publish(&reported_velocity);

  // Report the dist on the ROS publisher
  // Change to forward distance and total turns? 
  reported_dist.linear.x = encoder1*mpe;
  reported_dist.linear.y = encoder2*mpe;
  
  distReporter.publish(&reported_dist); 

  nh.spinOnce();
  delay(20);
}
