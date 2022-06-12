/*Written by Dean 4/23/20*/
//incoming command will be between -pi and pi and needs to be converted to XX-XXX
//fl servo = pin 9
//fr servo = pin 6
//bl servo = pin 5
//br servo = pin 3

//fl potfeedback = pin A1
//fr potfeedback = pin A2
//bl potfeedback = pin A3
//br potfeedback = pin A4

#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <Servo.h>

ros::NodeHandle nh;

std_msgs::Float64 float_msg_fl;
std_msgs::Float64 float_msg_fr;
std_msgs::Float64 float_msg_bl;
std_msgs::Float64 float_msg_br;

ros::Publisher pub_fl("true_position_fl", &float_msg_fl);
ros::Publisher pub_fr("true_position_fr", &float_msg_fr);
ros::Publisher pub_bl("true_position_bl", &float_msg_bl);
ros::Publisher pub_br("true_position_br", &float_msg_br);


Servo myservo_fl;
Servo myservo_fr;
Servo myservo_bl;
Servo myservo_br;

// Determine if the motor rotate
bool b_rotate = true;

// For tire alignment. The raw anlog value from Arduino.
// potentiometer value should be 0 to 1023. But now the voltage is not 5v. being 0 to 625. not exactly -pi/2, pi/2. Should be 0 to 840
// The range is [-2pi, 2pi]. We only use [-pi, pi] for left pi/2, right -pi/2 turning.
// The below default values should be within a range, so that the maximum turning (-pi/2 or pi/2) will not exceed the limit.
#define FL_VALUE 293
#define FR_VALUE 359
#define BL_VALUE 312
#define BR_VALUE 266
// [-pi/2, pi/2] corresponds to [0,840]
#define DENOM 840.0 * 2 * 3.141592
int pin_fl_value;
int pin_fr_value;
int pin_bl_value;
int pin_br_value;

void callback_fl(const std_msgs::Float64 &fl_command)
{
  int fl_int; //needs to be initialized outside of a loop
  if (abs(fl_command.data) > 15 && b_rotate)
  { //15 seems to be the highest number that will create motion
    fl_int = 93 + fl_command.data; //converts the float in the command to a value that can be read by the .write command
  }
  else
  { //so if the number is under 18, dont do anything (this prevents a lot of unecessary current flow)
    fl_int = 93;
  }
  myservo_fl.write(fl_int);
}

void callback_fr(const std_msgs::Float64 &fr_command)
{
  int fr_int; //needs to be initialized outside of a loop
  if (abs(fr_command.data) > 15 && b_rotate)
  { //15 seems to be the highest number that will create motion
    fr_int = 93 + fr_command.data; //converts the float in the command to a value that can be read by the .write command
  }
  else
  { //so if the number is under 18, dont do anything (this prevents a lot of unecessary current flow)
    fr_int = 93;
  }
  myservo_fr.write(fr_int);
}

void callback_bl(const std_msgs::Float64 &bl_command)
{
  int bl_int; //needs to be initialized outside of a loop
  if (abs(bl_command.data) > 15 && b_rotate)
  { //15 seems to be the highest number that will create motion
    bl_int = 93 + bl_command.data; //converts the float in the command to a value that can be read by the .write command
  }
  else
  { //so if the number is under 18, dont do anything (this prevents a lot of unecessary current flow)
    bl_int = 93;
  }
  myservo_bl.write(bl_int);
}

void callback_br(const std_msgs::Float64 &br_command)
{
  int br_int; //needs to be initialized outside of a loop
  if (abs(br_command.data) > 15 && b_rotate)
  { //15 seems to be the highest number that will create motion
    br_int = 93 + br_command.data; //converts the float in the command to a value that can be read by the .write command
  }
  else
  { //so if the number is under 18, dont do anything (this prevents a lot of unecessary current flow)
    br_int = 93;
  }
  myservo_br.write(br_int);
}

ros::Subscriber<std_msgs::Float64> sub_fl("arduino/fl_steering/control_effort", callback_fl);
ros::Subscriber<std_msgs::Float64> sub_fr("arduino/fr_steering/control_effort", callback_fr);
ros::Subscriber<std_msgs::Float64> sub_bl("arduino/bl_steering/control_effort", callback_bl);
ros::Subscriber<std_msgs::Float64> sub_br("arduino/br_steering/control_effort", callback_br);

void setup()
{
  Serial.begin(57600);
  nh.getHardware()->setBaud(57600);
  nh.initNode();

  nh.advertise(pub_fl);
  nh.advertise(pub_fr);
  nh.advertise(pub_bl);
  nh.advertise(pub_br);

  nh.subscribe(sub_fl);
  nh.subscribe(sub_fr);
  nh.subscribe(sub_bl);
  nh.subscribe(sub_br);

  myservo_fl.attach(9); //defines pin for the servo output
  myservo_fr.attach(6);
  myservo_bl.attach(5);
  myservo_br.attach(3);

  myservo_fl.write(93); //starts wheels at speed 0 (I found that 90 wasn't quite 0)
  myservo_fr.write(93);
  myservo_bl.write(93);
  myservo_br.write(93);
}

void loop()
{
  // A1 is the Pin number is a value from 0-1023 (linear scale with 0-5 V) // YQ: found to be 0-625
  // scales to voltage to radians and adds small offset if needed
  // float_msg_fl.data = fl_int; //used to test it is publishing correctly. replace this with analog read
  pin_fl_value = analogRead(A1);
  pin_fr_value = analogRead(A2);
  pin_bl_value = analogRead(A3);
  pin_br_value = analogRead(A4);

  /* Dean original
    // float_msg_fl.data = (pin_fl_value * (5.0 / 1023.0) - 1.5) * 3.14159 / 2;
    // float_msg_fr.data = (pin_fr_value * (5.0 / 1023.0) - 1.5) * 3.14159 / 2;
    // float_msg_bl.data = (pin_bl_value * (5.0 / 1023.0) - 1.5) * 3.14159 / 2;
    // float_msg_br.data = (pin_br_value * (5.0 / 1023.0) - 1.5) * 3.14159 / 2 + .017; */

  // If emergency stop, the values will be near 0, which generates wrong wheel positions.
  // Then the PID will generate wrong control commands.
  // Thus, do not publish these values and the servos do not move.

  if ( nh.connected() )
  {
    b_rotate = true;
    float_msg_fl.data = (pin_fl_value - FL_VALUE) / DENOM;
    float_msg_fr.data = (pin_fr_value - FR_VALUE) / DENOM;
    float_msg_bl.data = (pin_bl_value - BL_VALUE) / DENOM;
    float_msg_br.data = (pin_br_value - BR_VALUE) / DENOM;

    pub_fl.publish(&float_msg_fl); //publish voltages of potentiometers
    pub_fr.publish(&float_msg_fr);
    pub_bl.publish(&float_msg_bl);
    pub_br.publish(&float_msg_br);
  }
  else
  { //if connection is lost, send wheels to 90 degrees
    b_rotate = false;
    myservo_fl.write(93); //starts wheels at speed 0 (I found that 90 wasn't quite 0)
    myservo_fr.write(93);
    myservo_bl.write(93);
    myservo_br.write(93);
  }

  nh.spinOnce(); //enters the call back functions if there are messages on them
  delay(10);
}