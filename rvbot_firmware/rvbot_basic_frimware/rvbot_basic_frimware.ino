#include <SoftwareSerial.h>
#include <ros.h>
#include <rvbot_bringup/MotorSpeed.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>


#define LeftEN 2          //green     ENA
#define RightEN 3         //orange      ENB

#define LeftForward 25    //green   //red    //grey     IN1
#define LeftBackward 23   //red     //green  //black        IN2
#define RightForward 24   //yellow  //orange //white         IN3
#define RightBackward 22  //orange  //yellow //yeoolow         IN4

#define LEncoderA 27      //grey  //brown
#define LEncoderB 29      //yellow
#define REncoderA 26      //blue    //red
#define REncoderB 28      //purple

#define motorspeed 75



int LeftCounter = 0;
int LeftState;
int LeftLastState;
int RightCounter = 0;
int RightState;
int RightLastState;

ros::NodeHandle nh;
std_msgs::Int64 le;
std_msgs::Int64 re;
geometry_msgs::Twist msg;

ros::Publisher lencoder("lwheel", &le);
ros::Publisher rencoder("rwheel", &re);

void forward()
{ 

  analogWrite(LeftEN, motorspeed);

  digitalWrite(LeftForward, HIGH);
  digitalWrite(LeftBackward, LOW);

  analogWrite(RightEN, motorspeed);

  digitalWrite(RightForward, HIGH);
  digitalWrite(RightBackward, LOW);

}

void right()
{  

  analogWrite(LeftEN, motorspeed);

  digitalWrite(LeftForward, HIGH);
  digitalWrite(LeftBackward, LOW);

  analogWrite(RightEN, motorspeed);

  digitalWrite(RightForward, LOW);
  digitalWrite(RightBackward, HIGH);

}

void stop()
{
  digitalWrite(LeftForward, LOW);
  digitalWrite(LeftBackward, LOW);

  digitalWrite(RightForward, LOW);
  digitalWrite(RightBackward, LOW);

}

void left()
{ 
  analogWrite(LeftEN, motorspeed);

  digitalWrite(LeftForward, LOW);
  digitalWrite(LeftBackward, HIGH);

  analogWrite(RightEN, motorspeed);

  digitalWrite(RightForward, HIGH);
  digitalWrite(RightBackward, LOW);

}

void backward()
{ 
 
  analogWrite(LeftEN, motorspeed);

  digitalWrite(LeftForward, LOW);
  digitalWrite(LeftBackward, HIGH);

  analogWrite(RightEN, motorspeed);

  digitalWrite(RightForward, LOW);
  digitalWrite(RightBackward, HIGH);

}

void callback(const geometry_msgs::Twist& cmd_vel)
{
  
  if (cmd_vel.linear.x == 0 && cmd_vel.angular.z == 0)
  {
    stop(); //k
  }
  else if (cmd_vel.linear.x > 0 && cmd_vel.angular.z == 0)
  {
    forward(); //i
  }
  else if (cmd_vel.linear.x == 0 && cmd_vel.angular.z > 0)
  {
    left(); //j
  }
  else if (cmd_vel.linear.x == 0 && cmd_vel.angular.z < 0)
  {
    right(); //l
  }
  else if (cmd_vel.linear.x < 0 && cmd_vel.angular.z == 0)
  {
    backward(); //
  }
  else
  {
    stop(); //default
  }

//Serial.print("Left Speed: ");
//Serial.print(motor_speeds.lspeed); 
//Serial.print("Right Speed: ");
//Serial.print(motor_speeds.rspeed);  
}

ros::Subscriber <geometry_msgs::Twist> sub("cmd_vel", callback);


void setup()
{
  nh.initNode();
  nh.subscribe(sub);

  //Serial.begin(115200);

  pinMode (LEncoderA, INPUT);
  pinMode (LEncoderB, INPUT);
  pinMode (REncoderA, INPUT);
  pinMode (REncoderB, INPUT);

  LeftLastState = digitalRead(LEncoderA);
  RightLastState = digitalRead(REncoderA);
  nh.advertise(lencoder);
  nh.advertise(rencoder);

  analogWrite(LeftEN, motorspeed);
analogWrite(RightEN, motorspeed);

}

void loop()
{
  LeftState = digitalRead(LEncoderA);
  if (LeftState != LeftLastState)
  {
    if (digitalRead(LEncoderB) != LeftState)
    {
      LeftCounter ++;
    }
    else
    {
      LeftCounter --;
    }
    
  }
  le.data = LeftCounter;
    lencoder.publish(&le);
      LeftLastState = LeftState;

  
  RightState = digitalRead(REncoderA);
  if (RightState != RightLastState)
  {
    if (digitalRead(REncoderB) != RightState)
    {
      RightCounter --;
    }
    else
    {
      RightCounter ++;
    }
    
  }
  re.data = RightCounter;
    rencoder.publish(&re);


  RightLastState = RightState;

  
    //Serial.print("Left Position: ");
    //Serial.print(LeftCounter);
    //Serial.print("\t");
    //Serial.print("Right Position: ");
    //Serial.println(RightCounter);
    
  
  nh.spinOnce();
  
}
