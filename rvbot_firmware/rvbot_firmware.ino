#include <SoftwareSerial.h>
#include <ros.h>
#include <rvbot_bringup/MotorSpeed.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"


//MPU6050 Accelerometer 
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


#define LeftEN 2          //green     ENA
#define RightEN 3         //orange      ENB

#define LeftForward 25    //green        IN1
#define LeftBackward 23   //red    IN2
#define RightForward 24   //yellow    IN3
#define RightBackward 22  //orange     IN4

#define LEncoderA 27      //grey 
#define LEncoderB 29      //yellow
#define REncoderA 26      //blue
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
std_msgs::String imu_msg;

ros::Publisher lencoder("lwheel", &le);
ros::Publisher rencoder("rwheel", &re);
ros::Publisher imu("/serial_imu", &imu_msg);

void forward(float lspeed, float rspeed)
{ 
  int ls = abs(lspeed);
  int rs = abs(rspeed);
  int ls_new = map(lspeed,0,255,75,255);
  int rs_new = map(lspeed,0,255,75,255);
  //analogWrite(LeftEN, ls_new);

  digitalWrite(LeftForward, HIGH);
  digitalWrite(LeftBackward, LOW);

  //analogWrite(RightEN, rs_new);

  digitalWrite(RightForward, HIGH);
  digitalWrite(RightBackward, LOW);

}

void right(float lspeed, float rspeed)
{  
  int ls = abs(lspeed);
  int rs = abs(rspeed);
  int ls_new = map(lspeed,0,255,75,255);
  int rs_new = map(lspeed,0,255,75,255);
  //analogWrite(LeftEN, ls_new);

  digitalWrite(LeftForward, HIGH);
  digitalWrite(LeftBackward, LOW);

  //analogWrite(RightEN, rs_new);

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

void left(float lspeed, float rspeed)
{ 
  int ls = abs(lspeed);
  int rs = abs(rspeed);
  int ls_new = map(lspeed,0,255,75,255);
  int rs_new = map(lspeed,0,255,75,255);
  
  //analogWrite(LeftEN, ls_new);
  
  digitalWrite(LeftForward, LOW);
  digitalWrite(LeftBackward, HIGH);

  //analogWrite(RightEN, rs_new);

  digitalWrite(RightForward, HIGH);
  digitalWrite(RightBackward, LOW);

}

void backward(float lspeed, float rspeed)
{ 
  int ls = abs(lspeed);
  int rs = abs(rspeed);
  int ls_new = map(lspeed,0,255,75,255);
  int rs_new = map(lspeed,0,255,75,255);
  
  //analogWrite(LeftEN, ls_new);
  

  digitalWrite(LeftForward, LOW);
  digitalWrite(LeftBackward, HIGH);

  //analogWrite(RightEN, rs_new);
  

  digitalWrite(RightForward, LOW);
  digitalWrite(RightBackward, HIGH);

}

void callback(const rvbot_bringup::MotorSpeed& motor_speeds)
{

  if (motor_speeds.lspeed > 0 && motor_speeds.rspeed > 0)
  {
    forward(motor_speeds.lspeed, motor_speeds.rspeed); 
  }
  if (motor_speeds.lspeed < 0 && motor_speeds.rspeed > 0)
  {
      left(motor_speeds.lspeed, motor_speeds.rspeed);
  }
  if (motor_speeds.lspeed > 0 && motor_speeds.rspeed < 0)
  {
      right(motor_speeds.lspeed, motor_speeds.rspeed);
  }
  if (motor_speeds.lspeed < 0 && motor_speeds.rspeed < 0)
  {
      backward(motor_speeds.lspeed, motor_speeds.rspeed);
  }
  if (motor_speeds.lspeed == 0 && motor_speeds.rspeed == 0)
  {
    stop();
  }

//Serial.print("Left Speed: ");
//Serial.print(motor_speeds.lspeed); 
//Serial.print("Right Speed: ");
//Serial.print(motor_speeds.rspeed);  
}

ros::Subscriber <rvbot_bringup::MotorSpeed> sub("motor_speeds", callback);


void setup()
{
  Wire.begin();
  nh.initNode();
  nh.subscribe(sub);

  //Serial.begin(115200);

  //imu initialize
  accelgyro.initialize();
  accelgyro.setI2CBypassEnabled(true);
  
  pinMode (LEncoderA, INPUT);
  pinMode (LEncoderB, INPUT);
  pinMode (REncoderA, INPUT);
  pinMode (REncoderB, INPUT);

  LeftLastState = digitalRead(LEncoderA);
  RightLastState = digitalRead(REncoderA);
  nh.advertise(lencoder);
  nh.advertise(rencoder);
  nh.advertise(imu);

  analogWrite(LeftEN, motorspeed);
analogWrite(RightEN, motorspeed);

}

void loop()
{
  LeftState = digitalRead(LEncoderA);
  RightState = digitalRead(REncoderA);
  
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

  le.data = LeftCounter;
    lencoder.publish(&le);
    //Serial.print("Left Position: ");
    //Serial.print(LeftCounter);
    //Serial.print("\t");
    //Serial.print("Right Position: ");
    //Serial.println(RightCounter);
    re.data = RightCounter;
    rencoder.publish(&re);

  LeftLastState = LeftState;
  RightLastState = RightState;


  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  String AX = String(ax);
  String AY = String(ay);
  String AZ = String(az);
  String GX = String(gx);
  String GY = String(gy);
  String GZ = String(gz);
  String data = "A" + AX + "B"+ AY + "C" + AZ + "D" + GX + "E" + GY + "F" + GZ + "G" ;
  int length = data.indexOf("G") +2;
  char data_final[length+1];
  data.toCharArray(data_final, length+1);
  imu_msg.data = data_final;
  imu.publish(&imu_msg);
  
  nh.spinOnce();
  
}
