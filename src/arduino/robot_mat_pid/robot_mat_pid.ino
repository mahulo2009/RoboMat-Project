#include <ESP8266WiFi.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>
#include "network_connection.h"
#include <WheelEncoder.h>

#define ENCODER_TICKS_PER_REVOLUTION 21
#define MAX_SPEED 10.471975511965978
#define POWER_MIN 0
#define POWER_MAX 1023
#define PIN_ENCODER 12
#define PIN_POWER 4
#define PIN_DIRECTION 2

//Pid
Pid * pid = new Pid();
//Encoders
Encoder * encoder = new Encoder(ENCODER_TICKS_PER_REVOLUTION);
//Wheel
WheelEncoder wheel(MAX_SPEED);

IPAddress server(192, 168, 1, 40); // ip of your ROS server
const uint16_t serverPort = 11411;

void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}

//Ros node handler
ros::NodeHandle nh;

void pid_configCallback( const geometry_msgs::Vector3& pid_params) {
  nh.logdebug("pid_configCallback");

  pid->reset();
  pid->setKp(pid_params.x);
  pid->setKi(pid_params.y);
  pid->setKd(pid_params.z);

  wheel.stop();
  wheel.move(8.0);
}
ros::Subscriber<geometry_msgs::Vector3> cmd_pid_sub("/car/pid_params", &pid_configCallback );


geometry_msgs::Vector3 pid_telemetry_msg;              
ros::Publisher pid_telemetry_pub("/car/pid_telemetry", &pid_telemetry_msg); 

void setup() {
  //Setup Serial line.
  Serial.begin(115200);
  //Attach pins
  wheel.attachPower(PIN_POWER,POWER_MIN,POWER_MAX);
  wheel.attachDirection(PIN_DIRECTION);
  //Attach Encoder Left
  encoder->attach(PIN_ENCODER);
  wheel.attachEncoder(encoder);
  //Attach Pid
  pid->setMaxWindup(MAX_SPEED);
  pid->setAlpha(1.0);
  wheel.attachPid(pid);

  //Setup Wifi
  setupWiFi();
  delay(2000);
  //Init rose node & subscribe
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(cmd_pid_sub);
  nh.advertise(pid_telemetry_pub);

  while(!nh.connected()) {nh.spinOnce();};
}

void loop() { 
	if (nh.connected()) {             	         
    pid_telemetry_msg.x=wheel.getTargetVelocity();
    pid_telemetry_msg.y=wheel.getVelocity();
    pid_telemetry_msg.z=wheel.getDemandedVelocity();
    
    pid_telemetry_pub.publish(&pid_telemetry_msg);
    
    nh.spinOnce(); 
    delay(250);  
  } 
}
