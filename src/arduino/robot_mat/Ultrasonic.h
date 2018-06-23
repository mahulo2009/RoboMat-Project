#ifndef Ultrasonic_H
#define Ultrasonic_H

#include <ESP8266WiFi.h>
#include <Servo.h>

/**
 * 
 */
class Ultrasonic {

  public:
    /**
     * 
     */
    Ultrasonic(int pin_trigger,int pin_echo);
    /*
     * 
     */
    double updateDistance();
    /**
     * 
     */
    inline double distance() {
      return distance_;
    }

    void move(double pos);

  private:
  
    /**
     * 
     */
    Servo servoMotor_;
  
    /**
     * 
     */
    int pin_trigger_;
    /**
     * 
     */
    int pin_echo_;
    /**
     * 
     */
    double distance_;  
};
#endif
