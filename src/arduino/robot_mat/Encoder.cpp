#include "Encoder.h"

Encoder::Encoder(int pin_power,int pin_direction,int pin_position) : pid_(0.5,0.1,0.0) 
{
  //Set teh parameters to the hardware GIOP.
  this->pin_power_=pin_power;
  this->pin_direction_=pin_direction;
  this->pin_position_=pin_position;
  this->direction_= 1;
  this->duty_=0;

  //Setup the PID max velocity
  pid_.setOutputLimits(-max_tic_per_second_,max_tic_per_second_);
  this->is_stopped=true;
  setup_();
}

void Encoder::setup_() 
{
  pinMode(pin_power_, OUTPUT);
  pinMode(pin_direction_, OUTPUT);
  pinMode(pin_position_, INPUT_PULLUP);
}

void Encoder::move_velocity_(double velocity)
{
  
  this->velocity_encoder_demanded_=velocity;
  //Convert from velocity to duty
  this->duty_ = this->duty_ + ceil(velocity_encoder_demanded_*power_by_velocity_factor_);
  if (this->duty_<0)
	  this->duty_=0;
  if (this->duty_>1023)
	  this->duty_=1023;
/*
  Serial.print("move move_velocity_:");
  Serial.print("\t");
  Serial.print(velocity_encoder_demanded_);
  Serial.print("\t");
  Serial.print(this->duty_);
  Serial.print("\n");
*/
    
  //Send to the hardware both duty and direcction
  analogWrite(pin_power_,this->duty_); 
}

void Encoder::move(double velocity)
{
  this->is_stopped=false;
  this->velocity_encoder_target_=fabs(velocity);
  //Setup the PID target.
  pid_.setTarget(velocity_encoder_target_); 

  //Setup direction
  if( velocity >= 0 ) {
    digitalWrite(pin_direction_,LOW);
    this->direction_= 1;
  } else {
    digitalWrite(pin_direction_,HIGH);
    this->direction_= -1;
  } 
  //Move
  //move_velocity_(velocity_encoder_target_); 
}   

void Encoder::stop() 
{
  analogWrite(pin_power_,0);
  digitalWrite(pin_direction_,LOW);
  this->velocity_encoder_target_=0;
  this->is_stopped=true;
}

void Encoder::updateEncoder()
{
  this->encoder_=this->encoder_ + ( 1 * direction_) ;
/*
  Serial.print(encoder_);
  Serial.print("\n");
*/
}

void Encoder::updateControlLoop(double dt) 
{
  if (!this->is_stopped)
  {

    
    this->velocity_encoder_current_ =  (encoder_ - previous_encoder_) / dt;
    this->previous_encoder_=encoder_;     
    
    /*
    Serial.print("updateControlLoop:");
    Serial.print("\t");
    */
    if ( pin_power_ == 4 ) {      
      Serial.print(dt);
      Serial.print("\t");
      Serial.print(velocity_encoder_target_);
      Serial.print("\t");
      Serial.print(velocity_encoder_current_);
      Serial.print("\t");
      Serial.print(velocity_encoder_demanded_);
      Serial.print("\n");    
    }
  
    //Update the current velocity.
    pid_.setInput(this->velocity_encoder_current_);
    //Update the demanded velocity.
    this->velocity_encoder_demanded_ = pid_.compute(dt);
    //Move demanded.
    move_velocity_(velocity_encoder_demanded_);
  }
}

long Encoder::getEncoder() 
{
  return this->encoder_;
}

long Encoder::getEncoderVelocityTarget() 
{
  return this->velocity_encoder_target_;
}

long Encoder::getEncoderVelocityDemanded() 
{
  return this->velocity_encoder_demanded_;
}

long Encoder::getEncoderVelocityCurrent() 
{
  return this->velocity_encoder_current_;
}
