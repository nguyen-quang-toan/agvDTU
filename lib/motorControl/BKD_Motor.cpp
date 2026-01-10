#include "BKD_Motor.h"

Motor::Motor()
{
  Motor(type::H_STD, -1, -1, 0.0, true);
}

Motor::Motor(type hType, uint8_t pinDir, uint8_t pinPul)
{
  Motor(hType, pinDir, pinPul, 0.0, true);
  this->initialize();
}

Motor::Motor(type hType, uint8_t pinDir, uint8_t pinPul, double deadBand)
{
  Motor(hType, pinDir, pinPul, deadBand, true);
  this->initialize();
}

Motor::Motor(type hType, uint8_t pinDir, uint8_t pinPul, double deadBand, bool direction)
{
  this->hType = hType;
  this->pinDir = pinDir;
  this->pinPul = pinPul;
  this->deadBand = deadBand;
  this->direction = direction;
  this->initialize();
}

void Motor::initialize()
{
  this->isInitialized = true;
  pinMode(this->pinDir, OUTPUT);
  pinMode(this->pinPul, OUTPUT);
  this->setPower(0.0);
}

void Motor::setPower(double power)
{
  if(!this->isInitialized)
    return;
  if(abs(power) < this->deadBand)
  {
    power = 0;
  }

  if (power > +1.0) power = +1.0;
  if (power < -1.0) power = -1.0;
  this->powerInt = (int)(power*255);

  if(power == 0)
  {
    digitalWrite(this->pinDir, 1);
    digitalWrite(this->pinPul, 1);
  }
  else if (power > 0)
  {
    digitalWrite(this->pinDir, (this->direction) ? (LOW) : (HIGH));
    if(this->hType == H_PID && (abs(this->powerInt) != 3))
    {
      analogWrite(this->pinPul, 255-abs(this->powerInt));
    }
    else if(this->hType == H_STD)
    {
      analogWrite(this->pinPul, this->direction ? abs(this->powerInt) : (255-abs(this->powerInt)));
    }
  }
  else if(power < 0)
  {
    digitalWrite(this->pinDir, (this->direction) ? (HIGH) : (LOW));
    if(this->hType == H_PID && (abs(this->powerInt) != 3))
    {
      analogWrite(this->pinPul, 255-abs(this->powerInt));
    }
    else if(this->hType == H_STD)
    {
      analogWrite(this->pinPul, this->direction ? (255-abs(this->powerInt)) : abs(this->powerInt));
    }
  }
}

void Motor::setPower(boolean lock)
{
  if(this->hType == H_PID)
  {
    digitalWrite(this->pinDir, 0);
    analogWrite(this->pinPul, 255-3);
  }
}

uint8_t Motor::getpinDir()
{
  return this->pinDir;
}

uint8_t Motor::getpinPul()
{
  return this->pinPul;
}

int16_t Motor::getPowerInt()
{
  return this->powerInt;
}

void Motor::setDirection(bool direction)
{
  this->direction = direction;
}