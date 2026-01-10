#ifndef MOTOR
#define MOTOR

#include "Arduino.h"

class Motor
{
  public:
    typedef enum {
      H_PID = 0,
      H_STD = 1,
    } type;
    Motor();
    Motor(type hType, uint8_t pinDir, uint8_t pinPul);
    Motor(type hType, uint8_t pinDir, uint8_t pinPul, double deadBand);
    Motor(type hType, uint8_t pinDir, uint8_t pinPul, uint8_t direction);
    Motor(type hType, uint8_t pinDir, uint8_t pinPul, double deadBand, bool direction);

    uint8_t getpinDir();
    uint8_t getpinPul();
    int16_t getPowerInt();
    int16_t powerInt = 0;
    void setPower(double power);
    void setPower(boolean lock);
    void initialize();
    void setDirection(bool direction);
    bool isInitialized = false;
    double deadBand = 0.0;
  private:
    type hType;
    uint8_t pinDir;
    uint8_t pinPul;
    uint8_t direction;
};
#endif
