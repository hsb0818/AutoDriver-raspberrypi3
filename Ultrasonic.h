#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#define MAX_SENSOR_DISTANCE 500

class Ultrasonic
{
public:
  Ultrasonic(int trigger_pin, int echo_pin);
    void Init(int trigger_pin, int echo_pin);
    float ReadDistByCentimeters();
    float ReadDistByInch();

private:
    int m_trigger_pin;
    int m_echo_pin;
};

#endif
