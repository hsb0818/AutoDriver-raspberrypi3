#include "Ultrasonic.h"
#include <wiringPi.h>
#include <stdio.h>
#include <unistd.h>

Ultrasonic::Ultrasonic(int trigger_pin, int echo_pin)
{
    Init(trigger_pin, echo_pin);
}

void Ultrasonic::Init(int trigger_pin, int echo_pin)
{
    m_trigger_pin = trigger_pin;
    m_echo_pin = echo_pin;
}

float Ultrasonic::ReadDistByCentimeters()
{
    float distance = 0.0;

    digitalWrite(m_trigger_pin, LOW);
    usleep(2);
    digitalWrite(m_trigger_pin, HIGH);
    usleep(20);
    digitalWrite(m_trigger_pin, LOW);

    while(digitalRead(m_echo_pin) == LOW);    

    long start_time = micros();
    while(digitalRead(m_echo_pin) == HIGH);    
    long travel_time = micros();
    
    distance = (travel_time - start_time)/58;

    //printf("%f cm \n", distance);
    //delay(10);

    return distance;
}

float Ultrasonic::ReadDistByInch()
{
    return (ReadDistByCentimeters() / 2.54);
}
