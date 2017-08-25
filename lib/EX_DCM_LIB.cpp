// EX_DCM_LIB.cpp

#include "L298N.h"

#include <wiringPi.h>
#include <wiringSerial.h>
#include <piFace.h>
#include <softPwm.h>

// PiFace Digital 2
#define PIFACE_BASE     200

// OUTPUT
#define RUN_LED         (PIFACE_BASE)
#define ULTRASONIC_TRIG (PIFACE_BASE+1)
#define L298N_ENA       (PIFACE_BASE+2)
#define L298N_IN1       (PIFACE_BASE+3)
#define L298N_IN2       (PIFACE_BASE+4)
#define L298N_IN3       (PIFACE_BASE+5)
#define L298N_IN4       (PIFACE_BASE+6)
#define L298N_ENB       (PIFACE_BASE+7)

// INPUT
#define EXIT_SW         (PIFACE_BASE)
#define ULTRASONIC_ECHO (PIFACE_BASE+4)
#define PD_LEFT         (PIFACE_BASE+5)
#define PD_CENTER       (PIFACE_BASE+6)
#define PD_RIGHT        (PIFACE_BASE+7)


L298N mtDriver(L298N_ENA, L298N_IN1, L298N_IN2, L298N_ENB, L298N_IN3, L298N_IN4);

void setup();
void loop();

void setup()
{
    // wiringPi
    wiringPiSetupGpio();
    
    // PiFace
    piFaceSetup(PIFACE_BASE);
    
    mtDriver.begin();
}

void loop()
{
    mtDriver.goForward(); delay(2000);
    mtDriver.goBack(); delay(2000);
    mtDriver.goRight(); delay(2000);
    mtDriver.goForward(); delay(2000);
    mtDriver.goLeft(); delay(2000);
    mtDriver.goForward(); delay(2000);
    mtDriver.stop(); delay(3000);
}
    
int main ()
{
    setup();
    
    while(1)
    {
        loop();
    }

    return 0 ;
}
