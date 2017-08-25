/**************************************************
 * RNT-NTRD  SHK
 * 2015. 3. 05 Ver.01 
 **************************************************/

#ifndef _L298N_H__
#define _L298N_H__

#include <inttypes.h>

/*
#define DEF_SPEED	25
#define MAX_SPEED       100
#define NOR_SPEED       70
#define MIN_SPEED       30

#define PWM_PERIOD      100
#define PWM_STOP        0
#define PWM_MAX         (MAX_SPEED)
*/

#define DEF_SPEED	13
#define MAX_SPEED       100
#define ROT_SPEED       60
#define NOR_SPEED       40
#define MIN_SPEED       13

#define PWM_PERIOD      100
#define PWM_STOP        0
#define PWM_MAX         (MAX_SPEED)

class L298N
{
public:
    //- Constructors:
    L298N(int enAPin, int en1Pin, int en2Pin, int enBPin, int en3Pin, int en4Pin);
	
	//- Functions:
	void begin(void);
	void goForward(int nSpeed = DEF_SPEED);
	void goBack(int nSpeed = DEF_SPEED);
	void goLeft(void);
	void goRight(void);
	void stop(void);

	void setSpeed(int _speed);
	int getSpeed(void);
	
private:
	uint8_t speed;
	
	int    enAPin;
	int    en1Pin;
	int    en2Pin;
	int    enBPin;
	int    en3Pin;
	int    en4Pin;		
};

#endif // _L298N_H__
