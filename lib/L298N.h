/**************************************************
 * RNT-NTRD  SHK
 * 2015. 3. 05 Ver.01 
 **************************************************/

#ifndef _L298N_H__
#define _L298N_H__

#include <inttypes.h>

#define DEF_SPEED	    50
#define MAX_SPEED       30
#define NOR_SPEED       70
#define MIN_SPEED       100

#define PWM_PERIOD      100
#define PWM_STOP        100
#define PWM_MAX         0

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