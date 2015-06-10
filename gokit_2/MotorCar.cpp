#include <Arduino.h>
#include "MotorCar.h"

/***********
类构造函数
**********/
MotorCar::MotorCar(int _slpin,int _dlpin,int _srpin,int _drpin)
{
	_speedLeftPin = _slpin;
	_speedRightPin = _srpin;
	_dirLeftPin = _dlpin;
	_dirRightPin = _drpin;
	
	pinMode(_speedLeftPin,OUTPUT);
	pinMode(_speedRightPin,OUTPUT);
	pinMode(_dirLeftPin,OUTPUT);
	pinMode(_dirRightPin,OUTPUT);
}

/***********
前进函数
**********/
void MotorCar::forward (int _speed)
{
	digitalWrite(_dirRightPin,HIGH);
	
	digitalWrite(_dirLeftPin,HIGH);
	
	analogWrite(_speedRightPin,_speed);
	
	analogWrite(_speedLeftPin,_speed);
}



/***********
后退函数
**********/
void MotorCar::back (int _speed)
{
	digitalWrite(_dirRightPin,LOW);

	digitalWrite(_dirLeftPin,LOW);

	analogWrite(_speedRightPin,_speed);

	analogWrite(_speedLeftPin,_speed);
}

/***********
左转函数
**********/
void MotorCar::turnLeft (int _speed)
{
	digitalWrite(_dirRightPin,HIGH);

	analogWrite(_speedRightPin,_speed);

	analogWrite(_speedLeftPin,0);
}


/***********
右转函数
**********/
void MotorCar::turnRight (int _speed)
{
	digitalWrite(_dirLeftPin,HIGH);

	analogWrite(_speedRightPin,0);

	analogWrite(_speedLeftPin,_speed);
}



/***********
原地左转函数
**********/
void MotorCar::turnLeftOrigin (int _speed)
{
	digitalWrite(_dirRightPin,HIGH);

	digitalWrite(_dirLeftPin,LOW);

	analogWrite(_speedRightPin,_speed);

	analogWrite(_speedLeftPin,_speed);
}



/***********
原地右转函数
**********/
void MotorCar::turnRightOrigin (int _speed)
{
	digitalWrite(_dirLeftPin,HIGH);

	digitalWrite(_dirRightPin,LOW);

	analogWrite(_speedRightPin,_speed);

	analogWrite(_speedLeftPin,_speed);
}

/***********
停止函数
**********/
void MotorCar::stop()
{
	analogWrite(_speedRightPin,0);

	analogWrite(_speedLeftPin,0);
}

