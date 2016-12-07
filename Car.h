#ifndef CAR

#define CAR

#include <windows.h>
#include <iostream>
#include <string>

using namespace std;
class Car
{
public:
	//don't change this function!or there may be some mistakes
	bool initialPort(char * port);//initial example initialPort("COM0")
	void up();
	void down();
	void left();
	void right();
	void stop();
	void changeDuty(string duty);
	void changeSpeed(string speed);
	void changeTurn(string Turn);
	void changeCYC(string CYC);
private:
	HANDLE m_hCom;
	char buffer[1];//input signal
	char* numbuffer;//use to input number
};


#endif