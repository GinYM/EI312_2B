#include"Car.h"
#include<windows.h>
#include<iostream>

using namespace std;

bool Car::initialPort(char * port)//initial example initialPort("COM0")
{
	
	
	m_hCom = CreateFile(port, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (m_hCom == INVALID_HANDLE_VALUE)
	{
		return false;
	}
	else
	{
		DCB dcb;
		GetCommState(m_hCom, &dcb);

		dcb.BaudRate = 9600;//port rate
		dcb.ByteSize = 8;
		dcb.Parity = NOPARITY;
		dcb.StopBits = ONESTOPBIT;
		dcb.fOutxCtsFlow = 0;
		dcb.fRtsControl = RTS_CONTROL_ENABLE;
		SetCommState(m_hCom, &dcb);
		PurgeComm(m_hCom, PURGE_TXCLEAR);
	}
	SetupComm(m_hCom, 1024, 1024);
	SetCommMask(m_hCom, EV_RXCHAR | EV_TXEMPTY);
	return true;
}

void Car::up()
{
	DWORD length;
	buffer[0] = 'A';
	WriteFile(m_hCom, buffer, 1, &length, NULL);
}

void Car::down()
{
	DWORD length;
	buffer[0] = 'B';
	WriteFile(m_hCom, buffer, 1, &length, NULL);
}
void Car::left()
{
	DWORD length;
	buffer[0] = 'L';
	WriteFile(m_hCom, buffer, 1, &length, NULL);
}
void Car::right()
{
	DWORD length;
	buffer[0] = 'R';
	WriteFile(m_hCom, buffer, 1, &length, NULL);
}
void Car::stop()
{
	DWORD length;
	buffer[0] = 'P';
	WriteFile(m_hCom, buffer, 1, &length, NULL);
}
void Car::changeDuty(string duty)
{
	DWORD length;
	buffer[0] = 'D';
	numbuffer = new char[duty.length()];
	for (unsigned int i = 0; i<duty.length(); ++i) numbuffer[i] = duty[i];
	WriteFile(m_hCom, buffer, 1, &length, NULL);
	WriteFile(m_hCom, numbuffer, duty.length(), &length, NULL);
	WriteFile(m_hCom, buffer, 1, &length, NULL);
	delete[]numbuffer;
}
void Car::changeSpeed(string speed)
{
	DWORD length;
	buffer[0] = 'S';
	numbuffer = new char[speed.length()];
	for (unsigned int i = 0; i<speed.length(); ++i) numbuffer[i] = speed[i];
	WriteFile(m_hCom, buffer, 1, &length, NULL);
	WriteFile(m_hCom, numbuffer, speed.length(), &length, NULL);
	WriteFile(m_hCom, buffer, 1, &length, NULL);
	delete[]numbuffer;
}
void Car::changeTurn(string Turn)
{
	DWORD length;
	buffer[0] = 'C';
	numbuffer = new char[Turn.length()];
	for (unsigned int i = 0; i<Turn.length(); ++i) numbuffer[i] = Turn[i];
	WriteFile(m_hCom, buffer, 1, &length, NULL);
	WriteFile(m_hCom, numbuffer, Turn.length(), &length, NULL);
	WriteFile(m_hCom, buffer, 1, &length, NULL);
	delete[]numbuffer;
}
void Car::changeCYC(string CYC)
{
	DWORD length;
	buffer[0] = 'K';
	numbuffer = new char[CYC.length()];
	for (unsigned int i = 0; i<CYC.length(); ++i) numbuffer[i] = CYC[i];
	WriteFile(m_hCom, buffer, 1, &length, NULL);
	WriteFile(m_hCom, numbuffer, CYC.length(), &length, NULL);
	WriteFile(m_hCom, buffer, 1, &length, NULL);
	delete[]numbuffer;
}
