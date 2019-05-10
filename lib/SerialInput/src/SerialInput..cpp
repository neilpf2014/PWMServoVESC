#include "SerialInput.h"
#include <HardwareSerial.h>

SerialInput::SerialInput(void)
{
    buffer = nullptr;
    bufferHard = nullptr;
    BufferSz = 0;
    bi = 0;
    biH = 0;
}

SerialInput::SerialInput(int Sz)
{
    buffer = new char[Sz];
    bufferHard = nullptr;
    BufferSz = Sz;
	buffer[0] = '\0';
    bi = 0;
    biH = 0;
}

SerialInput::SerialInput(int Sz, HardwareSerial* Port)
{
    bufferHard = new char[Sz];
	bufferHard[0] = '\0';
    buffer = nullptr;
    BufferSz = Sz;
    bi = 0;
    biH = 0;
}

void SerialInput::initUSBserial(int Sz)
{
    if (buffer == nullptr){
		 buffer = new char[Sz];
	}
	buffer[0] = '\0';
	bi = 0;
}

void SerialInput::initHardserial(int Sz)
{
    if (bufferHard == nullptr){
        bufferHard = new char[Sz];
	}
	bufferHard[0] = '\0';
	biH = 0;
}


void SerialInput::setHardSerial(HardwareSerial* Port)
{
    SerialHard = Port;
}
char* SerialInput::readUSBserial(void)
{
    char Ch = '\0';
	char* retVal;
	retVal = nullptr;

	// called each time the function is called to collect stuff from serial buffer
	while ((Serial.available() > 0))
	{
		Ch = Serial.read();
		if ((Ch != '\n') && (Ch != '\r')) //need to trap both <CR> & <LF> !
		{
			buffer[bi] = Ch;
			bi++;
		}
		Serial.print(Ch);
	}
	if (((Ch == '\n')||(Ch == '\r')) || (bi > (BufferSz-1))) {
		char* somearray = new char[bi + 1];
		for (int j = 0; j < bi; j++)
			somearray[j] = buffer[j];
		somearray[bi] = '\0'; // null termination
		retVal = somearray;
		// reset buffer
		buffer[0] = '\0';
		bi = 0;
	}
	if (retVal != nullptr)
		return retVal;
	else
		return (char *)'\0';
}

char* SerialInput::readHardserial(void)
{
    char Ch = '\0';
	char* retVal;
	retVal = nullptr;

	// called each time the function is called to collect stuff from serial buffer
	while ((SerialHard->available() > 0))
	{
		Ch = SerialHard->read();
		if ((Ch != '\n') && (Ch != '\r')) //need to trap both <CR> & <LF> !
		{
			bufferHard[biH] = Ch;
			biH++;
		}
		SerialHard->print(Ch);
	}
	if (((Ch == '\n')||(Ch == '\r')) || (bi > (BufferSz-1))) {
		char* somearray = new char[bi + 1];
		for (int j = 0; j < biH; j++)
			somearray[j] = bufferHard[j];
		somearray[biH] = '\0'; // null termination
		retVal = somearray;
		// reset buffer
		bufferHard[0] = '\0';
		bi = 0;
	}
	if (retVal != nullptr)
		return retVal;
	else
		return (char *)'\0';
}