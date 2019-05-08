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

SerialInput::SerialInput(uint32_t Sz)
{
    buffer = new char[Sz];
    bufferHard = nullptr;
    BufferSz = Sz;
    bi = 0;
    biH = 0;
}

SerialInput::SerialInput(uint32_t Sz, HardwareSerial* Port)
{
    bufferHard = new char[Sz];
    buffer = nullptr;
    BufferSz = Sz;
    bi = 0;
    biH = 0;
}

void SerialInput::initUSBserial(uint32_t Sz)
{
    if (buffer = nullptr)
        buffer = new char[Sz];
}

void SerialInput::initHardserial(uint32_t Sz)
{
    if (bufferHard = nullptr)
        bufferHard = new char[Sz];
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