#ifndef _SERIALINPUT_H_
#define _SERIALINPUT_H_

#include <Arduino.h>
class SerialInput
{
    private:
        uint32_t BufferSz;
        char* buffer; // "string" buffer
        char* bufferHard; // "string" buffer for hardware serial
        int bi; // buffer index
        int biH;

        HardwareSerial* SerialHard = NULL;

    public:
        SerialInput(void);
        SerialInput(uint32_t Sz);
        SerialInput(uint32_t Sz, HardwareSerial* port);

        void initUSBserial(uint32_t Sz);
        void initHardserial(uint32_t Sz);
        void setHardSerial(HardwareSerial* port);
        char* readUSBserial(void);
        char* readHardserial(void);
};

#endif