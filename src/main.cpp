/*  Read and generate PWM using STM32 timer registers
    Adaped from demo code for genration of code for VESC
    on Fubar self driving project

    This is a mash up to 2 projects so needs some work
    Right now has code for both wii numchuck and PWM to VESC
    5/5/2019

		Pins PA9(Tx1) PA10(Rx1) serial from VESC A9 gr, A10 orange
		Pins A2(TX2), A3(Rx2) serial out to other MC
		Pin PA0 is PWM in from RC
		Pin PB6 is PWM out to VESC 
		Pin PB0 is Analog read from throttle
		Pins PB11, PB12 have 10k pull ups on breakout board

*/
#include <Arduino.h>
#include <VescUart.h>

/** Initiate VescUart class */
VescUart UART;


int32_t channel_1_start, channel_1_stop, channel_1;

// #define LED PB12 // if black pill
#define DEBUG 1
#define STR_BUFFER_SZ 50

unsigned long pMills = 0;
unsigned long p2Mills = 0;
unsigned long cMills = 0;
unsigned long cTime = 200; // polling / send command time in ms
unsigned long rTime = 100;  // polling / for PWM
uint32_t TccrVal = 1000;

// for Analog throttle input from pot or hall sensor
uint32_t PotValue;
uint32_t oldPotNValue;
uint32_t PotNValue;
uint32_t PWMremValue;
uint32_t AnScaleOffset = 1500; // PWM offset from base 1000 full 1500 half
// 40 is full range 80 half PWM range
uint32_t AnScaleFactor = 80; // number to div 1024 by (* 10)

// Led hearbeat
uint8_t LEDstate = 1;

//char Tchar;
char CmdChr;
String VESCtlmtyHR;  // human readable
String VESCtlmty; // for fubarino 
String command;
String Sspeed;
String PWMinput;
bool upperBtn;
bool lowerBtn;
int Yvalue;
String lastCmd;
int throttle = 127;

char buffer[STR_BUFFER_SZ]; // "string" buffer
int bi; // buffer index

// Read String input handling: thanks Rick
// "String" is now returned as a ptr to char assay
char* readCharAInput(void)
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
	if (((Ch == '\n')||(Ch == '\r')) || (bi > (STR_BUFFER_SZ-1))) {
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

// Toggle state of onboard led
// for testing
uint8_t TogLED(uint8_t state)
{
	uint8_t rState;
	if (state > 0)
	{
		digitalWrite(LED_BUILTIN, HIGH);
		rState = 0;
	}
	else
	{
		digitalWrite(LED_BUILTIN, LOW);// Turn the LED on
		rState = 1;
	}
	return rState;
}



// timer interupt handler for channel 1 pulse detection
void handler_channel_1(void) {
  if (0b1 & GPIOA_BASE->IDR) {
    channel_1_start = TIMER2_BASE->CCR1;
    TIMER2_BASE->CCER |= TIMER_CCER_CC1P;
  }
  else {
    channel_1 = TIMER2_BASE->CCR1 - channel_1_start;
    if (channel_1 < 0)channel_1 += 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P;
  }
}

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
	/** Setup Serial port to enter commands */
	Serial.begin(115200);
	/** Setup UART port (Serial1 on bluepill) */
	Serial1.begin(115200);
	/** Setup port for upstream MC (Serial2 on bluepill) */
	Serial2.begin(115200);

	while (!Serial1) { ; }

	/** Define which ports to use as UART */
	UART.setSerialPort(&Serial1);
	// init nunchuck buttons
	upperBtn = false;
	lowerBtn = false;
	Yvalue = 127;
	//init buffer as empty "string"
	bi = 0;
	buffer[0] = '\0';

// Example pulse generation code
  pinMode(PB6, PWM);
  pinMode(PB0,INPUT_ANALOG);
  
  TIMER4_BASE->CR1 = TIMER_CR1_CEN|TIMER_CR1_ARPE;
  TIMER4_BASE->CR2 = 0;
  TIMER4_BASE->SMCR = 0;
  TIMER4_BASE->DIER = 0;
  TIMER4_BASE->EGR = 0;
  TIMER4_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE;
  TIMER4_BASE->CCMR2 = 0;
  TIMER4_BASE->CCER = TIMER_CCER_CC1E;
  TIMER4_BASE->PSC = 71;
  TIMER4_BASE->ARR = 4000;
  TIMER4_BASE->DCR = 0;
  TIMER4_BASE->CCR1 = TccrVal;

   // Pulse length counting code init

  pinMode(PA0, INPUT);
  delay(250);

  Timer2.attachCompare1Interrupt(handler_channel_1);
  TIMER2_BASE->CR1 = TIMER_CR1_CEN;
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR = 0;
  TIMER2_BASE->DIER = TIMER_DIER_CC1IE;
  TIMER2_BASE->EGR = 0;
  TIMER2_BASE->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1;
  TIMER2_BASE->CCMR2 = 0;
  TIMER2_BASE->CCER = TIMER_CCER_CC1E;
  TIMER2_BASE->PSC = 71;
  TIMER2_BASE->ARR = 0xFFFF;
  TIMER2_BASE->DCR = 0;
}


void loop() {
	command = readCharAInput();
	if (command != "")
	{
		CmdChr = command.charAt(0);
		switch (CmdChr)
		{
			/** The lowerButton is used to set cruise control */
		case 'c':
		{
			lowerBtn = true;
			lastCmd = command;
			break;
		}
		/** Set cruise control off*/
		case 'o':
		{
			lowerBtn = false;
			lastCmd = command;
			break;
		}
		// The upperButton is reverse (we think ??)
		case 'r':
		{
			upperBtn = true;
			lastCmd = command;
			break;
		}
		//forward
		case 'f':
		{
			upperBtn = false;
			lastCmd = command;
			break;
		}
		// kill motor (don't use)
		case 'k':
		{
			// UART.nunchuck.valueY = 127;
			Yvalue = 127;
			lastCmd = command;
			break;
		}
		// show telemetry
		case 't':
		{
			Serial.println(VESCtlmtyHR);
			Serial.println(PWMinput + String(PotNValue));
			break;
		}
		// now getting PWM, removed nunchuck code
		
			default:
				break;
		}
	}

	cMills = millis();
	if (cMills - pMills > cTime)
	{
		pMills = cMills;
		if (DEBUG){
			if (lastCmd != "") {
				Serial.println(lastCmd);
				lastCmd = "";
			}
		}
		if (UART.getVescValues()) {
			if (DEBUG){
				VESCtlmtyHR = "Throttle PWM " + String(TIMER14_BASE->CCR1) + '\n';
				VESCtlmtyHR = VESCtlmtyHR + "RPM " + UART.data.rpm + '\n';
				VESCtlmtyHR = VESCtlmty + "Batt volts " + String(UART.data.inpVoltage) + '\n';
				VESCtlmtyHR = VESCtlmty + "duty Cycle " + String(UART.data.dutyCycleNow);
				VESCtlmtyHR = VESCtlmty + "Batt volts " + String(UART.data.inpVoltage) + '\n';
			}
			VESCtlmty = String(TIMER14_BASE->CCR1) + "," + String(UART.data.rpm) + ',' + String(UART.data.avgMotorCurrent);
			VESCtlmty = VESCtlmty + "," +String(UART.data.dutyCycleNow) + "," + String(UART.data.inpVoltage) + '\n';
		}
		else
		{
			if (DEBUG)
				VESCtlmtyHR = "Throttle PWM " + String(TIMER14_BASE->CCR1) + '\n';
			VESCtlmty = String(TIMER14_BASE->CCR1);
		}
		UART.nunchuck.lowerButton = lowerBtn;
		UART.nunchuck.upperButton = upperBtn;
		//UART.nunchuck.valueY = Yvalue;
		UART.setNunchuckValues();
		Serial2.println(VESCtlmty); // to other MC
		if (DEBUG){
    		PWMinput = "PWM Value is: " + String(channel_1) + " ";
    		Serial.println(PWMinput + String(PotNValue));
		}
		LEDstate = TogLED(LEDstate);
	}
  if(cMills - p2Mills > rTime)
	{
		PWMremValue = channel_1;
		p2Mills = cMills;
		PotValue = analogRead(PB0);
		// this rownds to the 10's and scales the pot input
    	PotNValue = ((PotValue / AnScaleFactor)*10) + AnScaleOffset;
	// Manual override
		if (PotNValue != oldPotNValue)
		{
			TIMER4_BASE->CCR1 = PotNValue;
			oldPotNValue = PotNValue;
		}
		else
		//pass incoming PWM to outgoing PWM
			TIMER4_BASE->CCR1 = channel_1;
	}
}
