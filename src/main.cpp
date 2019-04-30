/*  Read and generate PWM using STM32 timer registers
    Adaped from demo code for genration of code for VESC
    on Fubar self driving project

    This is a mash up to 2 projects so needs some work
    Right now has code for both wii numchuck and PWM to VESC
    4/30/2019
*/
#include <Arduino.h>
#include <VescUart.h>

/** Initiate VescUart class */
VescUart UART;


int32_t channel_1_start, channel_1_stop, channel_1;

#define LED PB12
unsigned long pMills = 0;
unsigned long p2Mills = 0;
unsigned long cMills = 0;
//unsigned long cTime = 500;
unsigned long rTime = 20;
uint32_t PotValue;
uint32_t readPotValue;
uint32_t PotNormValue;
uint32_t TccrVal = 1000;
uint8_t LEDstate = 1;

unsigned long cTime = 100; // polling / send command time in ms

//char Tchar;
char CmdChr;
String testStr;
String command;
String Sspeed;
bool upperBtn;
bool lowerBtn;
int Yvalue;
String lastCmd;
int throttle = 127;

char buffer[50]; // "string"
int bi; // buffer index

// Read String input handling: thanks Rick
// "String" is now returned as a ptr to char assay
char* readCharAInput(void)
{
	char Ch = '\0';
	char* retVal;
	retVal = nullptr;
// compiler was complaining under new version of STM32 librarys so the 2 below
  char* noChar = new char[1];
  noChar[0] = '\0';

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
	if (((Ch == '\n')||(Ch == '\r')) || (bi > 49)) {
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
		return noChar;
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

	/** Setup Serial port to enter commands */
	Serial.begin(115200);
	/** Setup UART port (Serial1 on bluepill) */
	Serial1.begin(115200);

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
			//UART.nunchuck.lowerButton = true;
			lowerBtn = true;
			lastCmd = command;
			break;
		}
		/** Set cruise control off*/
		case 'o':
		{
			//UART.nunchuck.lowerButton = false;
			lowerBtn = false;
			lastCmd = command;
			break;
		}
		/** The upperButton is reverse (we think ??)*/
		case 'r':
		{
			//UART.nunchuck.upperButton = true;
			upperBtn = true;
			lastCmd = command;
		}
		case 'f':
		{
			//UART.nunchuck.upperButton = false;
			upperBtn = false;
			lastCmd = command;
		}
		// kill motor
		case 'k':
		{
			// UART.nunchuck.valueY = 127;
			Yvalue = 127;
			lastCmd = command;
		}
		// set throttle / brake value
		case 't':
		{
			Sspeed = command.substring(1);
			Serial.println(Sspeed);
			throttle = Sspeed.toInt();
			if (throttle >= 0 && throttle <= 255) {

				/** The valueY is used to control the speed, where 127 is the middle = no current */
				Yvalue = throttle;
				lastCmd = "T value is " + String(throttle);
			}
			else {
				Yvalue = 127;
				lastCmd = "invalid throttle value";
			}
			break;
		}
			default:
				break;
		}
	}

	cMills = millis();
	if (cMills - pMills > cTime)
	{
		pMills = cMills;
		if (lastCmd != "") {
			Serial.println(lastCmd);
			lastCmd = "";
		}

		
		if (UART.getVescValues()) {
			Serial.print("RPM ");
			Serial.println(UART.data.rpm);
			Serial.print("Batt volts ");
			Serial.println(UART.data.inpVoltage);
			Serial.print("Amps ");
			Serial.println(UART.data.avgMotorCurrent);
			Serial.print("duty Cycle ");
			Serial.println(UART.data.dutyCycleNow);
		}
		else
		{
			Serial.println("Failed to get data!!!");
		}
		UART.nunchuck.lowerButton = lowerBtn;
		UART.nunchuck.upperButton = upperBtn;
		UART.nunchuck.valueY = Yvalue;
		UART.setNunchuckValues();

    Serial.print("pot Value is: " + String(channel_1) + " ");
    Serial.println(PotNormValue);
		LEDstate = TogLED(LEDstate);
	}
  if(cMills - p2Mills > rTime)
	{
		p2Mills = cMills;
		PotValue = analogRead(PB0);
    PotNormValue = ((PotValue / 40)*10) + 1000;
    
    TIMER4_BASE->CCR1 = PotNormValue;
	}
}
