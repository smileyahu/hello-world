#include <avr/io.h>
#include <avr/interrupt.h>

void InitialiseGeneral();
void InitialiseTimer1();
void InitializeADC();

//declares variables for the PID regulators
struct PID FlowController;
double FlowPosition; //Current value
double FlowError; //Current value minus Set-point
double FlowOutput;

struct PID LevelController;
double LevelPosition;
double LevelError;
double LevelOutput;

double LevelSetpoint;

//Example code from https://www.embedded.com/design/prototyping-and-development/4211211/PID-without-a-PhD
typedef struct PID //Struct for storing PID values.
{
	double dState; // Last position input
	double iState; // Integrator state
	double iMax, iMin; // Maximum and minimum allowable integrator state
	double iGain, pGain, dGain;// integral gain, proportional gain, derivative gain
}PID;
double UpdatePID(PID * pid, double error, double position) //PID controller.
{
	double pTerm, dTerm, iTerm;

	// calculate the proportional term
	pTerm = pid->pGain * error;
	
	// calculate the integral state with appropriate limiting and calculates the integral term
	pid->iState += error;
	if (pid->iState > pid->iMax)pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) pid->iState = pid->iMin;
	iTerm = pid->iGain * pid->iState;
	
	// calculates the derivate term and stores the state
	dTerm = pid->dGain * (position - pid->dState);
	pid->dState = position;

	// returns result
	return pTerm + iTerm - dTerm;
}
//End of example code

int main(void)
{
	InitialiseGeneral();
	InitialiseTimer1();
	InitializeADC();

	while (1) { }
}

void InitialiseGeneral() //Stuff
{
	//Port declaration goes here
	DDRA = 0xFF;			// Configure PortA direction for Output
	PORTA = 0xFF;			// Set all LEDs initially off
	
	//temp declaration
	LevelSetpoint = 50;

	//Temp. The position values will come from analog inputs
	FlowPosition = 10;
	LevelPosition = 10;
	
	//Initialize values to flow controller
	FlowController.iGain = 10;
	FlowController.pGain = 10;
	FlowController.dGain = 10;
	FlowController.iMax = 100;
	FlowController.iMin = 0;

	//Again but for the level controller
	LevelController.iGain = 10;
	LevelController.pGain = 10;
	LevelController.dGain = 10;
	LevelController.iMax = 100;
	LevelController.iMin = 0;

	sei(); //Enable interrupt
}

void InitialiseTimer1() //Copied from TimerDemo3. Generates interrupt on a one second interval. This will be changed.
{
	TCCR1A = 0b00000000;	// Normal port operation (OC1A, OC1B, OC1C), Clear Timer on 'Compare Match' (CTC) waveform mode)
	TCCR1B = 0b00001101;	// CTC waveform mode, use prescaler 1024
	TCCR1C = 0b00000000;
	
	// For 1 MHz clock (with 1024 prescaler) to achieve a 1 second interval:
	// Need to count 1 million clock cycles (but already divided by 1024)
	// So actually need to count to (1000000 / 1024 =) 976 decimal, = 3D0 Hex
	OCR1AH = 0x03; // Output Compare Registers (16 bit) OCR1BH and OCR1BL
	OCR1AL = 0xD0;

	TCNT1H = 0b00000000;	// Timer/Counter count/value registers (16 bit) TCNT1H and TCNT1L
	TCNT1L = 0b00000000;
	TIMSK1 = 0b00000010;	// bit 1 OCIE1A		Use 'Output Compare A Match' Interrupt, i.e. generate an interrupt
	// when the timer reaches the set value (in the OCR1A register)
}

void InitializeADC() //Copied from TwoPotentiometers. Serves as input for all analog inputs
{
	ADMUX = 0b01100010;	// AVCC REF, Left-adjust output (Read most-significant 8 bits via ADCH), Convert channel 2
	ADCSRA = 0b10101101;	// ADC enabled, Auto trigger, Interrupt enabled, Prescaler = 32
	ADCSRB &= 0b11110000;	// clear bits 3,2,1,0 (Free running mode)
	DIDR0 = 0b00001100;	// Disable digital input on bits 2 and 3
	DIDR2 = 0b11111111;	// Disable digital input on all bits (64-pin version of ATmega1281 does not even have these inputs)
	ADCSRA |= 0b01000000;	// start ADC conversion
}

ISR(ADC_vect) // ADC Interrupt Handler. This interrupt handler is common for all ADC channels
{
	// Need to alternate which channel is converted
	unsigned char ADMUX_temp = ADMUX;
	unsigned char ADCH_temp = ADCH;
	
	ADMUX_temp &= 0b00011111;	// Mask off non-multiplexer bits
	if(0b00000010 == ADMUX_temp)
	{
		LevelPosition = ADCH_temp;
		ADMUX = 0b01100011;		// Set ADMUX ADC register - next conversion is for ADC3
	}
	else
	{
		FlowPosition = ADCH_temp;
		ADMUX = 0b01100010;		// Set ADMUX ADC register - next conversion is for ADC2
	}
	
	//Debug code
	//PORTA = FlowPosition;
	//PORTA = LevelPosition;
}

ISR(TIMER1_COMPA_vect)
{
	LevelError = LevelPosition - LevelSetpoint;
	LevelOutput = UpdatePID(&LevelController, LevelError, LevelPosition);

	FlowError = FlowPosition - LevelOutput;
	FlowOutput = UpdatePID(&FlowController, FlowError, FlowPosition);

	//Debug code
	PORTA = FlowOutput;
	//PORTA = LevelOutput;
}


