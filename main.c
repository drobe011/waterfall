/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#include "chip.h"

//#define WWDT

#define ON_TIME_AFTER_MVMT_MIN (15*60*1000)
#define OFF_TO_ON_HYSTERISIS_MIN (1*60*1000)
#define ON_TIME_TOPLED_SEC (5*1000)
#define ON_TIME_BOTTOMLED_SEC (5*1000)
#define ON_TIME_BOTHLED_SEC (5*1000)
#define ON_TIME_INIT_SEC (5*1000)
#define ON_TIME_ERROR_BLINK 10
#define ON_TIME_ERROR_BLINK_FX_MS (500*1000)

#define INPUT_PIR 0
#define INPUT_FLOAT 6
#define OUTPUT_LED_TOP 10
#define OUTPUT_LED_BOTTOM 11
#define OUTPUT_PUMP_CTRL 13

#define STATE_CHECK_MS 10

static volatile uint32_t sysTick;

void SysTick_Handler(void)
{
	sysTick++;
}

void init();
void checkMovement();
void run();
void stop();
void error();

const uint32_t OscRateIn = 0;
const uint32_t ExtRateIn = 0;

uint8_t checkFloat();
uint8_t checkPIR();
void setupPort();

void (*p_StateFunction)() = NULL;

int main(void) {
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000);

#ifdef WWDT
	uint32_t wdtFreq;	/* Freq = 0.6Mhz, divided by 64. WDT_OSC should be 9.375khz */
	Chip_Clock_SetWDTOSC(WDTLFO_OSC_0_60, 64);	/* Enable the power to the WDT */
	Chip_SYSCTL_PowerUp(SYSCTL_SLPWAKE_WDTOSC_PD);	/* The WDT divides the input frequency into it by 4 */
	wdtFreq = Chip_Clock_GetWDTOSCRate() / 4;	/* Initialize WWDT (also enables WWDT clock) */
	Chip_WWDT_Init(LPC_WWDT);	/* Set watchdog feed time constant to approximately 1s */
	Chip_WWDT_SetTimeOut(LPC_WWDT, wdtFreq/10);	 /* (100ms) */
	Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET); /* Configure WWDT to reset on timeout */
	Chip_WWDT_Start(LPC_WWDT); /* Start watchdog */
#endif

	setupPort();

	uint32_t stateTick = sysTick;
	p_StateFunction = &init;

	while(1) {
		if (sysTick - stateTick <= STATE_CHECK_MS)
		{
#ifdef WWDT
			Chip_WWDT_Feed(LPC_WWDT);
#endif
			p_StateFunction();
			stateTick = sysTick;
			Chip_GPIO_SetPinToggle(LPC_GPIO_PORT, 0, 7);
		}
    }
    return 0;
}

void setupPort()
{
	/* Initialize GPIO */
	Chip_GPIO_Init(LPC_GPIO_PORT);

	//PIO0_0 - INPUT, PD_ON :: PIR INPUT
	Chip_IOCON_PinSetMode(LPC_IOCON, IOCON_PIO0, PIN_MODE_PULLDN);
	//PIO0_6 - INPUT, PU_ON :: FLOAT INPUT
	//DEFAULTS OK
	//PIO0_7 - OUTPUT :: SYSTICK TOGGLE FOR DEBUG
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 7);
	//PIO0_10 - OUTPUT (o/c) :: LED_TOP
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, OUTPUT_LED_TOP);
	//PIO0_11 - OUTPUT (o/c) :: LEDBOTTOM
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM);
	//PIO0_13 - OUTPUT :: PUMP CONTROL
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, OUTPUT_PUMP_CTRL);
	//PIO0_1 - DEFAULT :: UNUSED
	//PIO0_2 - DEFAULT :: SWDIO
	//PIO0_3 - DEFAULT :: SWCLK
	//PIO0_4 - DEFAULT :: UNUSED
	//PIO0_5 - DEFAULT :: RESET
	//PIO0_8 - DEFAULT :: UNUSED
	//PIO0_9 - DEFAULT :: UNUSED
	//PIO0_12 - DEFAULT :: UNUSED (ISP p/u)

}

//system test
void init()
{
	uint32_t nowTime = sysTick;
	if (checkFloat()) //level good
	{
		Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_PUMP_CTRL);
		Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_LED_TOP);
		Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM);
		while (sysTick - nowTime <= ON_TIME_INIT_SEC)
		{
			if (!checkFloat()) break;
#ifdef WWDT
			Chip_WWDT_Feed(LPC_WWDT);
#endif
		}
		Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_PUMP_CTRL);
		Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_LED_TOP);
		Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM);
	}
	if (checkFloat()) p_StateFunction = &checkMovement;
	else p_StateFunction = &error;
}

void checkMovement()
{
	if (checkPIR() && checkFloat()) p_StateFunction = &run;
	if (!checkFloat()) p_StateFunction = &error;
}

void run()
{
	//runTime tracks overall run time
	//subTime tracks individual state times (top, bottom, both LEDs)
	static uint32_t runTime = 0, subTime = 0;
	typedef enum runState
	{
		START = 0,
		TOPLED = 1,
		CKTOPLED = 11,
		BOTTOMLED = 2,
		CKBOTTOMLED = 22,
		BOTHLED = 3,
		CKBOTHLED = 33
	} runState_T;

	static runState_T runState = START;

	if (!checkFloat())
	{
		runState = 0;
		p_StateFunction = &error;
	}
	else //do the run sequence
	{
		switch (runState)
		{
		case START: //INITIALIZE RUN SEQUENCE
			Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_PUMP_CTRL);
			runTime = sysTick;
			runState = 1;
			break;
		case TOPLED: //INITIALIZE TOP LED
			Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_LED_TOP);
			subTime = sysTick;
			runState = 11;
			break;
		case CKTOPLED: //CHECK ON TOP LED
			if (sysTick - subTime >= ON_TIME_TOPLED_SEC)
			{
				Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_LED_TOP);
				runState = 2;
			}
			break;
		case BOTTOMLED: //INITIALIZE BOTTOM LED
			Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM);
			subTime = sysTick;
			runState = 22;
			break;
		case CKBOTTOMLED: //CHECK ON BOTTOM LED
			if (sysTick - subTime >= ON_TIME_BOTTOMLED_SEC)
			{
				Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM);
				runState = 3;
			}
			break;
		case BOTHLED: //INITIALIZE BOTH LEDs
			Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_LED_TOP);
			Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM);
			subTime = sysTick;
			runState = 33;
			break;
		case CKBOTHLED: //CHECK ON BOTH LEDs
			if (sysTick - subTime >= ON_TIME_BOTTOMLED_SEC)
			{
				Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_LED_TOP);
				Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM);
				runState = 1;
			}
			break;
		}
		if (sysTick - runTime >= ON_TIME_AFTER_MVMT_MIN)
		{
			runState = 0;
			p_StateFunction = &stop;
		}
		else if (checkPIR()) runState = sysTick;
	}
}

void stop()
{
	uint32_t nowTime = sysTick;
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_PUMP_CTRL);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_LED_TOP);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM);
	while (sysTick - nowTime <= OFF_TO_ON_HYSTERISIS_MIN)
	{
		if (!checkFloat()) break;
#ifdef WWDT
			Chip_WWDT_Feed(LPC_WWDT);
#endif
	}
	p_StateFunction = &checkMovement;
}

void error()
{
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_PUMP_CTRL);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_LED_TOP);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM);

	uint8_t blinkLoop = 0;
	uint32_t storeTime = 0;

	for (blinkLoop = 0; blinkLoop < ON_TIME_ERROR_BLINK; blinkLoop++)
	{
		Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM);
		storeTime = sysTick;
		while (sysTick - storeTime <= (ON_TIME_ERROR_BLINK_FX_MS/2))
		{
			//__NOP;
#ifdef WWDT
			Chip_WWDT_Feed(LPC_WWDT);
#endif
		}
		Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM);
		storeTime = sysTick;
		while (sysTick - storeTime <= (ON_TIME_ERROR_BLINK_FX_MS/2))
		{
			//__NOP;
#ifdef WWDT
			Chip_WWDT_Feed(LPC_WWDT);
#endif
		}
	}
	p_StateFunction = &checkMovement;
}

uint8_t checkFloat()
{
	return !Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0, INPUT_FLOAT);
}

uint8_t checkPIR()
{
	return Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0, INPUT_PIR);
}
