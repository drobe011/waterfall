/*
===============================================================================
 Name        : main.c
 Author      : David Robertson <>
 Version     :
 Copyright   :
 Description : mini waterfall controller (LPC811)
===============================================================================
 */

#include "chip.h"

const uint32_t ExtRateIn = 0;
const uint32_t OscRateIn = 0;

/* SYSTEM SEQUENCE
 * POWER ON -> INIT STATE
 * 		WAIT FOR PIR TO STABALIZE
 * 		CHECK IF IN TEST MODE
 * 		CHECK WATER LEVEL (ENTER ERROR IF LOW)
 * 		TEST PUMP AND LIGHTS (1 BLINK = NORMAL, 2 = TEST)
 * INIT -> CHECK FOR MOVEMENT STATE
 * 		CHECK WATER LEVEL (ENTER ERROR IF LOW)
 * 		CHECK FOR MOVEMENT
 * CHECK FOR MOVEMENT -> RUN
 * 		CHECK WATER LEVEL (ENTER ERROR IF LOW)
 * 		TURN PUMP ON
 * 		START LED SEQUENCE SUB STATE
 * 			CHECK WATER LEVEL (ENTER ERROR IF LOW)
 * 			CHECK FOR MOVEMENT (RESET RUN TIMER)
 * 			CHECK RUN TIMER (SHORT RUN TIME IF TEST)
 * RUN -> STOP
 * 		ENSURE EVERYTHING SHUTOFF
 * 		DELAY HYSTERESIS (SHORT DELAY IF TEST)
 * STOP -> CHECK FOR MOVEMENT
 * * -> ERROR
 * 		BLINK LEDs
 * ERROR -> CHECK FOR MOVEMENT
 *
 */

#define WWDT

//OPERATIONAL TIMES TICKED BY 1ms SYSTEM TICK
#define PIR_STABILIZE_SEC (10*1000)
#define ON_TIME_AFTER_MVMT_MIN (15*60*1000)
#define ON_TEST_TIME_AFTER_MVMT_SEC (15*1000)
#define OFF_TO_ON_HYSTERISIS_MIN (1*60*1000)
#define OFF_TO_ON_TEST_HYSTERISIS_SEC (3*1000)
#define ON_TIME_TOPLED_SEC (5*1000)
#define ON_TIME_BOTTOMLED_SEC (5*1000)
#define ON_TIME_BOTHLED_SEC (5*1000)
#define ON_TIME_INIT_SEC (5*1000)
#define ON_TIME_ERROR_BLINK 10
#define ON_TIME_ERROR_BLINK_FX_MS (500*1000)

//GP MACROS
#define TIME_TO_CHECK_STATE (sysTick - stateTick >= STATE_CHECK_MS)
#define TIME_TO_TURN_OFF (sysTick - waterfallRunning[STARTWFALL].savedTime >= waterfallRunning[STARTWFALL].timeOn)

//PROGRAM FLOW TIMING
#define STATE_CHECK_MS 10

//PIN MAPPING (NO FUNCTIONS DEFINED IN SWM)
#define INPUT_PIR 0
#define INPUT_TESTPIN 4
#define INPUT_FLOAT 6
#define OUTPUT_DEBUG_SYSTICK 7
#define OUTPUT_LED_TOP 10
#define OUTPUT_LED_BOTTOM 11
#define OUTPUT_PUMP_CTRL 13

//GLOBALS
static volatile uint32_t sysTick;
uint32_t RUNTIME, HYSTERISIS_TIME; //HOLDS TIMES BASED ON TEST MODE
uint8_t IS_TEST; //REDUCES TIME FOR T/S IF INPUT_TESTPIN IS LOW ON POWERUP/RESET
uint8_t waterfallIsRunning;

struct runIt_S
{
	uint8_t amIrunning;
	uint32_t savedTime;
	uint32_t timeOn;
	void (*p_RunStateFunction)(struct runIt_S *);
};

//STATE FUNCTION PROTOTYPES
void init();
void checkMovement();// __attribute__((optimize(1)));
void runWaterfall(struct runIt_S *);
void topLED_run(struct runIt_S *);
void bottomLED_run(struct runIt_S *);
void bothLED_run(struct runIt_S *);
void stop();
void error();

void (*p_StateFunction)() = NULL;

//WORKER FUNCTION PROTOTYPES
uint8_t checkFloat();
uint8_t checkPIR();
void setupPort();

int main(void) {
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000);

	setupPort();

	typedef enum {STARTWFALL=0, TOPLED, BOTTOMLED, BOTHLED, OFF} runState_T;
	runState_T runState = STARTWFALL;

	struct runIt_S waterfallRunning[4] = {{0, 0, RUNTIME, &runWaterfall}, {0, 0, ON_TIME_TOPLED_SEC, &topLED_run}, {0, 0, ON_TIME_BOTTOMLED_SEC, &bottomLED_run}, {0, 0, ON_TIME_BOTHLED_SEC, &bothLED_run}};

	uint32_t stateTick = sysTick;
	p_StateFunction = &init;

#ifdef WWDT
	uint32_t wdtFreq;
	Chip_Clock_SetWDTOSC(WDTLFO_OSC_0_60, 64);
	Chip_SYSCTL_PowerUp(SYSCTL_SLPWAKE_WDTOSC_PD);
	wdtFreq = Chip_Clock_GetWDTOSCRate() / 4;
	Chip_WWDT_Init(LPC_WWDT);
	Chip_WWDT_SetTimeOut(LPC_WWDT, wdtFreq/10);	 /* (100ms) */
	Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);
	Chip_WWDT_Start(LPC_WWDT);
#endif

	while(1) {
		if (TIME_TO_CHECK_STATE)
		{
#ifdef WWDT
			Chip_WWDT_Feed(LPC_WWDT);
#endif
			if (!checkFloat())
			{
				p_StateFunction = &error;
				waterfallRunning[runState].amIrunning = 0;
				waterfallIsRunning = 0;
			}
			else if (waterfallIsRunning)
			{
				if (waterfallRunning[runState].amIrunning == 2)
				{
					waterfallRunning[runState].amIrunning = 0;
					runState = runState < OFF ? runState + 1 : TOPLED;
				}
				if (checkPIR()) waterfallRunning[STARTWFALL].timeOn = sysTick;
				if (TIME_TO_TURN_OFF)
				{
					runState = STARTWFALL;
					waterfallIsRunning = 0;
					p_StateFunction = &stop;
				}
				else waterfallRunning[runState].p_RunStateFunction(&waterfallRunning[runState]);
			}
			else p_StateFunction();
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
	//PIO0_4 - INPUT, PU_ON :: TEST MODE INPUT
	//PIO0_6 - INPUT, PU_ON :: FLOAT INPUT
	//PIO0_7 - OUTPUT :: SYSTICK TOGGLE FOR DEBUG
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, OUTPUT_DEBUG_SYSTICK);
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

void init()
{
	uint32_t nowTime = sysTick;
	uint8_t run2xIfTestMode = 0;

	Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM);
	while (sysTick - nowTime <= PIR_STABILIZE_SEC)
	{
#ifdef WWDT
		Chip_WWDT_Feed(LPC_WWDT);
#endif
	}
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM);

	IS_TEST = !Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0, INPUT_TESTPIN) ? 1 : 0;

	if (checkFloat()) //level good
	{
		Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_PUMP_CTRL);
		for (run2xIfTestMode = 0; run2xIfTestMode < IS_TEST+1; run2xIfTestMode++)
		{
			nowTime = sysTick;
			Chip_GPIO_SetPortOutHigh(LPC_GPIO_PORT, 0, (1 << OUTPUT_LED_TOP) | (1 << OUTPUT_LED_BOTTOM));
			while (sysTick - nowTime <= ON_TIME_INIT_SEC)
			{
				if (!checkFloat()) break;
#ifdef WWDT
				Chip_WWDT_Feed(LPC_WWDT);
#endif
			}
			Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, 0, (1 << OUTPUT_LED_TOP) | (1 << OUTPUT_LED_BOTTOM));
		}
		Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_PUMP_CTRL);
		p_StateFunction = &checkMovement;
	}
	else p_StateFunction = &error;

	RUNTIME = IS_TEST ? ON_TEST_TIME_AFTER_MVMT_SEC : ON_TIME_AFTER_MVMT_MIN;
	HYSTERISIS_TIME = IS_TEST ? OFF_TO_ON_TEST_HYSTERISIS_SEC : OFF_TO_ON_HYSTERISIS_MIN;
}

void checkMovement()
{
	if (checkPIR()) waterfallIsRunning = 1;
}

void runWaterfall(struct runIt_S *run)
{
	run->amIrunning = 2;
	run->savedTime = sysTick;
	run->timeOn = RUNTIME;
	Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_PUMP_CTRL);
}

void topLED_run(struct runIt_S *run)
{
	if (run->amIrunning)
	{
		if (sysTick - run->savedTime >= run->timeOn)
		{
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_LED_TOP);
			run->amIrunning = 2;
		}
	}
	else
	{
		run->amIrunning = 1;
		run->savedTime = sysTick;
		Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_LED_TOP);
	}
}

void bottomLED_run(struct runIt_S *run)
{
	if (run->amIrunning)
	{
		if (sysTick - run->savedTime >= run->timeOn)
		{
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM);
			run->amIrunning = 2;
		}
	}
	else
	{
		run->amIrunning = 1;
		run->savedTime = sysTick;
		Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM);
	}
}

void bothLED_run(struct runIt_S *run)
{
	if (run->amIrunning)
	{
		if (sysTick - run->savedTime >= run->timeOn)
		{
			Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, 0, (1 << OUTPUT_LED_TOP) | (1 << OUTPUT_LED_BOTTOM));
			run->amIrunning = 2;
		}
	}
	else
	{
		run->amIrunning = 1;
		run->savedTime = sysTick;
		Chip_GPIO_SetPortOutHigh(LPC_GPIO_PORT, 0, (1 << OUTPUT_LED_TOP) | (1 << OUTPUT_LED_BOTTOM));
	}
}

void stop()
{
	waterfallIsRunning = 0;
	uint32_t nowTime = sysTick;
	Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, 0, (1 << OUTPUT_PUMP_CTRL) | (1 << OUTPUT_LED_TOP) | (1 << OUTPUT_LED_BOTTOM));
	while (sysTick - nowTime <= HYSTERISIS_TIME)
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
	waterfallIsRunning = 0;
	Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, 0, (1 << OUTPUT_PUMP_CTRL) | (1 << OUTPUT_LED_TOP) | (1 << OUTPUT_LED_BOTTOM));

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

void SysTick_Handler(void)
{
	sysTick++;
}
