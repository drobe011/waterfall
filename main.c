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

#define WWDT_ON

const uint32_t ExtRateIn = 0;
const uint32_t OscRateIn = 0;

/* SYSTEM SEQUENCE
 * POWER ON -> INIT STATE
 * 		CHECK IF IN TEST MODE
 * 		CHECK WATER LEVEL (ENTER ERROR IF LOW)
 * 		TEST PUMP AND LIGHTS (TOP = TESTING PUMP, BOTTOM = REMAINING PIR STAB TIME)
 * 		WAIT FOR PIR TO STABALIZE
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
 * 		BLINK BOTH LEDs
 * ERROR -> CHECK FOR MOVEMENT
 *
 */

//PIN MAPPING (NO FUNCTIONS DEFINED IN SWM)
#define INPUT_PIR 0
#define INPUT_TESTPIN 4
#define INPUT_FLOAT 6
#define OUTPUT_DEBUG_SYSTICK 7
#define OUTPUT_LED_TOP 10
#define OUTPUT_LED_BOTTOM 11
#define OUTPUT_PUMP_CTRL 13

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
#define ON_TIME_ERROR_BLINK_SEC (5*1000)
#define LED_STATES 4

//GP MACROS
#define TIME_TO_CHECK_STATE (ticksNow - lastStateCheck >= STATE_CHECK_MS)
#define TIME_TO_TURN_OFF (ticksNow - LEDopMode[STARTWFALL].savedTime >= LEDopMode[STARTWFALL].timeOn)
#define RUN_WATERFALL_OPERATION (LED_run(&LEDopMode[runState]))
#define INIT_WATERFALL_OPERATION (startWaterfall(&LEDopMode[runState]))
#define TURN_TOP_LED_ON (Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_LED_TOP))
#define TURN_TOP_LED_OFF (Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_LED_TOP))
#define TURN_BOTTOM_LED_ON (Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM))
#define TURN_BOTTOM_LED_OFF (Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM))
#define TURN_BOTH_LED_ON (Chip_GPIO_SetPortOutHigh(LPC_GPIO_PORT, 0, (1 << OUTPUT_LED_TOP) | (1 << OUTPUT_LED_BOTTOM)))
#define TURN_BOTH_LED_OFF (Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, 0, (1 << OUTPUT_LED_TOP) | (1 << OUTPUT_LED_BOTTOM)))
#define TURN_PUMP_ON (Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, OUTPUT_PUMP_CTRL))
#define TURN_PUMP_OFF (Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, OUTPUT_PUMP_CTRL))
#define TURN_ALL_OFF (Chip_GPIO_SetPortOutLow(LPC_GPIO_PORT, 0, (1 << OUTPUT_LED_TOP) | (1 << OUTPUT_LED_BOTTOM) | (1 << OUTPUT_PUMP_CTRL)))

//PROGRAM FLOW TIMING
#define STATE_CHECK_MS 10

//GLOBALS
static volatile uint32_t ticksNow;
uint32_t RUNTIME, HYSTERISIS_TIME; //HOLDS TIMES BASED ON TEST MODE
uint32_t waterfallIsRunning = 0;
enum {
	INIT, RUNNING, TURN_OFF_TIME
};
typedef enum {
	STARTWFALL = 0, TOPLED, BOTTOMLED, BOTHLED, OFF
} runState_T;

struct runIt_S {
	runState_T LED;
	uint32_t mode;
	uint32_t savedTime;
	uint32_t timeOn;
};

//STATE FUNCTION PROTOTYPES
void init(void);
void checkMovement(void);
void startWaterfall(struct runIt_S *);
void LED_run(struct runIt_S *);
void stop(void);
void error(void);
void (*p_StateFunction)() = NULL;

//WORKER FUNCTION PROTOTYPES
uint32_t blinkLED(runState_T whichLED, uint32_t blinkDelay,
		uint32_t ignoreWaterLevel);
void setupPort(void);
uint32_t waterLevelGood(void) __attribute__((always_inline));
uint32_t movementDetected(void) __attribute__((always_inline));
uint32_t checkIfTestMode(void) __attribute__((always_inline));

int main(void) {
	runState_T runState = STARTWFALL;
	uint32_t lastStateCheck = ticksNow;

	struct runIt_S LEDopMode[LED_STATES] = { { TOPLED, 0, 0, RUNTIME }, { TOPLED, 0, 0,
			ON_TIME_TOPLED_SEC }, { BOTTOMLED, 0, 0, ON_TIME_BOTTOMLED_SEC }, {
					BOTHLED, 0, 0, ON_TIME_BOTHLED_SEC } };
	p_StateFunction = &init;

	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000);

	setupPort();

	while (1) {
		if (TIME_TO_CHECK_STATE) {
			if (waterfallIsRunning && !waterLevelGood()) {
				p_StateFunction = &error;
				LEDopMode[runState].mode = INIT;
				waterfallIsRunning = FALSE;
			} else if (waterfallIsRunning) {
				if (LEDopMode[runState].mode == TURN_OFF_TIME) {
					LEDopMode[runState].mode = INIT;
					runState = runState < OFF ? runState + 1 : TOPLED;
				}
				if (movementDetected())
					LEDopMode[STARTWFALL].savedTime = ticksNow;
				if (TIME_TO_TURN_OFF) {
					runState = STARTWFALL;
					waterfallIsRunning = FALSE;
					LEDopMode[runState].mode = INIT;
					p_StateFunction = &stop;
				} else if (runState == STARTWFALL)
					INIT_WATERFALL_OPERATION;
				else
					RUN_WATERFALL_OPERATION;
			} else
				p_StateFunction();
			lastStateCheck = ticksNow;
			Chip_GPIO_SetPinToggle(LPC_GPIO_PORT, 0, 7);
#ifdef WWDT_ON
			Chip_WWDT_Feed(LPC_WWDT);
#endif
		}
	}
}

void setupPort() {
#ifdef WWDT_ON
	uint32_t wdtFreq;
	Chip_Clock_SetWDTOSC(WDTLFO_OSC_0_60, 64);
	Chip_SYSCTL_PowerUp(SYSCTL_SLPWAKE_WDTOSC_PD);
	wdtFreq = Chip_Clock_GetWDTOSCRate() / 4;
	Chip_WWDT_Init(LPC_WWDT);
	Chip_WWDT_SetTimeOut(LPC_WWDT, wdtFreq / 10); /* (100ms) */
	Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);
	Chip_WWDT_Start(LPC_WWDT);
#endif
	Chip_GPIO_Init(LPC_GPIO_PORT);

	Chip_IOCON_PinSetMode(LPC_IOCON, IOCON_PIO0, PIN_MODE_PULLDN);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, OUTPUT_DEBUG_SYSTICK);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, OUTPUT_LED_TOP);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, OUTPUT_LED_BOTTOM);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, OUTPUT_PUMP_CTRL);
}

void init() {
	uint32_t IS_TEST = checkIfTestMode() ? TRUE : FALSE;

	if (waterLevelGood()) {
		TURN_PUMP_ON;
		blinkLED(TOPLED, ON_TIME_INIT_SEC, FALSE);
		TURN_PUMP_OFF;
		p_StateFunction = &checkMovement;

		if (!blinkLED(BOTTOMLED, PIR_STABILIZE_SEC - ON_TIME_INIT_SEC, FALSE))
			p_StateFunction = &error;
	} else
		p_StateFunction = &error;

	RUNTIME = IS_TEST ? ON_TEST_TIME_AFTER_MVMT_SEC : ON_TIME_AFTER_MVMT_MIN;
	HYSTERISIS_TIME =
			IS_TEST ? OFF_TO_ON_TEST_HYSTERISIS_SEC : OFF_TO_ON_HYSTERISIS_MIN;
}

void checkMovement() {
	if (movementDetected())
		waterfallIsRunning = TRUE;
}

void startWaterfall(struct runIt_S *run) {
	run->mode = TURN_OFF_TIME;
	run->savedTime = ticksNow;
	run->timeOn = RUNTIME;
	TURN_PUMP_ON;
}

void LED_run(struct runIt_S *run) {
	if (run->mode) {
		if (ticksNow - run->savedTime >= run->timeOn) {
			TURN_BOTH_LED_OFF;
			run->mode = TURN_OFF_TIME;
		}
	} else {
		run->mode = RUNNING;
		run->savedTime = ticksNow;
		if (run->LED == TOPLED)
			TURN_TOP_LED_ON;
		else if (run->LED == BOTTOMLED)
			TURN_BOTTOM_LED_ON;
		else
			TURN_BOTH_LED_ON;
	}
}

void stop() {
	uint32_t nowTime = ticksNow;
	TURN_ALL_OFF;

	waterfallIsRunning = FALSE;

	while (ticksNow - nowTime <= HYSTERISIS_TIME) {
		if (!waterLevelGood())
			break;
#ifdef WWDT_ON
		Chip_WWDT_Feed(LPC_WWDT);
#endif
	}
	p_StateFunction = &checkMovement;
}

void error() {
	waterfallIsRunning = FALSE;
	TURN_ALL_OFF;

	blinkLED(BOTHLED, ON_TIME_ERROR_BLINK_SEC, TRUE);

	p_StateFunction = &checkMovement;
}

uint32_t blinkLED(runState_T whichLED, uint32_t blinkDelay,
		uint32_t ignoreWaterLevel) {
	uint32_t ticksThen = ticksNow;

	if (whichLED == BOTHLED)
		TURN_BOTH_LED_ON;
	else if (whichLED == TOPLED)
		TURN_TOP_LED_ON;
	else
		TURN_BOTTOM_LED_ON;

	while (ticksNow - ticksThen <= blinkDelay) {
		//__NOP;
#ifdef WWDT_ON
		Chip_WWDT_Feed(LPC_WWDT);
#endif
		if (ignoreWaterLevel && !waterLevelGood())
			return FALSE;
	}

	TURN_BOTH_LED_OFF;

	return TRUE;
}

inline uint32_t waterLevelGood() {
	return !Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0, INPUT_FLOAT);
}

inline uint32_t movementDetected() {
	return Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0, INPUT_PIR);
}

inline uint32_t checkIfTestMode() {
	return !Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0, INPUT_TESTPIN);
}

void SysTick_Handler(void) {
	ticksNow++;
}
