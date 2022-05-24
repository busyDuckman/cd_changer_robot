#include <TimerOne.h>
#include <digitalWriteFast.h>
#include <stdlib.h>

//--------------------------------------------------------------------------------------------------------
// Global Variables
//--------------------------------------------------------------------------------------------------------
#define COMMAND_LINE_BUFFER_SIZE 64
//#define NUM_INPUT_PINS 20

Stream *commandLineStream;
Print *outStream;
char ParseBuffer[COMMAND_LINE_BUFFER_SIZE];

bool debugMode = false;
int vert_jog_len = 1000;

char commandLineBuffer[COMMAND_LINE_BUFFER_SIZE];
int commandLineBufferLen = 0;

const int numInputPins = 20;
const int stdDebounce = 1;
const int stdCommmit = 1;
int inputPins[numInputPins] = { 38,39,40,41,
															  42,43,44,45,
														  	46,47,48,49,
														  	50,51,52,53,
														  	30,31,32,33};
volatile long pulseCount [numInputPins];

volatile bool lastState[numInputPins];
bool lastStateDelta[numInputPins]; //for displaying changes
long timeOfNewState [numInputPins];

//Debouncing method used:
// -Any signal change with duration < commitTime, is ignored as noise
// -Once the signal is recognised as real, any change occuring, before
//  debounceTimetime has expired is ignored as bounce
int commitTime[numInputPins] = {};
int debounceTime[numInputPins] = {};
volatile bool anyChange;


#define H_BRIDGE_TRANSITION_DELAY 1

// output pins
#define GRIPPER_PIN 27
#define ERROR_LIGHT_PIN 23
#define ARM_UP_PIN 24
#define ARM_LEFT_PIN 25
#define BUSY_LIGHT_PIN 26
#define ARM_DOWN_PIN 28
#define ARM_RIGHT_PIN 29

// input pins
#define GRIP_SENSOR_PIN 40
#define ARM_VERTICAL_TICK_INDEX 4
#define ARM_VERTICAL_STOP_INDEX 19
#define ARM_CD_SENSOR_INDEX 2
#define ARM_LEFT_SENSOR_INDEX 6
#define ARM_MIDDLE_SENSOR_INDEX 1
#define ARM_RIGHT_SENSOR_INDEX 3

//--------------------------------------------------------------------------------------------------------
// commands
//--------------------------------------------------------------------------------------------------------
const char  pmCmdHelp[]          PROGMEM = "help";
const char  pmCmdGrip[]          PROGMEM = "grip";
const char  pmCmdRelease[]       PROGMEM = "release";
const char  pmCmdDrop[]          PROGMEM = "drop";
const char  pmCmdArmToLeft[]     PROGMEM = "full_left";
const char  pmCmdArmLeftToMiddle[]  PROGMEM = "left_to_middle";
const char  pmCmdArmRightToMiddle[] PROGMEM = "right_to_middle";
const char  pmCmdArmToRight[]    PROGMEM = "full_right";

const char  pmCmdJogArmUp[]      PROGMEM = "up";
const char  pmCmdJogArmDown[]    PROGMEM = "down";
const char  pmCmdArmLeft[]       PROGMEM = "left";
const char  pmCmdArmRight[]      PROGMEM = "right";
const char  pmCmdPinStates[]     PROGMEM = "who";

const char  pmCmdTop[]           PROGMEM = "top";
const char  pmCmdVDist[]         PROGMEM = "set_v_dist";

const char  pmCmdSpear[]         PROGMEM = "spear";

//--------------------------------------------------------------------------------------------------------
// Setup
//--------------------------------------------------------------------------------------------------------
void setup()
{
	Serial.begin(9600);
	commandLineStream = &Serial;
	outStream = &Serial;
	//setup all pins
	for(int i=0; i<numInputPins; i++)
	{
		pinMode(inputPins[i], INPUT);
	}

	//read all pins
	for(int i=0; i<numInputPins; i++)
	{
		lastState[i] = digitalRead(inputPins[i]);
		lastStateDelta[i] = lastState[i];
		timeOfNewState[i] = 0;
		commitTime[i] = stdCommmit;
		debounceTime[i] = stdDebounce;
		pulseCount[0] = 0;
	}

	//adjust custom debounce times here

	// pin modes
	pinMode(GRIPPER_PIN, OUTPUT);
	digitalWrite(GRIPPER_PIN, 0);
	pinMode(ERROR_LIGHT_PIN, OUTPUT);
	digitalWrite(ERROR_LIGHT_PIN, 0);
	pinMode(ARM_UP_PIN, OUTPUT);
	digitalWrite(ARM_UP_PIN, 0);
	pinMode(ARM_LEFT_PIN, OUTPUT);
	digitalWrite(ARM_LEFT_PIN, 0);
	pinMode(BUSY_LIGHT_PIN, OUTPUT);
	digitalWrite(BUSY_LIGHT_PIN, 0);
	pinMode(ARM_DOWN_PIN, OUTPUT);
	digitalWrite(ARM_DOWN_PIN, 0);
	pinMode(ARM_RIGHT_PIN, OUTPUT);
	digitalWrite(ARM_RIGHT_PIN, 0);

	anyChange = false;
  Timer1.initialize(500);
  Timer1.attachInterrupt(readInputs);

	outStream->println(F("info: Ready."));
	reportSuccess(true);
}

//--------------------------------------------------------------------------------------------------------
// loop
//--------------------------------------------------------------------------------------------------------
void loop()
{
	//---------- house keeping
	long now = millis();

	//readInputs();

	//---------- debug sensor redings
	//quickly get and reset the anychange flag.
	bool sensorEvent;
	noInterrupts();
	sensorEvent = anyChange;
	anyChange = false;
	interrupts();

	// if(sensorEvent)	{
	// 	updateAndPrintPinStates(true);
	// }

	//---------- Process commands
	scanCommandLineStream();
}

bool updateAndPrintPinStates(bool onlyChanged, int specific_pin)
{
	bool changeDetected = false;

	outStream->println(onlyChanged ? F("info: listing changed states: ")
																 : F("info: listing states: "));

	for(int i=0; i<numInputPins; i++)
	{
		bool state = getInputStatus(i);
		int pin = inputPins[i];
		if ((specific_pin >= 0) && (specific_pin != pin)) {
			continue;
		}

		if((state != lastStateDelta[i]) || !onlyChanged)
		{
			//we detected a change, nb we will mis many that happen while executing a command
			//this is only a debug tool, to help develop the system
			changeDetected = true;
			outStream->print(F(" pin="));
			// outStream->print(i);
			// outStream->print(F(" ("));
			outStream->print(pin);
			// outStream->print(F(") = "));

			outStream->print(F(" state="));
			outStream->print(state);
			outStream->print(F(", transitions="));
			outStream->print(pulseCount[i]);
			outStream->println();

			lastState[i] = state;
		}
	}


	if(changeDetected) {
		outStream->println("info: pin stage(s) changed");
	}
	else {
			outStream->println("info: no pin stage(s) were changed");
	}

	return changeDetected;
}

bool getInputStatus(int inputPin)
{
	bool r;
	noInterrupts();
	r = lastState[inputPin];
	interrupts();

	return r;

}

void readInputs()
{
	long now = millis();
	for(int i=0; i<numInputPins; i++)
	{
		bool state = digitalReadFast(inputPins[i]);
		if(state != lastState[i]) {
			pulseCount[i]++;
			lastState[i] = state;
			anyChange = true;
		}

	//
	// 	if(state == lastState[i])
	// 	{
	// 		if(timeOfNewState[i] > debounceTime[i])
	// 		{
	// 			//debounce complete, reset timer to allow new changes
	// 			timeOfNewState[i] = 0;
	// 		}
	// 		else if(timeOfNewState[i] < commitTime[i])
	// 		{
	// 			//a change was flagged but fluttered back before the commit time threshold.
	// 			timeOfNewState[i] = 0;
	// 		}
	// 	}
	// 	else
	// 	{
	// 		//time = 0 indicates a change is permisable (not in debounce)
	// 		if(timeOfNewState[i] == 0)
	// 		{
	// 			//state yet to be altered
	// 			timeOfNewState[i] = now;
	// 		}
	// 		else if(timeOfNewState[i] > commitTime[i])
	// 		{
	// 			//we have been consistantly in the state long enought to commit to it.
	// 			pulseCount[i]++;
	// 			lastState[i] = state;
	// 			anyChange = true;
	// 		}
	// 	}
	}
}

//--------------------------------------------------------------------------------------------------------
// commands
//--------------------------------------------------------------------------------------------------------

void doHelpCmd(const char* cmd)
{
	outStream->println(F("Command Reference:"));
	outStream->println(F("       help: this message"));
	outStream->println(F("       grip: griper will hold disk"));
	outStream->println(F("    release: griper wont hold disk"));
	outStream->println(F("       drop: griper lets go of disk"));
	outStream->println(F("             then retuns to grip state."));
	outStream->println(F("        up: jog arm up a bit."));
	outStream->println(F("      down: jog arm down a bit."));
	outStream->println(F("      left: jog arm left a bit."));
	outStream->println(F("     right: jog arm right a bit."));
	outStream->println(F("       who: List the state of the sensor pins."));
}


void doCmd(const char* cmd)
{
	if(isCommand(cmd,  pmCmdHelp)) {
		doHelpCmd(NULL);
	}
	else if(isCommand(cmd,  pmCmdGrip)) {
		doGripCmd(NULL);
	}
	else if(isCommand(cmd,  pmCmdRelease)) {
		doCmdRelease(NULL);
	}
	else if(isCommand(cmd,  pmCmdDrop)) {
		doDropCmd(NULL);
	}
	else if(isCommand(cmd,  pmCmdArmLeft)) {
		jogMotor(ARM_LEFT_PIN, 500);
	}
	else if(isCommand(cmd,  pmCmdArmRight)) {
		jogMotor(ARM_RIGHT_PIN, 500);
	}
	else if(isCommand(cmd,  pmCmdArmLeftToMiddle)) {
		driveMotorUntil(ARM_LEFT_PIN, ARM_MIDDLE_SENSOR_INDEX, LOW, 3000);
	}
	else if(isCommand(cmd,  pmCmdArmRightToMiddle)) {
		// The cam rotation makes it longer for the sensor to engage moving
		// from this side.
		driveMotorUntil(ARM_RIGHT_PIN, ARM_MIDDLE_SENSOR_INDEX, LOW, 4000);
	}
	else if(isCommand(cmd,  pmCmdArmToLeft)) {
		driveMotorUntil(ARM_LEFT_PIN, ARM_LEFT_SENSOR_INDEX, LOW, 7000);
	}
	else if(isCommand(cmd,  pmCmdArmToRight)) {
		driveMotorUntil(ARM_RIGHT_PIN, ARM_RIGHT_SENSOR_INDEX, LOW, 7000);
	}
	else if(isCommand(cmd,  pmCmdJogArmUp)) {
		//jogMotor(ARM_UP_PIN, 200);
		driveMotor(ARM_UP_PIN, ARM_VERTICAL_TICK_INDEX, vert_jog_len, 100);
	}
	else if(isCommand(cmd,  pmCmdJogArmDown)) {
		// jogMotor(ARM_DOWN_PIN, 200);
		driveMotor(ARM_DOWN_PIN, ARM_VERTICAL_TICK_INDEX, vert_jog_len, 100);
	}
	else if(isCommand(cmd, pmCmdPinStates)) {
		const char* arg = getArg(cmd);
		if(arg == NULL) {
			updateAndPrintPinStates(false, -1);
		}
		else {
			int pin = atoi(arg);
			updateAndPrintPinStates(false, pin);
		}

		reportSuccess(true);
	}
	else if(isCommand(cmd,  pmCmdTop)) {
		driveMotorUntil(ARM_UP_PIN, ARM_VERTICAL_STOP_INDEX, LOW, 5000);
	}
	else if(isCommand(cmd,  pmCmdVDist)) {
		const char* arg = getArg(cmd);
		if(arg == NULL) {
			outStream->println(F("error: could not parse"));
			reportSuccess(false);
			return;
		}
		vert_jog_len = max(atoi(arg), 1);
		outStream->print(F("info: vert_jog_len="));
		outStream->println(vert_jog_len);
		reportSuccess(true);
	}
	else if(isCommand(cmd,  pmCmdSpear)) {
		driveMotorUntil(ARM_DOWN_PIN, ARM_CD_SENSOR_INDEX, LOW, 2000);
	}
	else {
		outStream->print(F("error: unknown command: \""));
		outStream->print(cmd);
		outStream->println('\"');
		reportSuccess(false);
	}
}

void doGripCmd(const char* cmd)
{
	outStream->println(F("info: Gripping"));
	digitalWrite(GRIPPER_PIN, LOW);
	delay(H_BRIDGE_TRANSITION_DELAY);

	reportSuccess(isDiskInGripper());
}

void doCmdRelease(const char* cmd)
{
	outStream->println(F("info: releasing"));
	digitalWrite(GRIPPER_PIN, HIGH);
	delay(H_BRIDGE_TRANSITION_DELAY);

	reportSuccess(!isDiskInGripper());
}

void doDropCmd(const char* cmd)
{
	outStream->println(F("info: releasing"));
	digitalWrite(GRIPPER_PIN, HIGH);
	delay(500);
	digitalWrite(GRIPPER_PIN, LOW);
	delay(H_BRIDGE_TRANSITION_DELAY);
	reportSuccess(!isDiskInGripper());
}

//--------------------------------------------------------------------------------------------------------
// Drive Motors
//--------------------------------------------------------------------------------------------------------
void jogMotor(int pin, int ms)
{
	digitalWrite(pin, 1);
	delay(ms);
	digitalWrite(pin, 0);
	reportSuccess(true);
}

void driveMotor(int drive_pin, int tick_pin_index, int ticks, int timeOutMsPerTick)
{
	long start = millis();
	long startCount = pulseCount[tick_pin_index];
	digitalWrite(drive_pin, 1);
	int total_time = timeOutMsPerTick * ticks;
	bool error = false;

	while ((pulseCount[tick_pin_index] - startCount) < ticks) {
		if ((millis() - start) > total_time) {
			error = true;
			break;
		}
		// delay();
	}
	digitalWrite(drive_pin, 0);

	if(error) {
		outStream->println(F("error: motor movement timed out."));
	}

	outStream->print(F("info: actual_ticks="));
	outStream->print(pulseCount[tick_pin_index] - startCount);
	outStream->print(F(", requested_ticks="));
	outStream->println(ticks);
	reportSuccess(!error);
}

void driveMotorUntil(int drive_pin, int stop_pin_index, int stop_state, int timeOutMs)
{
	if (lastState[stop_pin_index] == stop_state) {
		outStream->println(F("info: already at limit switch."));
		reportSuccess(true);
		return;
	}
	int startPulseCount = pulseCount[stop_pin_index];
	long start = millis();
	digitalWrite(drive_pin, 1);
	bool error = false;

	while (pulseCount[stop_pin_index] == startPulseCount) {
		if ((millis() - start) > timeOutMs) {
			error = true;
			break;
		}
		// delay();
	}
	digitalWrite(drive_pin, 0);

	if(error) {
		outStream->println(F("error: motor movement timed out."));
	}
	reportSuccess(!error);
}

//--------------------------------------------------------------------------------------------------------
// read sensors
//--------------------------------------------------------------------------------------------------------
bool isDiskInGripper()
{
	return !digitalReadFast(GRIP_SENSOR_PIN);
}



//--------------------------------------------------------------------------------------------------------
// misc
//--------------------------------------------------------------------------------------------------------

void scanCommandLineStream()
{
	if(commandLineStream != NULL)
	{
		if (commandLineStream->available() > 0)
			//if (client.available() > 0)
		{
			//char c = client.read();
			char c = commandLineStream->read();  //was client.read
			switch (c)
			{
			case '\r':
			case '\n':
				if(commandLineBufferLen > 0)
				{
					doCmd(commandLineBuffer);
				}
				commandLineBuffer[0] = '\0';
				commandLineBufferLen = 0;
				outStream->print(F(">"));
				break;
			default:
				if((c < ' ') || (c > '~'))
				{
					//trash incoming, clear buffer.
					commandLineBuffer[0] = '\0';
					commandLineBufferLen = 0;
				}
				else if(commandLineBufferLen < (COMMAND_LINE_BUFFER_SIZE-1))
				{
					commandLineBuffer[commandLineBufferLen] = c;
					commandLineBuffer[commandLineBufferLen+1] = '\0';
					commandLineBufferLen++;
				}
				break;
			}
		}
	}
}

bool myStrncasecmp(const char* str, const char* pmStr, int len)
{
	strcpy_P(ParseBuffer, pmStr);
	return strncasecmp(str, ParseBuffer, len) == 0;

	//return strncasecmp_P(str, pmStr, len) == 0;
}

const char* getArg(const char* cmd)
{
	for(int i=0; i<(strlen(cmd)-1); i++)
	{
		if(cmd[i] == ' ') {
			return (cmd + i);
		}
	}
	return NULL;
}

bool isCommand(const char* cmd, const char* pmCmd)
{
	int len = strlen_P(pmCmd);
	if(len <= strlen(cmd))
	{
		//safe compare as the input is longer than the command
		//the extra chars could be <args>
		if(myStrncasecmp(cmd, pmCmd, len))
		{
			//check for command, without args
			if(len == strlen(cmd))
			{
				return true;
			}
			//check for a space, this means that args follow
			return (cmd[len] == (byte)' ');
		}
	}
	return false;
}

void reportSuccess(bool worked)
{
	if(worked)
	{
		outStream->println(F("_OK_"));
	}
	else
	{
		outStream->println(F("_FAIL_"));
	}
}
