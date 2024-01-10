/*

  Chicken guard

  Opens and closes the chicken door in the morning and evening.
  A light sensor is used to determine when it is opened and closed.
  However also a clock module (DS3231) can be attached to not open the door before a given time.
  This for the summertime to not open it too soon. Early in the morning when there is already light but not many people are awake the preditors are still around...
  But even without that clock module it is possible to work on time via the timer from the arduino.
  For this to work, the current time must be provided each time the arduino resets.
  This is not the case with the timer module because it works on its own and it even has a battery to keep the time when power is disconnected.
  So for that timer module the date/time must only be set once.

  Two leds also show if the door is (about to) open or close and also to show if there is an error.
  When the door is open, the red led is on. When the door is closed, the green led is on.
  The colors are chosen as such because green means the chickens are safe and red means they are unsafe.
  I chose for a 2-in-one led (with 3 connections)

  A minute or two before the opening/closing the red or green led starts to blink every second to indicate the event is about to happen.

  When it is dark and thus the door should be closed, the system checks every second if the magnetic switch still says that the door is closed.
  If not, then it closes the door again. This in case an agressor tries to open the door (by pulling it).

  If the door is closed manually during the day (for example because the chickens may not enter or exit the henhouse), the system will not run the motor anymore to open or close
  the door until the door is manually opened again. This must of course be done during light when the door is supposed to be open.
  Also the leds will not change anymore until the door is opened again.
  Opening the door again can then be done during light or dark.
  In dark, the door will then close again via the motor and when light the door will stay open until it becomes dark.

  At the light detector there is also a 3-way switch where in normal mode the ldr is connected.
  A second mode is where the ldr is disconnected resulting in an input value of 0 and this results in an immediate close of the door.
  A third mode is where the ldr is connected to +5 resulting in a maximum input value and this results in an immediate open of the door.
  
  If there is an error, the red led will blink as many times as the error status, then once green, then once off and then repeat this.

  This error can be reset by switching the switch between immediate close and open within 5 seconds.

  When power is switched on, the first minute nothing will happen and the system waits for commands you can enter.
  Each time a command is given, the minute starts again. There are commands to manually open and close the door, doing this repeatingly, ... H gives a help screen.

  Then it starts with first closing the door (if it is not closed yet) and then depending on the light it stays closed (night) or opens (day).
  This is done as such because a magnet switch can detect if the door is closed. When the magnet is not at the switch, the door is (possible only partially) open.
  As such the system knows after this exactly if the door is open or closed.
  So closing the door is done until the magnet switch says it is closed but there is also a timer to make sure the motor doesn't stay on if something goes wrong and then the system goes into error
  Opening the door is done with a timer because the system has no detector if it is completely open. It is not a problem if the motor runs a bit longer because this means only that the cable is looser.
  Note that not a hard cable is used on the door but an elastic. That way when the door is closed there is some elastic on the cable.
  Also when the system detects that the door is closed it will close it again for a couple of milliseconds to tighten it extra.
  This is tried max 10 times because it can happen that the motor cannot hold the force and returns a bit but it practice most of the time only 1 time must be tried.
  Since my door is a trap door (like a medieval castle), also an elastic is on the outside to pull it open the first centimeters (and then gravity takes over).
  I use an elastic used for bicycles to hold things on the rack.

  Make sure that the door is not closed and that there is enough light (daytime) the first time the system is taken into operation.
  That will make the system setup automatically properly. The ax will turn one way or another and eventually wind up the elastic and close the door.
  To open it, the motor will turn the other way and thus unwind the elastic.

  Also when in normal operation, commands can still be given. Note that the first minute alot is shown in the output but once in normal operation there is no much logging anymore.
  This can be changed by the L command. Again, the H command shows a help screen with all the commands.

  Note also that by default de arduino resets when the monitor is started (Ctrl Shift M)
  So you also loose all your variables :-(
  This can be fixed with a condensator between the reset pin and gnd. I have used 0.47 µF and this works fine.
  However note that it must be disconnected when a new sketch is uploaded. Otherwise you get an error that it could not upload. I use a small switch to do that.

  Next to controling the door, also the water level can be monitored. I have two water level switched in the water tank. One shows if the water is close to empty, the other if it is really empty.

  When SERIAL1 is defined then communication can also be done via Serial1. A bluetooth HC-05 module can be connected to make this possible.

  When EEPROMModule is defined then some data is written to EEPROM such that the values are not lost when restarted

  If EthernetModule is defined and the arduino is connected to ethernet then two extra modules also become available:
   MQTTModule to send MQTT messages to/from the arduino to the MQTT server. This can be used in combination with home assistant to monitor the status
   NTPModule to synchronise the date/time with internet time. This is done every 30 days and at startup when ClockModule is not defined

*/

#include <Arduino.h>

#define ClockModule                 // If defined then compile with the clock module code
#define EthernetModule              // If defined then connect to ethernet
#define MQTTModule                  // If defined then compile with MQTT support - can be used via Home Assistant
#define NTPModule                   // If defined then get internet time
#define EEPROMModule                // If defined then store data in EEPROM
#define SERIAL1                     // If defined then also communicate via Serial1
//#define CONTROLBUILTIN              // If set then set the BUILTIN LED

#if !defined EthernetModule
# undef MQTTModule                  // MQTT can't work without ethernet
# undef NTPModule                   // NTP can't work without ethernet
#endif

#if defined MQTTMModule

/*

Following must be added to configuration.yaml of HA:

logbook:
  exclude:
    entities:
      - sensor.chickenguard_time_now
      - sensor.chickenguard_ldr
      - sensor.chickenguard_ldr_average
      - sensor.chickenguard_monitor

*/

#endif

const int ldrOpenNow = 1020;        // light value for door open now
const int ldrCloseNow = 0;          // light value for door close now

const int nMeasures = 5;            // number of measures to do on which an average is made to determine if door is opened (avg >= ldrMorning) or closed (avg <= ldrEvening)
const int measureEverySeconds = 60; // number of seconds between light measurements and descision if door should be closed or opened
const int motorClosePin = 4;        // motor turns one way to close the door
const int motorOpenPin = 5;         // motor turns other way to open the door
const int motorPWMPin = 44;         // motor PWM pin: 490 Hz (4 and 13 are 980 Hz but 490 seems to work alot better) - See https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
const int ledClosedPinInit = 6;
int ledClosedPin = ledClosedPinInit;// green LED - door closed  D6
int ledOpenedPin = 7;               // red LED - door open  D7
const int magnetPin = 3;            // magnet switch  D3
const int ldrPin = A2;              // LDR (light sensor) A2

int ledNotEmptyPin = 9;             // green LED - enough water  D9
int ledEmptyPin = 8;                // red LED - (almost) empty water  D8
const int almostEmptyPin = 11;      // almost empty switch D11
const int emptyPin = 12;            // empty switch D12

// ... for this much of milliseconds and then continue closing the door
const unsigned long waitForInputMaxMs = 900000; // maximum wait time for input (900000 ms = 15 min)

int motorPWM;                       // motor PWM value - See https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/

int ldrMorning;                     // light value for door to open
int ldrEvening;                     // light value for door to close

int openMilliseconds;               // number of milliseconds to open door
int closeMilliseconds;              // maximal number of milliseconds to close door
int closeWaitTime1;                 // after this much milliseconds, stop closing door ...
int closeWaitTime2;
int closeWaitTime3;

struct
{
  char *name;
  int *variable;
  int initialValue;
} changeableData[] = 
{
  { "ldrMorning", &ldrMorning, 600 },
  { "ldrEvening", &ldrEvening, 40 },
  { "motorPWM", &motorPWM, 255 }, // 255 = full speed
  { "openMilliseconds", &openMilliseconds, 1100 },
  { "closeMilliseconds", &closeMilliseconds, 3000 },
  { "closeWaitTime1", &closeWaitTime1, 1400 },
  { "closeWaitTime2", &closeWaitTime2, 2000 },
  { "closeWaitTime3", &closeWaitTime3, 30 },
};

int status = 0;                     // status. 0 = all ok
int toggle = 0;                     // led blinking toggle
bool logit = false;                 // log to monitor
bool isClosedByMotor;               // is door closed by motor
bool keepOpen = false;              // keep door forced open
bool keepClosed = false;            // keep door forced closed

uint16_t lightMeasures[nMeasures];  // array with light measurements (nMeasures measures taken every measureEverySeconds seconds). Contains the last nMeasures light measures
int measureIndex = 0;               // position in lightMeasures for next measurement.

uint16_t ldrMinimum = 9999;         // minimum ldr value
uint16_t ldrMaximum = 0;            // maximum ldr value

const byte startHourOpen = 7;       // minimum hour that door may open
const byte startMinuteOpen = 0;     // minimum minute that door may open

bool hasClockModule = false;        // Is the clock module detected

#if defined ClockModule
byte hourOpened = 0;                // hour of last door open
byte minuteOpened = 0;              // minute of last door open
byte secondOpened = 0;              // second of last door open

byte hourClosed = 0;                // hour of last door close
byte minuteClosed = 0;              // hour of last door close
byte secondClosed = 0;              // hour of last door close
#else
unsigned long msTime = 0;           // millis() value of set time
int dayTime = 0;                    // set day at msTime
int monthTime = 0;                  // set month at msTime
int yearTime = 0;                   // set year at msTime
int hourTime = 0;                   // set hour at msTime
int minuteTime = 0;                 // set minute at msTime
int secondsTime = 0;                // set seconds at msTime

unsigned long msOpened = 0;         // millis() value of last door open
unsigned long msClosed = 0;         // millis() value of last door close
#endif

unsigned long timePrevReset = 0;    // time when a previous open now or close now was done
bool openPrevReset;

const byte dstDay = 1;              // day on which DST is changed. 1 = sunday
const byte dstMinutes = 60;         // number of minutes on DST change

const byte summerMonth = 3;         // month that summertime begins
const int summerDstDayWeek = -1;    // week in summerMonth that summertime begins. Positive means starting from the beginning from the month, negative from the end of the month (-1 = last week)
const byte summerHour = 2;          // hour that summertime begins

const byte winterMonth = 10;        // month that wintertime begins
const int winterDstDayWeek = -1;    // week in winterMonth that summertime begins. Positive means starting from the beginning from the month, negative from the end of the month (-1 = last week)
const byte winterHour = 3;          // hour that wintertime begins

bool dstAdjust = true;              // Is there still a possible adjust today?

unsigned long PrevTime;             // Time to execute every second chicken loop
int measureEverySecond;

bool blinkEmpty;                    // blink for (almost) empty

unsigned long PrevSyncTime;         // previous time a sync was done

#define SyncTime 30 * (24 * 60 * 60 * 1000) // sync every x days

void(* resetFunc) (void) = 0;       //declare reset function at address 0

void printSerial(char *data)
{
  int l = strlen(data);
  while (l > 0 && (data[l - 1] == '\r' || data[l - 1] == '\n'))
    data[--l] = 0;
  Serial.print(data);
# if defined SERIAL1
    Serial1.print(data);
# endif
}

void printSerialInt(int a)
{
  Serial.print(a);
# if defined SERIAL1
    Serial1.print(a);
# endif
}

void printSerialln(char *data)
{
  if (data != NULL)
    printSerial(data);
  Serial.println();
# if defined SERIAL1
    Serial1.println();
# endif
}

void printSerialln()
{
  printSerialln(NULL);
}

void showChangeableData(char *name)
{
  char buf[50];
  bool found = false;

  for (int i = 0; i < sizeof(changeableData) / sizeof(*changeableData); i++)
  {
    if (name == NULL || *name == 0 || strcasecmp(name, changeableData[i].name) == 0)
    {
      sprintf(buf, "%s = %d", changeableData[i].name, *(changeableData[i].variable));
      if (name != NULL && *name)
        setMQTTMonitor(buf);
      printSerialln(buf);
      found = true;
    }
  }
  if (!found)
  {
    printSerialln("Variable not found in changeable data");
    setMQTTMonitor("Variable not found in changeable data");
  }
}

void showChangeableData()
{
  showChangeableData(NULL);
}

void setChangeableData()
{
  for (int i = 0; i < sizeof(changeableData) / sizeof(*changeableData); i++)
    *(changeableData[i].variable) = changeableData[i].initialValue;

# if defined EEPROMModule
    readChangeableData();
# endif

  showChangeableData();
}

// arduino function called when it starts or a reset is done
void setup(void)
{
  Serial.begin(9600);
  Serial.setTimeout(60000);
# if defined SERIAL1
    Serial1.begin(9600);
    Serial1.setTimeout(60000);
# endif

  printSerialln("Chicken hatch 05/01/2024. Copyright peno");

  setChangeableData();

#if defined ClockModule
  hasClockModule = InitClock();
  if (hasClockModule)
    printSerialln("Clock module found");
  else
    printSerialln("Clock module not found");
#else
  printSerialln("Clock module not included");
#endif

  pinMode(motorClosePin, OUTPUT);
  pinMode(motorOpenPin, OUTPUT);
  pinMode(motorPWMPin, OUTPUT);
  pinMode(ldrPin, INPUT);
  pinMode(magnetPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(ledOpenedPin, OUTPUT);
  digitalWrite(ledOpenedPin, LOW);
  pinMode(ledClosedPin, OUTPUT);
  digitalWrite(ledClosedPin, LOW);
  
  pinMode(ledNotEmptyPin, OUTPUT);
  digitalWrite(ledNotEmptyPin, LOW);
  pinMode(ledEmptyPin, OUTPUT);
  digitalWrite(ledEmptyPin, LOW);
  pinMode(almostEmptyPin, INPUT_PULLUP);
  pinMode(emptyPin, INPUT_PULLUP);

# if defined EthernetModule
    setupEthernet();
# endif

# if defined MQTTModule
    printSerialln("MQTT enabled");
    
    setupMQTT();

    loopEthernet();
    loopMQTT(true);
    setMQTTDoorStatus("Setup");
    setMQTTWaterStatus("Setup");
    setMQTTMonitor("Setup");
    loopEthernet();
    loopMQTT(true);
# else
    printSerialln("MQTT not enabled");
# endif

#if defined NTPModule
  printSerialln("NTP module enabled");
  InitUdp();

# if !defined ClockModule
    SyncDateTime();
# endif  
#else
  printSerialln("NTP module not enabled");
#endif

  SetLEDOff();

  SetStatusLed(false);

  logit = true;

  isClosedByMotor = IsClosed();

  SetLEDOpenClosed();

  LightMeasurement(true); // fill the whole light measurement array with the current light value

  // First 60 seconds chance to enter commands
  for (int i = 59; i >= 0; i--)
  {
    loopEthernet();
    loopMQTT(false);

    info(i, logit);

    setMQTTTime();

    switch (Command(true))
    {
      case 1:
        i = 60; // When a command was given then wait again 60 seconds
        break;
      case 2:
        i = 0;
        break;
    }

    delay(1000);
  }

  setMQTTMonitor("");

  logit = false;

  printSerialln("Starting");

  Close(); // Via the magnet switch we know for sure that the door is closed hereafter

  LightMeasurement(true); // fill the whole light measurement array with the current light value

  ProcessDoor(true); // opens the door if light and lets it closed if dark

  measureEverySecond = measureEverySeconds;
  PrevTime = PrevSyncTime = millis();
}

void SetStatusLed(bool on)
{
#if CONTROLBUILTIN
  if (on)
    digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);
#endif
}

// Is the door closed according to the magnetic switch
bool IsClosed()
{
  return digitalRead(magnetPin) == 0;
}

// stop the motor
void MotorOff(void)
{
  digitalWrite(motorClosePin, LOW);
  digitalWrite(motorOpenPin, LOW);
}

// run the motor in opening direction
void MotorOpen(void)
{
  analogWrite(motorPWMPin, motorPWM); // See https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/

  digitalWrite(motorClosePin, LOW);
  digitalWrite(motorOpenPin, HIGH);

#if defined ClockModule
  if (hasClockModule)
    readDS3231time(&secondOpened, &minuteOpened, &hourOpened, NULL, NULL, NULL, NULL);
#else
  msOpened = millis();
#endif
}

// run the motor in closing direction
void MotorClose(void)
{
  analogWrite(motorPWMPin, motorPWM);

  digitalWrite(motorClosePin, HIGH);
  digitalWrite(motorOpenPin, LOW);

#if defined ClockModule
  if (hasClockModule)
    readDS3231time(&secondClosed, &minuteClosed, &hourClosed, NULL, NULL, NULL, NULL);
#else
  msClosed = millis();
#endif
}

// close the door
void Close(void)
{
  printSerialln("Closing door");

  if (!IsClosed())
  {
    MotorClose();

    // run the motor until magnetic switch detects closed or maximum closeMilliseconds milliseconds (for safety if something goes wrong)
    // after closeWaitTime1 the motor stops for closeWaitTime2 milliseconds such that the door is again in rest because the elastic lets it vibrate
    int ElapsedTime = 0;
    unsigned long StartTime = millis();
    bool waited = false;
    printSerialln("Closing door 1");
    while (!IsClosed() && ElapsedTime < closeMilliseconds)
    {
      unsigned long CurrentTime = millis();
      ElapsedTime = CurrentTime - StartTime; // note that an overflow of millis() is not a problem. ElapsedTime will still be correct
      if (!waited && ElapsedTime >= closeWaitTime1)
      {
        printSerialln("Closing door 2");
        MotorOff();
        delay(closeWaitTime2);
        waited = true;
        StartTime += closeWaitTime2;
        MotorClose();
        printSerialln("Closing door 3");
      }
    }
    printSerial("Closing door 4 (ElapsedTime = ");
    printSerialInt(ElapsedTime);
    printSerialln(")");

    MotorOff();

    if (ElapsedTime >= closeMilliseconds)
      status = 2; // oops, magnetic switch did not detect a door close within closeMilliseconds milliseconds
    else
    {
      status = 3; // door not closed

      delay(500); // wait a half of a second

      // close the door again to have a good close. This can be done because an elastic is used
      // sometimes this must be repeated multiple times because the motor rolls back because of the tension of the elastic. Try max 10 times
      for (int i = 0; i < 10 && status != 0; i++)
      {
        printSerial("Closing door 5 (");
        printSerialInt(i);
        printSerialln(")");

        MotorClose();

        delay(closeWaitTime3); // close only for 30 milliseconds

        MotorOff();

        delay(500); // wait a half of a second

        // check if door is closed
        if (IsClosed())
          status = 0; // yes, all ok
        else
          printSerialln("Door not closed, try again");
      }
    }

    printSerialln("Closing door 6");

    SetLEDOff();

    if (status == 0)
    {
      printSerialln("Door closed");
      SetLEDOpenClosed();
      isClosedByMotor = true;
    }
    else
      printSerialln("Oops, door *not* closed");
  }
}

// Open the door
void Open(void)
{
  printSerialln("Opening door");

  if (isClosedByMotor) // If not closed by motor then do nothing
  {
    if (IsClosed())
    {
      MotorOpen();
      // There is no detection on the door when it is open so the motor runs for openMilliseconds milliseconds to open the door
      delay(openMilliseconds);
      MotorOff();
    }

    // check if door is now really open (ie not closed)
    status = 1; // door not open
    // check max 10 times
    for (int i = 0; i < 10 && status != 0; i++)
    {
      delay(500);
      if (!IsClosed())
        status = 0; // all ok, door is open
      else
        printSerialln("Door not open, check again");
    }

    SetLEDOff();

    if (status == 0)
    {
      printSerialln("Door open");
      SetLEDOpenClosed();
      isClosedByMotor = false;
    }
    else
      printSerialln("Oops, door *not* open");
  }
}

// Do a light measurement
void LightMeasurement(bool init)
{
  int counter;

  uint16_t ldr = analogRead(ldrPin);
  if (keepClosed)
    ldr = ldrCloseNow;
  else if (keepOpen)
    ldr = ldrOpenNow;

  if (init)
    counter = nMeasures;
  else
  {
    counter = 1;
    
    if (ldr > ldrCloseNow && ldr < ldrOpenNow)
    {
      int i;
      for (i = 1; i < nMeasures && lightMeasures[i] == lightMeasures[0]; i++);
      if (i >= nMeasures && (lightMeasures[0] <= ldrCloseNow || lightMeasures[0] >= ldrOpenNow))  
        counter = nMeasures;
    }
  }
  
  for (; counter >= 1; counter--)
  {
    //do a light measure
    lightMeasures[measureIndex] = ldr;
    if (lightMeasures[measureIndex] > ldrMaximum)
      ldrMaximum = lightMeasures[measureIndex];
    if (lightMeasures[measureIndex] < ldrMinimum)
      ldrMinimum = lightMeasures[measureIndex];
    measureIndex++;
    if (measureIndex >= nMeasures)
      measureIndex = 0;
  }
}

// Calculation on light measurements
void LightCalculation(uint16_t &average, uint16_t &minimum, uint16_t &maximum, bool ahead = false)
{
  uint16_t ldr;

  // calculate the average of the array and the minimal and maximum value
  average = 0;
  minimum = ~0;
  maximum = 0;

  for (int counter = nMeasures - 1; counter >= 0; counter--)
  {
    if (!ahead || counter != measureIndex)
      ldr = lightMeasures[counter];
    else
      ldr = analogRead(ldrPin);

    average += ldr;
    if (ldr > maximum)
      maximum = ldr;
    if (ldr < minimum)
      minimum = ldr;
  }

  average /= nMeasures;
}

void checkReset(bool open)
{
  unsigned long timeNow = millis();
  if (timeNow == 0)
    timeNow++;

  if (timePrevReset != 0 && 
      open != openPrevReset && 
      timeNow - timePrevReset < 5000)
  {
    isClosedByMotor = IsClosed();
    status = 0;
  }

  if (open != openPrevReset || timePrevReset == 0)
  {
    openPrevReset = open;
    timePrevReset = timeNow;
  }
}

// check if door must be opened or closed and do it if needed
int ProcessDoor(bool mayOpen)
{
  uint16_t average, minimum, maximum;
  int ret = 0;

  LightCalculation(average, minimum, maximum);
  setMQTTLDRavg((int)average);
  setMQTTTemperature();

  int ldr = analogRead(ldrPin);
  setMQTTLDR(ldr);
  if (ldr <= ldrCloseNow || keepClosed)
  {
    checkReset(false);
    
    average = 0;
    LightMeasurement(true); // fill the whole light measurement array with the current light value
  }
  else if (ldr >= ldrOpenNow || keepOpen)
  {
    checkReset(true);
    
    average = ldrOpenNow;
    LightMeasurement(true); // fill the whole light measurement array with the current light value
  }
  else
    timePrevReset = 0;
  
  bool isClosed = IsClosed();

#if defined ClockModule
  static byte hourOpened2 = 0;                // hour of last door open
  static byte minuteOpened2 = 0;              // minute of last door open
  static byte secondOpened2 = 0;              // second of last door open

  if (isClosed)
    hourOpened2 = minuteOpened2 = secondOpened2 = 0;
  else if (hourOpened2 == 0 && minuteOpened2 == 0 && secondOpened2 == 0)
    readDS3231time(&secondOpened2, &minuteOpened2, &hourOpened2, NULL, NULL, NULL, NULL);
#endif

  char *ptr = status == 0 ? isClosed ? "Door closed" : "Door open" : status == 1 ? "Door should be open but is still closed" : status == 2 ? "Door not closed after timeout" : "Door not closed after 10 tries to tighten";;

  if (status != 0)
  {
    char data[100];
    
    sprintf(data, "Something is not ok (%d - %s); idle", status, ptr);
    printSerialln(data);
#if defined ClockModule
    printSerial(" Time open: ");
    ShowTime(&hourOpened, &minuteOpened, &secondOpened);
    printSerialln();
    printSerial(" Time actually open: ");
    ShowTime(&hourOpened2, &minuteOpened2, &secondOpened2);
    printSerialln();
#endif    
  }

  else if (isClosed && !isClosedByMotor)
    // door is manually closed instead of automatically with the motor. Do nothing until manually opened again
    ;

  // If the average is less than ldrEvening and the door is not closed then close it
  else if (average <= ldrEvening && !isClosed)
  {
    Close();
    ret = motorClosePin;
  }

  // If the average is greater than ldrMorning and the door may open and it is closed and it may open by time then open it
  else if (average >= ldrMorning && isClosed && (average == ldrOpenNow || (mayOpen && MayOpen(0))))
  {
    Open();
    ret = motorOpenPin;
  }

  // Else if the minimum ldr value is smaller than ldrEvening, but the average isn't (yet) and the door is not closed (open) then it is about time to close it
  else if (minimum <= ldrEvening && !isClosed)
    ret = ledClosedPin;

  // Else if the maximum ldr value is greater than ldrMorning, but the average isn't (yet) and the door is closed and the time is later than a couple of minutes before may open then it is about time to open it
  else if (maximum >= ldrMorning && isClosed && MayOpen(-nMeasures / 2))
    ret = ledOpenedPin;

  if (keepOpen || keepClosed)
    ret = 0;
  else if (ret == motorOpenPin)
    ptr = "Door opening";
  else if (ret == motorClosePin)
    ptr = "Door closing";
  else if (ret == ledOpenedPin)
    ptr = "Door closed, about time to open it";
  else if (ret == ledClosedPin)
    ptr = "Door open, about time to close it";

  setMQTTDoorStatus(ptr);
  setMQTTTime();

  return ret;
}

// Check if the current time is later than the may open time - deltaMinutes
bool MayOpen(int deltaMinutes)
{
  byte hour, minute, second;
  GetTime(hour, minute, second);

  int startHourOpen1 = startHourOpen;
  int startMinuteOpen1 = startMinuteOpen + deltaMinutes;
  if (startMinuteOpen1 < 0)
  {
    startHourOpen1--;
    startMinuteOpen1 += 60;
  }

  return (hour > startHourOpen1 || (hour == startHourOpen1 && minute >= startMinuteOpen1));
}

void loop(void)
{
  loopEthernet();
  loopMQTT(false);

  unsigned long CurrentTime = millis();

#if defined NTPModule
  if (CurrentTime - PrevSyncTime >= SyncTime)
  {
    PrevSyncTime = CurrentTime;

    SyncDateTime();
  }
#endif

  if (CurrentTime - PrevTime >= 1000) // every second
  {
    PrevTime = CurrentTime;

#if defined NTPModule
    //printNTP();
#endif

    ProcessWater();    

    measureEverySecond--;

    int ret = ProcessDoor(false);

    if (status != 0)
    {
      // Oops something is wrong. Blink the red led as many times as the error code and then 2 seconds off and then start again
      if (++toggle > status * 2)
        toggle = 0;
      SetStatusLed(toggle % 2 != 0);
      digitalWrite(ledClosedPin, toggle == status * 2 ? HIGH : LOW);
      digitalWrite(ledOpenedPin, toggle % 2 == 0 ? LOW : HIGH);
    }

    else if (ret == ledClosedPin || ret == ledOpenedPin)
    {
      // The door is about to close or open
      // Blink the corresponding led every second
      if (++toggle > 1)
        toggle = 0;

      digitalWrite(ret, toggle == 0 ? LOW : HIGH);
      digitalWrite(ret == ledClosedPin ? ledOpenedPin : ledClosedPin, LOW);
    }

    else
      SetLEDOpenClosed();

    info(measureEverySecond, logit);

    Command(false);

    if (measureEverySecond == 0)
    {
      // every minute
      DSTCorrection();

      LightMeasurement(false);

      ProcessDoor(isClosedByMotor == IsClosed()); // If the door is manually closed then don't try to open

      measureEverySecond = measureEverySeconds;
    }
  }
}

void ProcessWater()
{
  if (digitalRead(emptyPin)) // empty open
  {
    // blink red
    blinkEmpty = !blinkEmpty;
    digitalWrite(ledEmptyPin, blinkEmpty ? HIGH : LOW);
    digitalWrite(ledNotEmptyPin, LOW);
    setMQTTWaterStatus("Empty");
  }
  else if (digitalRead(almostEmptyPin)) // almost empty open
  {
    // blink red/green
    blinkEmpty = !blinkEmpty;
    digitalWrite(ledEmptyPin, blinkEmpty ? HIGH : LOW);
    digitalWrite(ledNotEmptyPin, blinkEmpty ? LOW : HIGH);
    setMQTTWaterStatus("Low");
  }
  else
  {
    // green
    digitalWrite(ledNotEmptyPin, HIGH);
    digitalWrite(ledEmptyPin, LOW);
    setMQTTWaterStatus("Ok");
  }
#if 0
  printSerial("Almost empty: ");
  printSerialln(digitalRead(almostEmptyPin));

  printSerial("Empty: ");
  printSerialln(digitalRead(emptyPin));  
#endif  
}

void SetLEDOff()
{
  digitalWrite(ledOpenedPin, LOW);
  digitalWrite(ledClosedPin, LOW);
}

void SetLEDOpenClosed()
{
  if (IsClosed())
  {
    digitalWrite(ledOpenedPin, LOW);
    digitalWrite(ledClosedPin, HIGH);
  }
  else
  {
    digitalWrite(ledClosedPin, LOW);
    digitalWrite(ledOpenedPin, HIGH);
  }
}

void info(int measureEverySecond, char *buf)
{
  if (measureEverySecond >= 0)
    sprintf(buf + strlen(buf), "%d: ", measureEverySecond);

  uint16_t average, minimum, maximum;

  LightCalculation(average, minimum, maximum);
  sprintf(buf + strlen(buf), "LDR: %d, ~: %d", analogRead(ldrPin), average);

  LightCalculation(average, minimum, maximum, true);
  sprintf(buf + strlen(buf), ", ~~: %d, Door is %s (%s), Open LDR: %d, Close LDR: %d, Status: %d", average, IsClosed() ? "closed" : "open", isClosedByMotor ? "closed" : "open", ldrMorning, ldrEvening, status);

  if (hasClockModule)
  {
#if defined ClockModule
    byte second, minute, hour;
    // retrieve data from DS3231
    readDS3231time(&second, &minute, &hour, NULL, NULL, NULL, NULL);
    strcat(buf, ", Time now: ");
    ShowTime(&hour, &minute, &second, buf + strlen(buf));

    strcat(buf, ", Time open: ");
    ShowTime(&hourOpened, &minuteOpened, &secondOpened, buf + strlen(buf));

    strcat(buf, ", Time closed: ");
    ShowTime(&hourClosed, &minuteClosed, &secondClosed, buf + strlen(buf));
#endif
  }
#if !defined ClockModule    
  else
  {
    unsigned long timeNow = millis();

    strcat(buf, ", Time now: ");
    ShowTime(timeNow, timeNow, buf + strlen(buf));

    strcat(buf, ", Time open: ");
    ShowTime(msOpened, timeNow, buf + strlen(buf));

    strcat(buf, ", Time closed: ");
    ShowTime(msClosed, timeNow, buf + strlen(buf));
  }
#endif
}

void info(int measureEverySecond, bool dolog)
{
  if (dolog)
  {
    char buf[255] = { 0 };

    info(measureEverySecond, (char *) buf);

    printSerialln(buf);
    setMQTTMonitor(buf);
  }
}

#if defined ClockModule
void ShowTime(byte *hour, byte *minute, byte *second, char *buf)
{
  char *buffer[10];
  
  if (buf == NULL)
    buf = (char *)buffer;

  sprintf(buf, "%d:%02d", *hour, *minute);
  if (second != NULL)
    sprintf(buf + strlen(buf), ":%02d", *second);

  if (buf == buffer)
    printSerial(buf);
}

void ShowTime(byte *hour, byte *minute, byte *second)
{
  ShowTime(hour, minute, second, NULL);
}
#endif

void GetTime(byte &hour, byte &minute, byte &second)
{
  minute = 0;
  hour = 24;
  if (hasClockModule)
  {
#if defined ClockModule
    // retrieve data from DS3231
    readDS3231time(&second, &minute, &hour, NULL, NULL, NULL, NULL);
#endif
  }
#if !defined ClockModule  
  else if (msTime != 0)
  {
    unsigned long timeNow = millis();

    GetTime(timeNow, timeNow, hour, minute, second);
  }
#endif
}

#if !defined ClockModule
void GetTime(unsigned long time, unsigned long timeNow, byte &hour, byte &minute, byte &second)
{
  byte year, month, day;

  GetTime(time, timeNow, year, month, day, hour, minute, second);
}

void GetTime(unsigned long time, unsigned long timeNow, byte &year, byte &month, byte &day, byte &hour, byte &minute, byte &second)
{
  if (msTime != 0 && time >= msTime)
  {
    unsigned long ms = time - msTime;

    unsigned long days = ms / 1000 / 60 / 60 / 24;
    ms -= days * 24 * 60 * 60 * 1000;
    unsigned long hours = ms / 1000 / 60 / 60;
    ms -= hours * 60 * 60 * 1000;
    unsigned long minutes = ms / 1000 / 60;
    ms -= minutes * 60 * 1000;
    unsigned long seconds = ms / 1000;

    unsigned long D1 = dayTime + days;
    unsigned long M1 = monthTime;
    unsigned long Y1 = yearTime;
    unsigned long h1 = hourTime + hours;
    unsigned long m1 = minuteTime + minutes;
    unsigned long s1 = secondsTime + seconds;
    while (s1 >= 60)
    {
      m1++;
      s1 -= 60;
    }
    while (m1 >= 60)
    {
      h1++;
      m1 -= 60;
    }
    while (h1 >= 24)
    {
      h1 -= 24;
      D1++;
    }
    while (D1 > (unsigned long)daysInMonth(Y1, M1))
    {
      D1 -= daysInMonth(Y1, M1);
      M1++;
      if (M1 > 12)
      {
        Y1++;
        M1 = 1;
      }
    }

    year = (byte)Y1;
    month = (byte)M1;
    day = (byte)D1;
    hour = (byte)h1;
    minute = (byte)m1;
    second = (byte)s1;
  }
  else
    year = month = day = hour = minute = second = 0;
}

void ShowTime(unsigned long time, unsigned long timeNow, char *buf)
{
  char data[100];

  if (buf == NULL)
    buf = data;

  if (msTime == 0)
  {
    sprintf(buf, "%lu", time);

    if (timeNow > time)
    {
      unsigned long ms = timeNow - time;
      unsigned long days = ms / 1000 / 60 / 60 / 24;
      ms -= days * 24 * 60 * 60 * 1000;
      unsigned long hours = ms / 1000 / 60 / 60;
      ms -= hours * 60 * 60 * 1000;
      unsigned long minutes = ms / 1000 / 60;
      ms -= minutes * 60 * 1000;
      unsigned long seconds = ms / 1000;
      ms -= seconds * 1000;

      strcat(buf, " (");
      sprintf(buf + strlen(buf), "%dd%d:%02d:%02d:%03d", (int)days, (int)hours, (int)minutes, (int)seconds, (int)ms);
      strcat(buf, ")");
    }
  }
  else
  {
    *buf = 0;
    if (time >= msTime)
    {
      byte hour, minute, second;

      GetTime(time, timeNow, hour, minute, second);

      sprintf(buf + strlen(buf), "%02d:%02d:%02d", (int)hour, (int)minute, (int)second);
    }
    else
      strcat(buf, "-");
  }
  
  if (buf == data)
    printSerial(buf);
}

void ShowTime(unsigned long time, unsigned long timeNow)
{
  ShowTime(time, timeNow, NULL);
}
#endif

String WaitForInput(char *question)
{
  printSerialln(question);

  unsigned long StartTime = millis();
  unsigned long ElapsedTime = 0;
  while (!Serial.available()
# if defined SERIAL1
         && !Serial1.available()
# endif
         && ElapsedTime < waitForInputMaxMs)
  {
    // wait for input
    unsigned long CurrentTime = millis();
    ElapsedTime = CurrentTime - StartTime; // note that an overflow of millis() is not a problem. ElapsedTime will still be correct
  }

  return ElapsedTime < waitForInputMaxMs ?
          Serial.available() ?
          Serial.readStringUntil(10) :
# if defined SERIAL1          
          Serial1.readStringUntil(10)
# else
          ""
# endif
          : "";
}

int Command(bool start)
{
  if (logit)
    printSerialln("Waiting for command ");

  if (Serial.available()
# if defined SERIAL1
      || Serial1.available()
# endif
     )
  {
    String answer;

    answer = WaitForInput("");
    answer.toUpperCase();
    if (answer.substring(0, 5) == "START")
      return 2;

    Command(answer, true, start);

    return 1;
  }

  return 0;
}

void setChangeableValue(char *name, char *svalue)
{
  int value;
  bool found = false;

  printSerial("Setting value for variable ");
  printSerial(name);
  printSerial(" to ");
  printSerialln(svalue);

  for (int i = 0; i < sizeof(changeableData) / sizeof(*changeableData); i++)
  {
    if (strcasecmp(name, changeableData[i].name) == 0)
    {
      value = atoi(svalue);
      *(changeableData[i].variable) = value;

#if defined EEPROMModule
        writeChangeableData();
#endif      
      found = true;
      break;
    }
  }

  if (!found)
  {
    printSerialln("Variable not found in changeable data");
    setMQTTMonitor("Variable not found in changeable data");
  }
}

void swap(int &i1, int &i2)
{
  int i = i1;

  i1 = i2;
  i2 = i;
}

void Command(String answer, bool wait, bool start)
{
  answer.toUpperCase();

  printSerial("Received: ");
  printSerialln(answer.c_str());

  if (answer.substring(0, 2) == "AT") // current date/time arduino: dd/mm/yy hh:mm:ss
  {
#if !defined ClockModule
    if (answer[2])
    {      
      int day = answer.substring(2, 4).toInt();
      int month = answer.substring(5, 7).toInt();
      int year = answer.substring(8, 10).toInt();

      int hour = answer.substring(11, 13).toInt();
      int minute = answer.substring(14, 16).toInt();
      int sec = answer.substring(17, 19).toInt();
      if (day != 0 && month != 0 && year != 0)
      {
        msTime = millis();

        dayTime = day;
        monthTime = month;
        yearTime = year;
        hourTime = hour;
        minuteTime = minute;
        secondsTime = sec;
      }
    }

    byte second, minute, hour, dayOfMonth, month, year;
    char data[30];
    unsigned long timeNow = millis();
    
    GetTime(timeNow, timeNow, year, month, dayOfMonth, hour, minute, second);

    sprintf(data, "%02d/%02d/%02d %02d:%02d:%02d", (int)dayOfMonth,  (int)month, (int)year, (int)hour, (int)minute, (int)second);
    printSerialln(data);
    setMQTTMonitor(data);

    if (logit && wait)
      WaitForInput("Press enter to continue");
#endif
  }

  else if (answer.substring(0, 2) == "CT") // current date/time clock module: dd/mm/yy hh:mm:ss
  {
#if defined ClockModule
    if (answer[2])
    {
      int day = answer.substring(2, 4).toInt();
      int month = answer.substring(5, 7).toInt();
      int year = answer.substring(8, 10).toInt();

      int hour = answer.substring(11, 13).toInt();
      int minute = answer.substring(14, 16).toInt();
      int sec = answer.substring(17, 19).toInt();
      if (day != 0 && month != 0 && year != 0)
        setDS3231time(sec, minute, hour, 0, day, month, year);
    }

    printDS3231time();

    if (logit && wait)
      WaitForInput("Press enter to continue");
#endif
  }

  else if (answer.substring(0, 1) == "O") // open
  {
    Open();

    keepOpen = true;
    keepClosed = false;

    LightMeasurement(true);

    if (logit && wait)
      WaitForInput("Press enter to continue");
  }

  else if (answer.substring(0, 1) == "C") // close
  {
    Close();

    keepClosed = true;
    keepOpen = false;

    LightMeasurement(true);

    if (logit && wait)
      WaitForInput("Press enter to continue");
  }

  else if (answer.substring(0, 1) == "A") // auto
  {
    keepOpen = keepClosed = false;

    LightMeasurement(true); // fill the whole light measurement array with the current light value
    ProcessDoor(true); // opens the door if light and lets it closed if dark

    if (logit && wait)
      WaitForInput("Press enter to continue");
  }

  else if (answer.substring(0, 2) == "SL")
  {
    int ledClosedPinValue = digitalRead(ledClosedPin);
    int ledOpenedPinValue = digitalRead(ledOpenedPin);
    int ledNotEmptyPinValue = digitalRead(ledNotEmptyPin);
    int ledEmptyPinValue = digitalRead(ledEmptyPin);

    swap(ledClosedPin, ledNotEmptyPin);
    swap(ledOpenedPin, ledEmptyPin);

    digitalWrite(ledClosedPin, ledClosedPinValue);
    digitalWrite(ledOpenedPin, ledOpenedPinValue);
    digitalWrite(ledNotEmptyPin, ledNotEmptyPinValue);
    digitalWrite(ledEmptyPin, ledEmptyPinValue);

    if (ledClosedPin == ledClosedPinInit)
    {
      printSerialln("LEDs normal");
      setMQTTMonitor("LEDs normal");
    }
    else
    {
      printSerialln("LEDs switched");
      setMQTTMonitor("LEDs switched");
    }
    
    if (logit && wait)
      WaitForInput("Press enter to continue");
  }

#if defined NTPModule
  else if (answer.substring(0, 4) == "SYNC")
  {
    SyncDateTime();    
    
    if (logit && wait)
      WaitForInput("Press enter to continue");
  }
#endif

  else if (answer.substring(0, 4) == "LET ")
  {
    char *ptr, *value, *variable = answer.c_str() + 4;

    while (*variable == ' ')
      variable++;
    for (ptr = variable; *ptr && *ptr != '='; ptr++);    
    if (*ptr)
    {
      *ptr = 0;

      value = ptr + 1;

      while (--ptr >= variable && *ptr == ' ')
        *ptr = 0;

      while (*value == ' ')
        value++;
      ptr = value + strlen(value);
      while (--ptr >= value && *ptr == ' ')
        *ptr = 0;
      
      setChangeableValue(variable, value);
      showChangeableData(variable);
    }
  }

  else if (answer.substring(0, 3) == "GET")
  {
    char *ptr, *value, *variable;

    variable = answer.length() > 3 ? answer.c_str() + 4 : NULL;
    if (variable != NULL)
    {
      while (*variable == ' ')
        variable++;
      ptr = variable + strlen(variable);
      while (--ptr >= variable && *ptr == ' ')
        *ptr = 0;
    } 
    showChangeableData(variable);
  }

  else if (answer.substring(0, 1) == "L") // log toggle
  {
    logit = !logit;
    if (!logit)
      setMQTTMonitor("");
  }

  else if (answer.substring(0, 1) == "S") // reset status
  {
    status = answer.substring(1).toInt();
  }
  
  else if (answer.substring(0, 5) == "RESET") // reset
  {
    resetFunc(); //call reset
  }

  else if (answer.substring(0, 1) == "R") // repeat
  {
    int x = answer.substring(1).toInt();

    for (int i = 0; i < x; i++)
    {
      Close();

      delay(5000);

      Open();

      delay(5000);
    }
    if (logit && wait)
      WaitForInput("Press enter to continue");
  }

  else if (answer.substring(0, 1) == "T") // temperature
  {
#if defined ClockModule
    char buf[50];

    strcpy(buf, "Temperature: ");
    dtostrf(readTemperature(), 5, 2, buf + strlen(buf));
    strcat(buf, " °C");
    printSerialln(buf);
    setMQTTMonitor(buf);

    if (logit && wait)
      WaitForInput("Press enter to continue");
#endif
  }

#if defined EthernetModule
  else if (answer.substring(0, 2) == "IP")
  {
    printLocalIP();
    if (logit && wait)
      WaitForInput("Press enter to continue");
  }
  else if (answer.substring(0, 2) == "IS")
  {
    printEthernetStatus();

    if (logit && wait)
      WaitForInput("Press enter to continue");
  }
#endif

  else if (answer.substring(0, 1) == "I") // info
  {
    char buf[512] = { 0 };

    info(-1, (char *) buf);

    strcat(buf, "\r\n");

    strcat(buf, "Measurements:");
    int measureIndex0 = measureIndex;
    for (int counter = nMeasures - 1; counter >= 0; counter--)
    {
      measureIndex0 = (measureIndex0 > 0 ? measureIndex0 : nMeasures) - 1;
      sprintf(buf + strlen(buf), " %d", lightMeasures[measureIndex0]);
    }
    strcat(buf, "\r\n");

    uint16_t average, minimum, maximum;
    LightCalculation(average, minimum, maximum);

    sprintf(buf + strlen(buf), "Avg: %d, Min: %d, Max: %d", average, minimum, maximum);
    strcat(buf, "\r\n");

    sprintf(buf + strlen(buf), "@ Min: %d, Max: %d", ldrMinimum, ldrMaximum);
    
    printSerialln(buf);    
    
    setMQTTMonitor(buf);

    if (logit && wait)
      WaitForInput("Press enter to continue");
  }

  else if (answer.substring(0, 1) == "H") // help
  {
    printSerialln("O: Open door");
    printSerialln("C: Close door");
    printSerialln("A: Auto door");
    printSerialln("S(x): Reset status to x (default 0)");
    printSerialln("R<times>: Repeat opening and closing door");
    printSerialln("I: Info");
    printSerialln("L: Log toggle");
    printSerialln("SL: Switch LEDs");
#if !defined ClockModule
    printSerialln("AT<dd/mm/yy hh:mm:ss>: set arduino timer date/time");
#endif      
#if defined ClockModule
    printSerialln("CT<dd/mm/yy hh:mm:ss>: Set clockmodule date/time");
#endif
#if defined NTPModule
    printSerialln("SYNC: Synchronize date/time with NTP server");
#endif
#if defined ClockModule
    printSerialln("T: Temperature");
#endif
#if defined EthernetModule
    printSerialln("IP: Print IP address");
    printSerialln("IS: Show Ethernet status");
#endif
    printSerialln("RESET: Reset Arduino");
    printSerialln("LET name=value");
    printSerialln("GET {name}");
    if (start)
      printSerialln("START: Start loop");

    if (logit && wait)
      WaitForInput("Press enter to continue");
  }
}

bool isLeapYear(int year)
{
  return (((year % 4) == 0) && ((year % 100) != 0)) || ((year % 400) == 0);
}

int dayofweek(int year, int month, int day)
{
  static byte t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  year -= month < 3;
  return (year + year / 4 - year / 100 + year / 400 + t[month - 1] + day) % 7;
}

int daysInMonth(int year, int month)
{
  static byte days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  return days[month - 1] + (month == 2 && isLeapYear(year) ? 1 : 0);
}

void DSTCorrection()
{
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;

#if defined ClockModule
  if (hasClockModule)
    readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  else
#endif
  {
#if !defined ClockModule    
    unsigned long timeNow = millis();

    GetTime(timeNow, timeNow, year, month, dayOfMonth, hour, minute, second);
    dayOfWeek = dayofweek(2000 + year, month, dayOfMonth) + 1;
#endif    
  }

  if (dayOfWeek != dstDay)
    dstAdjust = true;
  else if (dstAdjust)
  {
    int dstDayWeek = 0;
    int adjustMinutes = 0;

    if (month == summerMonth && hour == summerHour)
    {
      dstDayWeek = summerDstDayWeek;
      adjustMinutes = +dstMinutes;
    }
    else if (month == winterMonth && hour == winterHour)
    {
      dstDayWeek = winterDstDayWeek;
      adjustMinutes = -dstMinutes;
    }

    if (adjustMinutes != 0)
    {
      int weekDays = 7;
      int lastDay = daysInMonth(year, month);
      int firstDay = summerDstDayWeek > 0 ? 1 : lastDay - weekDays + 1;
      lastDay = summerDstDayWeek > 0 ? weekDays : lastDay;

      if (dstDayWeek < 0)
      {
        dstDayWeek = -dstDayWeek;
        weekDays = -weekDays;
      }

      int day = ((int)dayOfMonth) - (dstDayWeek - 1) * weekDays;
      if (day >= firstDay && day <= lastDay)
      {
#if defined ClockModule
        if (hasClockModule)
        {
          readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
          minute += adjustMinutes % 60;
          while (minute >= 60)
          {
            hour++;
            minute -= 60;
          }
          hour += adjustMinutes / 60;
          setDS3231time(second, minute, hour, dayOfWeek, dayOfMonth, month, year);
        }
        else
#endif
        {
#if !defined ClockModule          
          minuteTime += adjustMinutes % 60;
          while (minuteTime >= 60)
          {
            hourTime++;
            minuteTime -= 60;
          }
          hourTime += adjustMinutes / 60;
#endif          
        }
        dstAdjust = false;
      }
    }
  }
}

#if defined ClockModule

#include <Wire.h>

#define DS3231_I2C_ADDRESS 0x68

byte decToBcd(byte val)
{ // Convert normal decimal numbers to binary coded decimal
  return ((val / 10 * 16) + (val % 10));
}

byte bcdToDec(byte val)
{ // Convert binary coded decimal to normal decimal numbers
  return ((val / 16 * 10) + (val % 16));
}

bool InitClock()
{
  Wire.begin();
  byte sec, minute, hour;
  readDS3231time(&sec, &minute, &hour, NULL, NULL, NULL, NULL);
  return sec < 60 && minute < 60 && hour < 24;
}

void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
{
  if (dayOfWeek == 0)
    dayOfWeek = dayofweek(2000 + year, month, dayOfMonth) + 1;
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0);                    // set next input to start at the seconds register
  Wire.write(decToBcd(second));     // set seconds
  Wire.write(decToBcd(minute));     // set minutes
  Wire.write(decToBcd(hour));       // set hours
  Wire.write(decToBcd(dayOfWeek));  // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(month));      // set month
  Wire.write(decToBcd(year));       // set year (0 to 99)
  Wire.endTransmission();
}

void readDS3231time(byte *second,
                    byte *minute,
                    byte *hour,
                    byte *dayOfWeek,
                    byte *dayOfMonth,
                    byte *month,
                    byte *year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  byte b;
  b = bcdToDec(Wire.read() & 0x7f);
  if (second != NULL)
    *second = b;
  b = bcdToDec(Wire.read());
  if (minute != NULL)
    *minute = b;
  b = bcdToDec(Wire.read() & 0x3f);
  if (hour != NULL)
    *hour = b;
  b = bcdToDec(Wire.read());
  if (dayOfWeek != NULL)
    *dayOfWeek = b;
  b = bcdToDec(Wire.read());
  if (dayOfMonth != NULL)
    *dayOfMonth = b;
  b = bcdToDec(Wire.read());
  if (month != NULL)
    *month = b;
  b = bcdToDec(Wire.read());
  if (year != NULL)
    *year = b;
}

void printDS3231time()
{
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  char data[30];
  
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);

  sprintf(data, "%02d/%02d/%02d %02d:%02d:%02d", (int)dayOfMonth,  (int)month, (int)year, (int)hour, (int)minute, (int)second);
  printSerialln(data);
}

float readTemperature()
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0x11); // register address for the temperature
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);       // get 2 bytes
  int MSB = Wire.read();                         // 2's complement int portion
  int LSB = Wire.read();                         // fraction portion
  float temperature = MSB & 0x7F;                // do 2's moth on MSB
  temperature = temperature + (LSB >> 6) * 0.25; // only care about bits 7 and 8

  return temperature;
}
#endif /* ClockModule */

#if defined EthernetModule

const unsigned long waitReconnectEthernet = 5L * 60L * 1000L; /* 5 minutes */
const int maxDurationEthernetConnect = 5000; /* 5 seconds */

bool hasEthernet = false;
unsigned long prevEthernetCheck = 0;

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
// Nano
#include <EthernetENC.h> // uses a bit less memory
//#include <UIPEthernet.h> // uses a bit more memory
#else
// Mega
#include <Ethernet.h> // does not work with an Arduino nano and its internet shield because it uses ENC28J60 which is a different instruction set
#endif

byte mac[] = { 0x54, 0x34, 0x41, 0x30, 0x30, 0x32 };

EthernetClient client;

#endif /* EthernetModule */

void setupEthernet()
{
#if defined EthernetModule  
  int ret;

  printSerialln("Start Ethernet");

  ret = Ethernet.begin(mac);

  hasEthernet = (ret == 1);
  
  if (!hasEthernet)
    prevEthernetCheck = millis();
  else
    prevEthernetCheck = 0;

  printSerial("Done Ethernet: ");
  printSerial(hasEthernet ? "OK" : "NOK: ");
  if (!hasEthernet)
    printSerial(ret);
  printSerialln();
  printSerial("IP: ");
  printLocalIP();

#endif
}

void loopEthernet()
{
#if defined EthernetModule

  unsigned long CurrentTime1 = millis();
  if (prevEthernetCheck == 0 /* If previous connection was ok */ ||
      CurrentTime1 - prevEthernetCheck > waitReconnectEthernet /* If waitReconnectEthernet time is passed, try again to connect */)
  {
//printSerialln("Ethernet.maintain start");
    Ethernet.maintain();
//printSerialln("Ethernet.maintain done");
    unsigned long CurrentTime2 = millis();

    if (CurrentTime2 - CurrentTime1 > maxDurationEthernetConnect) /* If connecting to ethernet takes more than maxDurationEthernetConnect time then there is probably no ethernet. In this case we will wait 5 minutes before trying again or else everything is slowed down too much */
    {
      hasEthernet = false;
      prevEthernetCheck = CurrentTime2;
      printSerialln("No Ethernet...");
    }
    else if (prevEthernetCheck != 0)
    {
      hasEthernet = true;
      prevEthernetCheck = 0;
      printSerialln("Ethernet restored");
    }
    else
      hasEthernet = true;
  }
#endif
}

void printLocalIP()
{
#if defined EthernetModule
  IPAddress ip = Ethernet.localIP();

  char sip[16];
  sprintf(sip, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

  printSerialln(sip);
  setMQTTMonitor(sip);
#endif
}

void printEthernetStatus()
{
#if defined EthernetModule  
  if (hasEthernet)
    printSerialln("Ethernet ok");
  else
  {
    printSerialln("Ethernet nok");

    unsigned long CurrentTime1 = millis();
    unsigned long nextCheck;
    if (prevEthernetCheck == 0 /* If previous connection was ok */ ||
        CurrentTime1 - prevEthernetCheck > waitReconnectEthernet /* If waitReconnectEthernet time is passed, try again to connect */)
      nextCheck = 0;
    else
      nextCheck = waitReconnectEthernet - (CurrentTime1 - prevEthernetCheck);
    
    char buf[50];
    sprintf(buf, "Next check in %d seconds", (int) (nextCheck / 1000));
    printSerialln(buf);
  }
#else
  printSerialln("No Ethernet connection");
#endif  
}

#if defined MQTTModule

// See https://github.com/dawidchyrzynski/arduino-home-assistant/
// See https://dawidchyrzynski.github.io/arduino-home-assistant/documents/getting-started/index.html

#include <ArduinoHA.h>

const unsigned long waitReconnectMQTT = 5L * 60L * 1000L; /* 5 minutes */
const int maxDurationMQTT = 900; /* 0.9 seconds */

unsigned long prevMQTTCheck = 0;
int cntMQTTCheck = 0;

#define BROKER_ADDR IPAddress(192,168,1,121)

HADevice device(mac, sizeof(mac));
HAMqtt mqtt(client, device, 15);

HASensor chickenguardDoorStatus("chickenguardDoorStatus");
HASensorNumber chickenguardLDR("ChickenguardLDR");
HASensorNumber chickenguardLDRavg("ChickenguardLDRAverage");
HASensorNumber chickenguardTemperature("ChickenguardTemperature", HASensorNumber::PrecisionP1);
HASensor chickenguardTimeNow("chickenguardTimeNow");
HASensor chickenguardTimeOpened("chickenguardTimeOpened");
HASensor chickenguardTimeClosed("chickenguardTimeClosed");
HASensor chickenguardMonitor("chickenguardMonitor");
HASensor chickenguardWaterStatus("chickenguardWaterStatus");

void setupMQTT()
{
    device.setName("Chickenguard");
    device.setSoftwareVersion("1.0.0");

    // for icons,
    // see https://pictogrammers.com/library/mdi/

    chickenguardDoorStatus.setIcon("mdi:door");
    chickenguardDoorStatus.setName("Door Status");

    chickenguardLDR.setIcon("mdi:flare");
    chickenguardLDR.setName("LDR");
    
    chickenguardLDRavg.setIcon("mdi:flare");
    chickenguardLDRavg.setName("LDR Average");

    chickenguardTemperature.setIcon("mdi:thermometer");
    chickenguardTemperature.setName("Temperature");
    chickenguardTemperature.setUnitOfMeasurement("°C"); // Also results in no logging in logbook HA

    chickenguardTimeNow.setIcon("mdi:clock-outline");
    chickenguardTimeNow.setName("Time Now");

    chickenguardTimeOpened.setIcon("mdi:clock-out");
    chickenguardTimeOpened.setName("Time Opened");

    chickenguardTimeClosed.setIcon("mdi:clock-in");
    chickenguardTimeClosed.setName("Time Closed");

    chickenguardMonitor.setIcon("mdi:monitor");
    chickenguardMonitor.setName("Monitor");

    chickenguardWaterStatus.setIcon("mdi:water-outline");
    chickenguardWaterStatus.setName("Water Status");

    printSerialln("Start MQTT");

    mqtt.onMessage(onMqttMessage);
    mqtt.onConnected(onMqttConnected);

    mqtt.begin(BROKER_ADDR);

    prevMQTTCheck = 0;
    cntMQTTCheck = 0;

    printSerialln("Done MQTT");
}

void onMqttMessage(const char* topic, const uint8_t* payload, uint16_t length) 
{
  if (strcmp(topic, "ChickenGuard/cmd") == 0)
  {
    String answer = "";
    for (int i = 0; i < length; i++)
      answer = answer + (char)payload[i];
    
    Command(answer, false, false);
  }
}

void onMqttConnected() 
{
    printSerialln("Connected to the broker!");

    mqtt.subscribe("ChickenGuard/#");
}

#endif /* MQTTModule */

void loopMQTT(bool force)
{
#if defined MQTTModule

  if (hasEthernet)
  {
    unsigned long CurrentTime1 = millis();

    if (force ||
        prevMQTTCheck == 0 /* If previous connection was ok */ ||
        CurrentTime1 - prevMQTTCheck > waitReconnectMQTT /* If waitReconnectMQTT time is passed, try again to connect */)
      cntMQTTCheck = 0;
    else if (++cntMQTTCheck >= 5) /* wait until there are 5 times delays */
      cntMQTTCheck = 0;

    if (cntMQTTCheck == 0)
    {
  //printSerialln("mqtt.loop start");
      mqtt.loop();
  //printSerialln("mqtt.loop done");
      unsigned long CurrentTime2 = millis();

      if (force)
        cntMQTTCheck = 0;
      else if (CurrentTime2 - CurrentTime1 > maxDurationMQTT) /* if mqtt.loop() took more than 0.9 sec then we assume that no MQTT connection could be established (1 second seems to be the timeout) */
      {
        prevMQTTCheck = CurrentTime2;
        printSerialln("No MQTT...");
      }
      else if (prevMQTTCheck != 0)
      {
        prevMQTTCheck = 0;
        cntMQTTCheck = 0;
        printSerialln("MQTT restored");
      }
  }
  }  
 
#endif
}

void setMQTTDoorStatus(char *msg)
{
#if defined MQTTModule
  chickenguardDoorStatus.setValue(msg);
#endif
}

void setMQTTLDR(int ldr)
{
#if defined MQTTModule
  chickenguardLDR.setValue(ldr);
#endif
}

void setMQTTLDRavg(int average)
{
#if defined MQTTModule
  chickenguardLDRavg.setValue(average);
#endif
}

void setMQTTTemperature()
{
#if defined MQTTModule && defined ClockModule  
  chickenguardTemperature.setValue(readTemperature());
#endif
}

void setMQTTTime()
{
#if defined MQTTModule
  char buf[10];
  byte second, minute, hour;

#if defined ClockModule
  // retrieve data from DS3231
  readDS3231time(&second, &minute, &hour, NULL, NULL, NULL, NULL);
  ShowTime(&hour, &minute, &second, buf);
#else
  unsigned long timeNow = millis();

  ShowTime(timeNow, timeNow, buf);
#endif
  chickenguardTimeNow.setValue(buf);

#if defined ClockModule
  if (hourOpened != 0 || minuteOpened != 0 || secondOpened != 0)
    ShowTime(&hourOpened, &minuteOpened, &secondOpened, buf);
  else
    strcpy(buf, "Unknown");
#else
  ShowTime(msOpened, timeNow, buf);
#endif    
  chickenguardTimeOpened.setValue(buf);

#if defined ClockModule
  if (hourClosed != 0 || minuteClosed != 0 || secondClosed != 0)
    ShowTime(&hourClosed, &minuteClosed, &secondClosed, buf);
  else
    strcpy(buf, "Unknown");
#else
  ShowTime(msClosed, timeNow, buf);
#endif    
  chickenguardTimeClosed.setValue(buf);  
#endif
}

void setMQTTMonitor(char *msg)
{
#if defined MQTTModule
  if (strlen(msg) > 255) // it looks like that a message may not be longer than 255 characters
    msg[255] = 0;
  chickenguardMonitor.setValue(msg);
#endif
}

void setMQTTWaterStatus(char *msg)
{
#if defined MQTTModule
  chickenguardWaterStatus.setValue(msg);
#endif
}

#if defined NTPModule

// See https://docs.arduino.cc/tutorials/ethernet-shield-rev2/udp-ntp-client /* NTP */
// See https://interface.fh-potsdam.de/prototyping-machines//hardware-prototypes/Boxnet/hardware_code/Write_boxes/DigitalClock_paul/ /* NTP */
// See https://forum.arduino.cc/t/what-is-the-best-library-for-a-mega-for-local-and-gmt-time/693976/4 /* set_zone */
// See https://forum.arduino.cc/t/time-isnt-converted-correct-from-unix-time/1060600 /* UNIX_OFFSET */

//#include <SPI.h>
#include <EthernetUdp.h>
#include <time.h>

unsigned int localPort = 8888;       // local port to listen for UDP packets

const char timeServer[] = "time.nist.gov"; // time.nist.gov NTP server

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

void InitUdp()
{
  Udp.begin(localPort);

  set_zone(+1 * ONE_HOUR); // GMT+1
}

// send an NTP request to the time server at the given address
void sendNTPpacket(const char * address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); // NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

struct tm *GetNTP()
{
  int size;

  while ((size = Udp.parsePacket()) > 0)
  {
    while (size > 0)
    {
      Udp.read(packetBuffer, min(size, NTP_PACKET_SIZE)); // discard any previously received packets
      size -= min(size, NTP_PACKET_SIZE);
    }
  }

  sendNTPpacket(timeServer); // send an NTP packet to a time server

  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) 
  {
    size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE)
    {
      // We've received a packet, read the data from it
      Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

      // the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, extract the two words:

      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;

      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
  
      // subtract seventy years:
      unsigned long epoch = secsSince1900 - seventyYears;

      unsigned long unixTime = epoch - UNIX_OFFSET;

      struct tm *time_info = localtime(&unixTime);

      return time_info;
    }
  }

  return NULL;
}

void printNTP()
{
  struct tm *time_info = GetNTP();
  if (time_info != NULL)
  {
    char buf[20];

    sprintf(buf, "Date: %02d/%02d/%04d", time_info->tm_mday, 1 + time_info->tm_mon, 1900 + time_info->tm_year);
    printSerialln(buf);

    sprintf(buf, "Time: %02d:%02d:%02d", time_info->tm_hour, time_info->tm_min, time_info->tm_sec);
    printSerialln(buf);
  }
  else
    printSerialln("No NTP packet");
}

void SyncDateTime()
{
  for (int i = 0; i < 60; i++)
  {
    struct tm *time_info = GetNTP();
    if (time_info != NULL)
    {
#if defined ClockModule  
      setDS3231time(time_info->tm_sec, time_info->tm_min, time_info->tm_hour, 0, time_info->tm_mday, 1 + time_info->tm_mon, 1900 + time_info->tm_year);
      printDS3231time();
#else
      msTime = millis();

      dayTime = time_info->tm_mday;
      monthTime = 1 + time_info->tm_mon;
      yearTime = 1900 + time_info->tm_year;
      hourTime = time_info->tm_hour;
      minuteTime = time_info->tm_min;
      secondsTime = time_info->tm_sec;

      ShowTime(msTime, msTime, NULL);
      printSerialln();
#endif

      break;
    }
  }
}

#endif /* NTPModule */

#if defined EEPROMModule

#include <EEPROM.h>

#define EEPROMMagic 25687

void readChangeableData()
{
  int address = 0;
  int value;

  EEPROM.get(address, value);
  if (value == EEPROMMagic)
  {
    printSerialln("Reading changeable data from EEPROM");
    for (int i = 0; i < sizeof(changeableData) / sizeof(*changeableData); i++)
    {
      address += sizeof(value);
      EEPROM.get(address, value);
      *(changeableData[i].variable) = value;
    }
  }
  else
    printSerialln("No valid changeable data stored in EEPROM");
}

void writeChangeableData()
{
  int address = 0;
  int value;

  printSerialln("Writing changeable data to EEPROM");

  value = EEPROMMagic;
  EEPROM.put(address, value);
  for (int i = 0; i < sizeof(changeableData) / sizeof(*changeableData); i++)
  {
    address += sizeof(value);
    value = *(changeableData[i].variable);
    EEPROM.put(address, value);
  }
}

#endif /* EEPROMModule */