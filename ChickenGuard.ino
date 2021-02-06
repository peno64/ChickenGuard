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

*/

#include <Arduino.h>

#define ClockModule                 // If defined then compile with the clock module code

const int ldrMorning = 600;         // light value for door to open
const int ldrEvening = 40;          // light value for door to close

const int ldrOpenNow = 1020;        // light value for door open now
const int ldrCloseNow = 0;          // light value for door close now

const int nMeasures = 5;            // number of measures to do on which an average is made to determine if door is opened (avg >= ldrMorning) or closed (avg <= ldrEvening)
const int measureEverySeconds = 60; // number of seconds between light measurements and descision if door should be closed or opened
const int motorClosePin = 4;        // motor turns one way to close the door
const int motorOpenPin = 5;         // motor turns other way to open the door
const int ledClosedPin = 9;         // green LED - door closed  D9
const int ledOpenedPin = 7;         // red LED - door open  D7
const int magnetPin = A1;           // magnet switch  A1
const int ldrPin = A2;              // LDR (light sensor) A2

const int openMilliseconds = 1200;  // number of milliseconds to open door
const int closeMilliseconds = 3000; // maximal number of milliseconds to close door
const int closeWaitTime1 = 1000;    // after this much milliseconds, stop closing door ...
const int closeWaitTime2 = 1000;    // ... for this much of milliseconds and then continue closing the door

const unsigned long waitForInputMaxMs = 900000; // maximum wait time for input (900000 ms = 15 min)

int status = 0;                     // status. 0 = all ok
int toggle = 0;                     // led blinking toggle
bool logit = false;                 // log to monitor
bool isClosedByMotor;               // is door closed by motor

uint16_t lightMeasures[nMeasures];  // array with light measurements (nMeasures measures taken every measureEverySeconds seconds). Contains the last nMeasures light measures
int measureIndex = 0;               // position in lightMeasures for next measurement.

uint16_t ldrMinimum = 9999;         // minimum ldr value
uint16_t ldrMaximum = 0;            // maximum ldr value

unsigned long msOpened = 0;         // millis() value of last door open
unsigned long msClosed = 0;         // millis() value of last door close

const byte startHourOpen = 7;       // minimum hour that door may open
const byte startMinuteOpen = 0;     // minimum minute that door may close

bool hasClockModule = false;        // Is the clock module detected

#if defined ClockModule
byte hourOpened = 0;                // hour of last door open
byte minuteOpened = 0;              // minute of last door open
byte secondOpened = 0;              // second of last door open

byte hourClosed = 0;                // hour of last door close
byte minuteClosed = 0;              // hour of last door close
#endif

#if !defined ClockModule
unsigned long msTime = 0;           // millis() value of set time
int dayTime = 0;                    // set day at msTime
int monthTime = 0;                  // set month at msTime
int yearTime = 0;                   // set year at msTime
int hourTime = 0;                   // set hour at msTime
int minuteTime = 0;                 // set minute at msTime
int secondsTime = 0;                // set seconds at msTime
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

// arduino function called when it starts or a reset is done
void setup(void)
{
  Serial.begin(9600);
  Serial.setTimeout(60000);
  Serial.println("Chicken hatch 17/01/2021. Copyright peno");

#if defined ClockModule
  hasClockModule = InitClock();
  if (hasClockModule)
    Serial.println("Clock module found");
  else
    Serial.println("Clock module not found");
#else
  Serial.println("Clock module not included");
#endif

  pinMode(motorClosePin, OUTPUT);
  pinMode(motorOpenPin, OUTPUT);
  pinMode(ldrPin, INPUT);
  pinMode(magnetPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(ledOpenedPin, OUTPUT);
  pinMode(ledClosedPin, OUTPUT);

  SetLEDOff();

  SetStatusLed(false);

  logit = true;

  isClosedByMotor = IsClosed();

  SetLEDOpenClosed();

  LightMeasurement(true); // fill the whole light measurement array with the current light value

  // First 60 seconds chance to enter commands
  for (int i = 60; i > 0; i--)
  {
    info(i, logit);

    if (Command())
      i = 60; // When a command was given then wait again 60 seconds

    delay(1000);
  }

  logit = false;

  Serial.println("Starting");

  Close(); // Via the magnet switch we know for sure that the door is closed hereafter

  LightMeasurement(true); // fill the whole light measurement array with the current light value

  Process(true); // opens the door if light and lets it closed if dark
}

void SetStatusLed(bool on)
{
  if (on)
    digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);
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
  digitalWrite(motorClosePin, LOW);
  digitalWrite(motorOpenPin, HIGH);

  msOpened = millis();
#if defined ClockModule
  if (hasClockModule)
    readDS3231time(&secondOpened, &minuteOpened, &hourOpened, NULL, NULL, NULL, NULL);
#endif
}

// run the motor in closing direction
void MotorClose(void)
{
  digitalWrite(motorClosePin, HIGH);
  digitalWrite(motorOpenPin, LOW);

  msClosed = millis();
#if defined ClockModule
  if (hasClockModule)
    readDS3231time(NULL, &minuteClosed, &hourClosed, NULL, NULL, NULL, NULL);
#endif
}

// close the door
void Close(void)
{
  Serial.println("Closing door");

  if (!IsClosed())
  {
    MotorClose();

    // run the motor until magnetic switch detects closed or maximum closeMilliseconds milliseconds (for safety if something goes wrong)
    // after closeWaitTime1 the motor stops for closeWaitTime2 milliseconds such that the door is again in rest because the elastic lets it vibrate
    int ElapsedTime = 0;
    unsigned long StartTime = millis();
    bool waited = false;
    while (!IsClosed() && ElapsedTime < closeMilliseconds)
    {
      unsigned long CurrentTime = millis();
      ElapsedTime = CurrentTime - StartTime; // note that an overflow of millis() is not a problem. ElapsedTime will still be correct
      if (!waited && ElapsedTime >= closeWaitTime1)
      {
        MotorOff();
        delay(closeWaitTime2);
        waited = true;
        StartTime += closeWaitTime2;
        MotorClose();
      }
    }

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
        MotorClose();

        delay(30); // close only for 30 milliseconds

        MotorOff();

        delay(500); // wait a half of a second

        // check if door is closed
        if (IsClosed())
          status = 0; // yes, all ok
        else
          Serial.println("Door not closed, try again");
      }
    }

    SetLEDOff();

    if (status == 0)
    {
      Serial.println("Door closed");
      SetLEDOpenClosed();
      isClosedByMotor = true;
    }
    else
      Serial.println("Oops, door *not* closed");
  }
}

// Open the door
void Open(void)
{
  Serial.println("Opening door");

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
        Serial.println("Door not open, check again");
    }

    SetLEDOff();

    if (status == 0)
    {
      Serial.println("Door open");
      SetLEDOpenClosed();
      isClosedByMotor = false;
    }
    else
      Serial.println("Oops, door *not* open");
  }
}

// Do a light measurement
void LightMeasurement(bool init)
{
  int counter;

  uint16_t ldr = analogRead(ldrPin);

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
void LightCalculation(uint16_t &average, uint16_t &minimum, uint16_t &maximum)
{
  // calculate the average of the array and the minimal and maximum value
  average = 0;
  minimum = ~0;
  maximum = 0;

  for (int counter = nMeasures - 1; counter >= 0; counter--)
  {
    average += lightMeasures[counter];
    if (lightMeasures[counter] > maximum)
      maximum = lightMeasures[counter];
    if (lightMeasures[counter] < minimum)
      minimum = lightMeasures[counter];
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
int Process(bool mayOpen)
{
  uint16_t average, minimum, maximum;

  LightCalculation(average, minimum, maximum);

  int ldr = analogRead(ldrPin);
  if (ldr <= ldrCloseNow)
  {
    checkReset(false);
    
    average = 0;
    LightMeasurement(true); // fill the whole light measurement array with the current light value
  }
  else if (ldr >= ldrOpenNow)
  {
    checkReset(true);
    
    average = ldrOpenNow;
    LightMeasurement(true); // fill the whole light measurement array with the current light value
  }
  else
    timePrevReset = 0;
  
  int ret = 0;

  bool isClosed = IsClosed();

  static byte hourOpened2 = 0;                // hour of last door open
  static byte minuteOpened2 = 0;              // minute of last door open
  static byte secondOpened2 = 0;              // second of last door open

  if (isClosed)
    hourOpened2 = minuteOpened2 = secondOpened2 = 0;
  else if (hourOpened2 == 0 && minuteOpened2 == 0 && secondOpened2 == 0)
    readDS3231time(&secondOpened2, &minuteOpened2, &hourOpened2, NULL, NULL, NULL, NULL);

  if (status != 0)
  {
    char data[100];

    sprintf(data, "Something is not ok (%d - %s); idle", status, status == 1 ? "door not open" : status == 2 ? "door not closed after timeout" : "door not closed after 10 tries to tighten");
    Serial.println(data);
    Serial.print(" Time open: ");
    ShowTime(&hourOpened, &minuteOpened, &secondOpened);
    Serial.println();
    Serial.print(" Time actually open: ");
    ShowTime(&hourOpened2, &minuteOpened2, &secondOpened2);
    Serial.println();
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

  return ret;
}

// Check if the current time is later than the may open time - deltaMinutes
bool MayOpen(int deltaMinutes)
{
  byte hour, minute;
  GetTime(hour, minute);

  int startHourOpen1 = startHourOpen;
  int startMinuteOpen1 = startMinuteOpen + deltaMinutes;
  if (startMinuteOpen1 < 0)
  {
    startHourOpen1--;
    startMinuteOpen1 += 60;
  }

  return (hour > startHourOpen1 || (hour == startHourOpen1 && minute >= startMinuteOpen1));
}

// arduino loop, execute every measureEverySeconds seconds
void loop(void)
{
  for (int measureEverySecond = measureEverySeconds; measureEverySecond > 0; measureEverySecond--)
  {
    // every second
    delay(1000);

    int ret = Process(false);

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

    Command();
  }

  DSTCorrection();

  LightMeasurement(false);

  Process(isClosedByMotor == IsClosed()); // If the door is manually closed then don't try to open
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

void info(int measureEverySecond, bool dolog)
{
  if (dolog)
  {
    if (measureEverySecond != 0)
    {
      Serial.print(measureEverySecond);
      Serial.print(": ");
    }

    Serial.print("LDR: ");
    Serial.print(analogRead(ldrPin));

    uint16_t average, minimum, maximum;

    LightCalculation(average, minimum, maximum);
    Serial.print(", ~: ");
    Serial.print(average);

    Serial.print(", Door is ");
    if (IsClosed())
      Serial.print("closed");
    else
      Serial.print("open");

    if (isClosedByMotor)
      Serial.print(" (closed)");
    else
      Serial.print(" (open)");

    Serial.print(", Open LDR: ");
    Serial.print(ldrMorning);

    Serial.print(", Close LDR: ");
    Serial.print(ldrEvening);

    Serial.print(", Status: ");
    Serial.print(status);

    if (hasClockModule)
    {
#if defined ClockModule
      byte second, minute, hour;
      // retrieve data from DS3231
      readDS3231time(&second, &minute, &hour, NULL, NULL, NULL, NULL);
      Serial.print(", Time now: ");
      ShowTime(&hour, &minute, &second);

      Serial.print(", Time open: ");
      ShowTime(&hourOpened, &minuteOpened, NULL);

      Serial.print(", Time closed: ");
      ShowTime(&hourClosed, &minuteClosed, NULL);
#endif
    }
#if !defined ClockModule    
    else
    {
      unsigned long timeNow = millis();

      Serial.print(", Time now: ");
      ShowTime(timeNow, timeNow);

      Serial.print(", Time open: ");
      ShowTime(msOpened, timeNow);

      Serial.print(", Time closed: ");
      ShowTime(msClosed, timeNow);
    }
#endif

    Serial.println();
  }
}

#if defined ClockModule
void ShowTime(byte *hour, byte *minute, byte *second)
{
  Serial.print(*hour, DEC);
  Serial.print(":");
  if (*minute < 10)
  {
    Serial.print("0");
  }
  Serial.print(*minute, DEC);
  if (second != NULL)
  {
    Serial.print(":");
    if (*second < 10)
    {
      Serial.print("0");
    }
    Serial.print(*second, DEC);
  }
}
#endif

void GetTime(byte &hour, byte &minute)
{
  minute = 0;
  hour = 24;
  if (hasClockModule)
  {
#if defined ClockModule
    // retrieve data from DS3231
    readDS3231time(NULL, &minute, &hour, NULL, NULL, NULL, NULL);
#endif
  }
#if !defined ClockModule  
  else if (msTime != 0)
  {
    unsigned long timeNow = millis();

    GetTime(timeNow, timeNow, hour, minute);
  }
#endif
}

#if !defined ClockModule
void GetTime(unsigned long time, unsigned long timeNow, byte &hour, byte &minute)
{
  byte year, month, day;

  GetTime(time, timeNow, year, month, day, hour, minute);
}

void GetTime(unsigned long time, unsigned long timeNow, byte &year, byte &month, byte &day, byte &hour, byte &minute)
{
  if (msTime != 0 && time >= msTime)
  {
    unsigned long ms = time - msTime;

    unsigned long days = ms / 1000 / 60 / 60 / 24;
    ms -= days * 24 * 60 * 60 * 1000;
    unsigned long hours = ms / 1000 / 60 / 60;
    ms -= hours * 60 * 60 * 1000;
    unsigned long minutes = ms / 1000 / 60;

    unsigned long D1 = dayTime + days;
    unsigned long M1 = monthTime;
    unsigned long Y1 = yearTime;
    unsigned long h1 = hourTime + hours;
    unsigned long m1 = minuteTime + minutes;
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
  }
  else
    year = month = day = hour = minute = 0;
}

void ShowTime(unsigned long time, unsigned long timeNow)
{
  char data[100];

  if (msTime == 0)
  {
    sprintf(data, "%lu", time);
    Serial.print(data);

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

      Serial.print(" (");
      sprintf(data, "%dd%d:%02d:%02d:%03d", (int)days, (int)hours, (int)minutes, (int)seconds, (int)ms);
      Serial.print(data);
      Serial.print(")");
    }
  }
  else
  {
    if (time >= msTime)
    {
      byte hour, minute;

      GetTime(time, timeNow, hour, minute);

      sprintf(data, "%02d:%02d", (int)hour, (int)minute);
      Serial.print(data);
    }
    else
      Serial.print("-");
  }
}
#endif

String WaitForInput(String question)
{
  Serial.println(question);

  unsigned long StartTime = millis();
  unsigned long ElapsedTime = 0;
  while (!Serial.available() && ElapsedTime < waitForInputMaxMs)
  {
    // wait for input
    unsigned long CurrentTime = millis();
    ElapsedTime = CurrentTime - StartTime; // note that an overflow of millis() is not a problem. ElapsedTime will still be correct
  }

  return ElapsedTime < waitForInputMaxMs ? Serial.readStringUntil(10) : "";
}

bool Command()
{
  if (logit)
    Serial.println("Waiting for command ");

  if (Serial.available())
  {
    String answer;

    answer = WaitForInput("");
    answer.toUpperCase();
    Serial.print("Received: ");
    Serial.println(answer);

    if (answer.substring(0, 1) == "L") // log toggle
    {
      logit = !logit;
    }

#if !defined ClockModule
    else if (answer.substring(0, 2) == "AT") // current date/time arduino: dd/mm/yy hh:mm:ss
    {
      int day = answer.substring(2, 4).toInt();
      int month = answer.substring(5, 7).toInt();
      int year = answer.substring(8, 10).toInt();

      int hour = answer.substring(11, 13).toInt();
      int minute = answer.substring(14, 16).toInt();
      int sec = answer.substring(17, 19).toInt();
      if (day != 0 && month != 0 && year != 0)
      {
        dayTime = day;
        monthTime = month;
        yearTime = year;
        hourTime = hour;
        minuteTime = minute;
        secondsTime = sec;
        msTime = millis();
      }
    }
#endif

#if defined ClockModule
    else if (answer.substring(0, 2) == "CT") // current date/time clock module: dd/mm/yy hh:mm:ss
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
#endif

    else if (answer.substring(0, 1) == "O") // open
    {
      Open();

      if (logit)
        WaitForInput("Press enter to continue");
    }

    else if (answer.substring(0, 1) == "C") // close
    {
      Close();

      if (logit)
        WaitForInput("Press enter to continue");
    }

    else if (answer.substring(0, 1) == "S") // reset status
    {
      status = answer.substring(1).toInt();
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
      if (logit)
        WaitForInput("Press enter to continue");
    }

#if defined ClockModule
    if (answer.substring(0, 1) == "T") // temperature
    {
      Serial.print("Temperature: ");
      Serial.print(readTemperature());
      Serial.println("°C");

      if (logit)
        WaitForInput("Press enter to continue");
    }
#endif

    else if (answer.substring(0, 1) == "I") // info
    {
      info(0, true);

      Serial.print("Measurements:");
      int measureIndex0 = measureIndex;
      for (int counter = nMeasures - 1; counter >= 0; counter--)
      {
        measureIndex0 = (measureIndex0 > 0 ? measureIndex0 : nMeasures) - 1;
        Serial.print(" ");
        Serial.print(lightMeasures[measureIndex0]);
      }
      Serial.println();

      uint16_t average, minimum, maximum;
      LightCalculation(average, minimum, maximum);

      Serial.print("Average: ");
      Serial.print(average);
      Serial.print(", Minimum: ");
      Serial.print(minimum);
      Serial.print(", Maximum: ");
      Serial.println(maximum);

      Serial.print("@ Minimum: ");
      Serial.print(ldrMinimum);
      Serial.print(", Maximum: ");
      Serial.println(ldrMaximum);
      
      if (logit)
        WaitForInput("Press enter to continue");
    }

    else if (answer.substring(0, 1) == "0")
    {
      SetLEDOff();
    }

    else if (answer.substring(0, 1) == "1")
    {
      digitalWrite(ledOpenedPin, HIGH);
      digitalWrite(ledClosedPin, LOW);
    }

    else if (answer.substring(0, 1) == "2")
    {
      digitalWrite(ledOpenedPin, LOW);
      digitalWrite(ledClosedPin, HIGH);
    }

    else if (answer.substring(0, 1) == "3")
    {
      digitalWrite(ledOpenedPin, HIGH);
      digitalWrite(ledClosedPin, HIGH);
    }

    else if (answer.substring(0, 1) == "H") // help
    {
      Serial.println("O: Open door");
      Serial.println("C: Close door");
      Serial.println("S(x): Reset status to x (default 0)");
      Serial.println("R<times>: Repeat openen and closing door");
      Serial.println("0: Leds off");
      Serial.println("1: Led open on");
      Serial.println("2: Led close on");
      Serial.println("3: Led open and closed on");
      Serial.println("I: Info");
      Serial.println("L: Log toggle");
#if !defined ClockModule
      Serial.println("AT<dd/mm/yy hh:mm:ss>: set arduino timer date/time");
#endif      
#if defined ClockModule
      Serial.println("CT<dd/mm/yy hh:mm:ss>: Set clockmodule date/time");
#endif
#if defined ClockModule
      Serial.println("T: Temperature");
#endif
      Serial.println("H: This help");

      if (logit)
        WaitForInput("Press enter to continue");
    }

    return true;
  }

  return false;
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

    GetTime(timeNow, timeNow, year, month, dayOfMonth, hour, minute);
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
#endif
