/*
  Kippenluik
  Zorg ervoor dat het luik *niet* gesloten is de eerste keer en het systeem stelt zich automatisch goed in.
  De as zal beginnen draaien in de ene of andere richting en uiteindelijk de draad oprollen zodat het luik sluit.
  Bij openen draait de as de andere richting uit en dus de draad weer afrollen en dus het luik openen.
  Bij starten wacht de arduino 1 minuut met normale werking om commando's in te kunnen geven.
  Ook eens de normale werking is gestart kunnen nog commando's gegeven worden.
  Merk op dat het default verbose is. Geef het L commando om info te tonen en terug L om weer verbose te gaan.
  Het H commando geeft een help scherm met alle mogelijke commando's.
  Merk op dat by default de arduino telkens reset als de monitor wordt gestart (Ctrl Shift M)
  Dan ben je ook alle variabelen kwijt :-(
  Dit kan opgelost worden door een condensator te plaatsen tussen de reset pin en gnd. Ik heb 0.47 µF genomen en dat werkt prima.
  Merk op dat die wel moet losgekoppeld zijn als er een nieuwe sketch moet geupload worden. Anders krijg je een fout dat het uploaden niet gelukt is.

  De arduino kan ook gekoppeld zijn met een klok module om de huidige tijd uit te lezen. Dit wordt gebruikt om ervoor te zorgen dat 's morgens het luik niet te vroeg open gaat.
*/

#define ClockModule

const int LuikGoedemorgen = 600; // lichtwaarde wanneer luik open
const int LuikWelterusten = 35;  // lichtwaarde wanneer luik dicht

const int LuikMetingen = 5;    // het gemiddelde van aantal metingen waarop beslist wordt
const int intervalwaarde = 60; // aantal seconden pause tussen metingen
const int motorClosePin = 4;   // controller in1  D4
const int motorOpenPin = 5;    // controller in2  D5
const int LEDClosedPin = 9;    // green LED - door closed  D9
const int LEDOpenPin = 7;      // red LED - door open D7
const int magneetPin = A1;     // magneetswitch   A1
const int ldrPin = A2;         // LDR             A2

const int LuikOpenMS = 1300;  // aantal milliseconden om Luik open te laten gaan
const int LuikSluitMS = 3000; // maximaal aantal milliseconden om Luik te sluiten

int status = 0; // alles ok
int toggle = 0;
bool logit = false;
bool IsLuikGeslotenMetMotor;

int intMeetMoment = 0;        // positie in array met lichtmetingen
uint16_t Licht[LuikMetingen]; // array met de lichtmetingen

unsigned long TimeLuikOpen = 0;
unsigned long TimeLuikGesloten = 0;

const byte StartHourLuikOpen = 7;    // minimum startuur dat luik open mag
const byte StartMinuteLuikOpen = 30; // minimum startmin dat luik open mag

bool HasClockModule = false;

#if defined ClockModule
byte HourLuikOpen = 0;
byte MinuteLuikOpen = 0;

byte HourLuikGesloten = 0;
byte MinuteLuikGesloten = 0;
#endif

int H = 0;
int M = 0;
unsigned long HMtime = 0;

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Kippenluik 10/06/2020. original Copyright Techniek & Dier aangepast door peno");

#if defined ClockModule
  HasClockModule = InitClock();
  if (HasClockModule)
    Serial.println("Clock module found");
  else
    Serial.println("Clock module not found");
#else
  Serial.println("Clock module not included");
#endif

  pinMode(motorClosePin, OUTPUT);
  pinMode(motorOpenPin, OUTPUT);
  pinMode(ldrPin, INPUT);            //ldr+plus en ldr+port+10k
  pinMode(magneetPin, INPUT_PULLUP); //tussen 0 en de pin
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(LEDOpenPin, OUTPUT);
  pinMode(LEDClosedPin, OUTPUT);
  digitalWrite(LEDOpenPin, LOW);
  digitalWrite(LEDClosedPin, LOW);

  SetStatusLed(false);

  logit = true;

  IsLuikGeslotenMetMotor = IsLuikGesloten();

  SetLEDOpenClosed();

  LichtMeting(true);

  // Eerst 60 seconden de kans geven om commando's te geven
  for (int i = 60; i > 0; i--)
  {
    info(i, logit);

    if (Command())
      i = 60; // Als er een commando is gegeven dan opnieuw 60 seconden wachten op een antwoord

    delay(1000);
  }

  logit = false;

  Serial.println("Start kippenluik");

  SluitLuik(); // Door de magneetschakelaar weten we hierna zeker dat het luik gesloten is

  LichtMeting(true);
  ProcesLuik(true); // opent het luik indien het licht is en laat het gesloten indien het donker is
}

void SetStatusLed(bool on)
{
  if (on)
    digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);
}

bool IsLuikGesloten()
{
  return digitalRead(magneetPin) == 0;
}

void MotorUit(void)
{
  digitalWrite(motorClosePin, LOW);
  digitalWrite(motorOpenPin, LOW);
}

void MotorOpenLuik(void)
{
  digitalWrite(motorClosePin, LOW);
  digitalWrite(motorOpenPin, HIGH);

  TimeLuikOpen = millis();
#if defined ClockModule
  if (HasClockModule)
    readDS3231time(NULL, &MinuteLuikOpen, &HourLuikOpen, NULL, NULL, NULL, NULL);
#endif
}

void MotorSluitLuik(void)
{
  digitalWrite(motorClosePin, HIGH);
  digitalWrite(motorOpenPin, LOW);

  TimeLuikGesloten = millis();
#if defined ClockModule
  if (HasClockModule)
    readDS3231time(NULL, &MinuteLuikGesloten, &HourLuikGesloten, NULL, NULL, NULL, NULL);
#endif
}

void SluitLuik(void)
{
  Serial.println("Sluiten luik");

  if (!IsLuikGesloten())
  {
    MotorSluitLuik();

    int ElapsedTime = 0;
    unsigned long StartTime = millis();
    while (!IsLuikGesloten() && ElapsedTime < LuikSluitMS)
    { // sluiten totdat magneetschakelaar zegt dat deur gesloten is maar ook max LuikSluitMS milliseconden (veiligheid)
      unsigned long CurrentTime = millis();
      if (CurrentTime < StartTime) // overflow veiligheid
        StartTime = CurrentTime;
      ElapsedTime = CurrentTime - StartTime;
    }

    MotorUit();

    if (ElapsedTime >= LuikSluitMS)
      status = 2;
    else
    {
      status = 3;

      delay(500); // wacht een halve seconde

      // sluit nog eens goed het luik. Dit kan omdat een elastiek wordt gebruikt en die dus ook nog een beetje kan rekken
      // soms moet die nog eens geprobeerd worden omdat de motor terugdraait. Max 10x
      for (int i = 0; i < 10 && status != 0; i++)
      {
        MotorSluitLuik(); // sluit nog eens goed het luik. Dit kan omdat een elastiek wordt gebruikt en die dus ook nog een beetje kan rekken

        delay(30); // voor maar 30 ms zodat deurtje goed aangespannen is

        MotorUit();

        delay(500); // wacht een halve seconde

        // controle luik gesloten
        if (IsLuikGesloten())
          status = 0;
        else
          Serial.println("Luik niet gesloten, nog eens proberen");
      }
    }

    digitalWrite(LEDOpenPin, LOW);
    digitalWrite(LEDClosedPin, LOW);

    if (status == 0)
    {
      Serial.println("Luik gesloten");
      SetLEDOpenClosed();
      IsLuikGeslotenMetMotor = true;
    }
    else
      Serial.println("Oops, luik *niet* gesloten");
  }
}

void OpenLuik(void)
{
  Serial.println("Openen luik");

  if (IsLuikGeslotenMetMotor)
  {
    if (IsLuikGesloten())
    {
      MotorOpenLuik();
      delay(LuikOpenMS); // Er is geen detectie op luik volledig open daarom wordt de motor vast LuikOpenMS milliseconden aangezet en dan zou het luik open moeten zijn
      MotorUit();
    }

    // controle luik open
    status = 1;
    for (int i = 0; i < 10 && status != 0; i++)
    {
      delay(100);
      if (!IsLuikGesloten())
        status = 0;
      else
        Serial.println("Luik niet open, nog eens controleren");
    }

    digitalWrite(LEDOpenPin, LOW);
    digitalWrite(LEDClosedPin, LOW);

    if (status == 0)
    {
      Serial.println("Luik open");
      SetLEDOpenClosed();
      IsLuikGeslotenMetMotor = false;
    }
    else
      Serial.println("Oops, luik *niet* open");
  }
}

void LichtMeting(bool init)
{
  int teller;

  // doe een lichtmeting
  if (init)
    teller = LuikMetingen;
  else
    teller = 1;
  for (; teller >= 1; teller--)
  {
    //haal lichtmeting op
    Licht[intMeetMoment] = analogRead(ldrPin); // gemeten waarde in de array zetten
    intMeetMoment++;                           // verhoog positie in de array
    if (intMeetMoment >= LuikMetingen)
      intMeetMoment = 0; // als array gevuld is dan weer vooraf beginnen met vullen
  }
}

void LichtBerekening(uint16_t &gemiddelde, uint16_t &minimum, uint16_t &maximum, uint16_t &donkerder, uint16_t &lichter)
{
  // gemiddelde van de array berekenen, minimale en maximale waarde bepalen en bepalen als het donkerder of lichter wordt
  gemiddelde = 0;
  minimum = LuikWelterusten + 1;
  maximum = 0;
  int intMeetMoment1 = intMeetMoment == 0 ? LuikMetingen - 1 : intMeetMoment - 1;
  uint16_t licht0 = Licht[intMeetMoment1]; // laatste lichtmeting
  donkerder = 0;
  lichter = 0;

  for (int teller = LuikMetingen - 1; teller >= 0; teller--)
  {
    gemiddelde += Licht[teller];
    if (Licht[teller] > maximum)
      maximum = Licht[teller];
    if (Licht[teller] < minimum)
      minimum = Licht[teller];
    intMeetMoment1 = intMeetMoment1 == 0 ? LuikMetingen - 1 : intMeetMoment1 - 1;
    if (teller > 0)
    {
      if (licht0 < Licht[intMeetMoment1])      // als de lichtmeting ervoor minder is dan is het donkerder geworden
        donkerder++;                           // aantal keren donkerder
      else if (licht0 > Licht[intMeetMoment1]) // als de lichtmeting ervoor meer is dan is het lichter geworden
        lichter++;                             // aantal keren lichter
      licht0 = Licht[intMeetMoment1];
    }
  }

  gemiddelde = gemiddelde / LuikMetingen;
}

int ProcesLuik(bool openen)
{
  uint16_t gemiddelde, minimum, maximum, donkerder, lichter;

  LichtBerekening(gemiddelde, minimum, maximum, donkerder, lichter);

  int ret = 0;

  bool isLuikGesloten = IsLuikGesloten();

  if (status != 0)
  {
    char data[100];

    sprintf(data, "Iets is niet ok (%d - %s); idle", status, status == 1 ? "luik niet open" : status == 2 ? "luik niet gesloten na timeout" : "luik niet gesloten na 10 keer proberen aanspannen");
    Serial.println(data);
  }

  // beslissen of luik open, dicht of blijven moet
  else if (gemiddelde <= LuikWelterusten && !isLuikGesloten)
  {
    SluitLuik();
    ret = motorClosePin;
  }

  else if (gemiddelde >= LuikGoedemorgen && openen && isLuikGesloten && MayOpen(0))
  {
    OpenLuik();
    ret = motorOpenPin;
  }

  else if (donkerder > lichter && minimum <= LuikWelterusten && !isLuikGesloten) // Het wordt donkerder en de minimum lichtmeting ligt onder de waarde om te sluiten => bijna tijd om te sluiten
    ret = LEDClosedPin;

  else if (lichter > donkerder && maximum >= LuikGoedemorgen && isLuikGesloten && MayOpen(-LuikMetingen / 2)) // Het wordt lichter en de maximum lichtmeting ligt boven de waarde om te openen => bijna tijd om te openen
    ret = LEDOpenPin;

  return ret;
}

bool MayOpen(int delta)
{
  byte hour, minute;
  GetTime(hour, minute);

  int StartHourLuikOpen1 = StartHourLuikOpen;
  int StartMinuteLuikOpen1 = StartMinuteLuikOpen + delta;
  if (StartMinuteLuikOpen1 < 0)
  {
    StartHourLuikOpen1--;
    StartMinuteLuikOpen1 += 60;
  }

  return (hour > StartHourLuikOpen1 || (hour == StartHourLuikOpen1 && minute >= StartMinuteLuikOpen1));
}

void loop(void)
{
  static int ret = 0;

  //taakafhandeling. Moet elke minuut
  for (int intervalteller = intervalwaarde; intervalteller > 0; intervalteller--)
  {
    delay(1000);

    ret = ProcesLuik(false); // 's nachts elke seconde controleren als luik nog steeds toe is. Indien niet dan laten sluiten

    if (status != 0)
    {
      // knipper rood zoveel keer als de foutcode
      if (++toggle > status * 2)
        toggle = 0;
      SetStatusLed(toggle % 2 != 0);
      digitalWrite(LEDClosedPin, LOW);
      digitalWrite(LEDOpenPin, toggle % 2 == 0 ? LOW : HIGH);
    }

    else if ((ret == LEDClosedPin || ret == LEDOpenPin) && IsLuikGeslotenMetMotor == IsLuikGesloten())
    {
      if (++toggle > 1)
        toggle = 0;

      digitalWrite(ret, toggle == 0 ? LOW : HIGH);
      digitalWrite(ret == LEDClosedPin ? LEDOpenPin : LEDClosedPin, LOW);
    }

    else    
      SetLEDOpenClosed();

    info(intervalteller, logit);

    Command();
  }

  LichtMeting(false);

  ProcesLuik(IsLuikGeslotenMetMotor == IsLuikGesloten()); // Indien het luik manueel is gesloten dan niet proberen openen.
}

void SetLEDOpenClosed()
{
  if (IsLuikGesloten())
  {
    digitalWrite(LEDOpenPin, LOW);
    digitalWrite(LEDClosedPin, HIGH);
  }
  else
  {
    digitalWrite(LEDClosedPin, LOW);
    digitalWrite(LEDOpenPin, HIGH);
  }
}

void info(int intervalteller, bool dolog)
{
  if (dolog)
  {
    if (intervalteller != 0)
    {
      Serial.print(intervalteller);
      Serial.print(": ");
    }

    Serial.print("LDR: ");
    Serial.print(analogRead(ldrPin));

    uint16_t gemiddelde, minimum, maximum, donkerder, lichter;

    LichtBerekening(gemiddelde, minimum, maximum, donkerder, lichter);
    Serial.print(", ~: ");
    Serial.print(gemiddelde);

    Serial.print(", Luik is ");
    if (IsLuikGesloten())
      Serial.print("toe");
    else
      Serial.print("open");

    if (IsLuikGeslotenMetMotor)
      Serial.print(" (toe)");
    else
      Serial.print(" (open)");

    Serial.print(", Open LDR: ");
    Serial.print(LuikGoedemorgen);

    Serial.print(", Toe LDR: ");
    Serial.print(LuikWelterusten);

    Serial.print(", Status: ");
    Serial.print(status);

    if (HasClockModule)
    {
#if defined ClockModule
      byte second, minute, hour;
      // retrieve data from DS3231
      readDS3231time(&second, &minute, &hour, NULL, NULL, NULL, NULL);
      // send it to the serial monitor
      Serial.print(", Tijd nu: ");
      ShowTime(&hour, &minute, &second);

      Serial.print(", Tijd open: ");
      ShowTime(&HourLuikOpen, &MinuteLuikOpen, NULL);

      Serial.print(", Tijd toe: ");
      ShowTime(&HourLuikGesloten, &MinuteLuikGesloten, NULL);
#endif
    }
    else
    {
      unsigned long TimeNow = millis();

      Serial.print(", Tijd nu: ");
      ShowTime(TimeNow, TimeNow);

      Serial.print(", Tijd open: ");
      ShowTime(TimeLuikOpen, TimeNow);

      Serial.print(", Tijd toe: ");
      ShowTime(TimeLuikGesloten, TimeNow);
    }

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
  if (HasClockModule)
  {
#if defined ClockModule
    // retrieve data from DS3231
    readDS3231time(NULL, &minute, &hour, NULL, NULL, NULL, NULL);
#endif
  }
  else if (HMtime != 0)
  {
    unsigned long TimeNow = millis();

    GetTime(TimeNow, TimeNow, &hour, &minute);
  }
}

void GetTime(unsigned long Time, unsigned long TimeNow, byte *h, byte *m)
{
  if (HMtime != 0 && Time >= HMtime)
  {
    unsigned long ms = Time - HMtime;

    unsigned long dagen = ms / 1000 / 60 / 60 / 24;
    ms -= dagen * 24 * 60 * 60 * 1000;
    unsigned long uren = ms / 1000 / 60 / 60;
    ms -= uren * 60 * 60 * 1000;
    unsigned long minuten = ms / 1000 / 60;

    unsigned long h1 = H + uren;
    unsigned long m1 = M + minuten;
    while (m1 >= 60)
    {
      h1++;
      m1 -= 60;
    }
    while (h1 >= 24)
      h1 -= 24;

    *h = (byte)h1;
    *m = (byte)m1;
  }
  else
    *h = *m = 0;
}

void ShowTime(unsigned long Time, unsigned long TimeNow)
{
  char data[100];

  if (HMtime == 0)
  {
    sprintf(data, "%lu", Time);
    Serial.print(data);

    if (TimeNow > Time)
    {
      unsigned long ms = TimeNow - Time;
      unsigned long dagen = ms / 1000 / 60 / 60 / 24;
      ms -= dagen * 24 * 60 * 60 * 1000;
      unsigned long uren = ms / 1000 / 60 / 60;
      ms -= uren * 60 * 60 * 1000;
      unsigned long minuten = ms / 1000 / 60;
      ms -= minuten * 60 * 1000;
      unsigned long seconden = ms / 1000;
      ms -= seconden * 1000;

      Serial.print(" (");
      sprintf(data, "%dd%d:%02d:%02d:%03d", (int)dagen, (int)uren, (int)minuten, (int)seconden, (int)ms);
      Serial.print(data);
      Serial.print(")");
    }
  }
  else
  {
    if (Time >= HMtime)
    {
      byte h, m;

      GetTime(Time, TimeNow, &h, &m);

      sprintf(data, "%02d:%02d", (int)h, (int)m);
      Serial.print(data);
    }
    else
      Serial.print("-");
  }
}

String WachtOpInvoer(String Vraag)
{
  Serial.println(Vraag);

  while (!Serial.available())
  {
    // wacht op input
  }

  return Serial.readStringUntil(10);
}

bool Command()
{
  if (logit)
    Serial.println("Wachten op commando ");

  if (Serial.available())
  {
    String OntvangenAntwoord;

    OntvangenAntwoord = WachtOpInvoer("");
    OntvangenAntwoord.toUpperCase();
    Serial.print("Ontvangen: ");
    Serial.println(OntvangenAntwoord);

    if (OntvangenAntwoord.substring(0, 1) == "L") // log toggle
    {
      logit = !logit;
    }

    else if (OntvangenAntwoord.substring(0, 1) == "O") // open
    {
      OpenLuik();

      if (logit)
        WachtOpInvoer("Druk enter om verder te gaan");
    }

    else if (OntvangenAntwoord.substring(0, 1) == "S") // sluit
    {
      SluitLuik();

      if (logit)
        WachtOpInvoer("Druk enter om verder te gaan");
    }

    else if (OntvangenAntwoord.substring(0, 1) == "R") // repeat
    {
      int x = OntvangenAntwoord.substring(1).toInt();

      for (int i = 0; i < x; i++)
      {
        SluitLuik();

        delay(5000);

        OpenLuik();

        delay(5000);
      }
      if (logit)
        WachtOpInvoer("Druk enter om verder te gaan");
    }

    else if (OntvangenAntwoord.substring(0, 1) == "C") // current time: hh:mm
    {
      int h = OntvangenAntwoord.substring(1, 3).toInt();
      int m = OntvangenAntwoord.substring(4, 6).toInt();
      if (h != 0 || m != 0)
      {
        H = h;
        M = m;
        HMtime = millis();
      }
    }

#if defined ClockModule
    else if (OntvangenAntwoord.substring(0, 1) == "D") // current date/time: dd/mm/yy hh:mm:ss
    {
      int dag = OntvangenAntwoord.substring(1, 3).toInt();
      int maand = OntvangenAntwoord.substring(4, 6).toInt();
      int jaar = OntvangenAntwoord.substring(7, 9).toInt();

      int uur = OntvangenAntwoord.substring(10, 12).toInt();
      int minute = OntvangenAntwoord.substring(13, 15).toInt();
      int sec = OntvangenAntwoord.substring(16, 18).toInt();
      if (dag != 0 && maand != 0 && jaar != 0)
        setDS3231time(sec, minute, uur, 0, dag, maand, jaar);
    }
#endif

#if defined ClockModule
    if (OntvangenAntwoord.substring(0, 1) == "T") // temperatuur
    {
      Serial.print("Temperatuur: ");
      Serial.print(readTemperature());
      Serial.println("C");

      if (logit)
        WachtOpInvoer("Druk enter om verder te gaan");
    }
#endif

    else if (OntvangenAntwoord.substring(0, 1) == "I") // info
    {
      info(0, true);

      if (logit)
        WachtOpInvoer("Druk enter om verder te gaan");
    }

    else if (OntvangenAntwoord.substring(0, 1) == "0")
    {
      digitalWrite(LEDOpenPin, LOW);
      digitalWrite(LEDClosedPin, LOW);
    }

    else if (OntvangenAntwoord.substring(0, 1) == "1")
    {
      digitalWrite(LEDOpenPin, HIGH);
      digitalWrite(LEDClosedPin, LOW);
    }

    else if (OntvangenAntwoord.substring(0, 1) == "2")
    {
      digitalWrite(LEDOpenPin, LOW);
      digitalWrite(LEDClosedPin, HIGH);
    }

    else if (OntvangenAntwoord.substring(0, 1) == "3")
    {
      digitalWrite(LEDOpenPin, HIGH);
      digitalWrite(LEDClosedPin, HIGH);
    }

    else if (OntvangenAntwoord.substring(0, 1) == "H") // help
    {
      Serial.println("O: Open luik");
      Serial.println("S: Sluit luik");
      Serial.println("R<aantal keer>: Herhaal openen en sluiten luik");
      Serial.println("0: Led uit");
      Serial.println("1: Led open aan");
      Serial.println("2: Led sluiten aan");
      Serial.println("3: Led open en sluiten aan");
      Serial.println("I: Info");
      Serial.println("L: Log toggle");
      Serial.println("C<hh:mm>: zet huidige tijd timer arduino");
#if defined ClockModule
      Serial.println("D<dd/mm/yy hh:mm:ss>: zet clockmodule huidige datum/tijd");
#endif
#if defined ClockModule
      Serial.println("T: Temperatuur");
#endif
      Serial.println("H: Deze help");

      if (logit)
        WachtOpInvoer("Druk enter om verder te gaan");
    }

    return true;
  }

  return false;
}

#if defined ClockModule

#include <Wire.h>

#define DS3231_I2C_ADDRESS 0x68 // Convert normal decimal numbers to binary coded decimal

byte decToBcd(byte val)
{
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
    dayOfWeek = dayofweek(dayOfMonth, month, 2000 + year) + 1;
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

int dayofweek(int d, int m, int y)
{
  static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  y -= m < 3;
  return (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
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
