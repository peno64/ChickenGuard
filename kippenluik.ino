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

*/

const int LuikGoedemorgen = 600;  // lichtwaarde wanneer luik open
const int LuikWelterusten = 35;   // lichtwaarde wanneer luik dicht

const int LuikMetingen = 5;       // het gemiddelde van aantal metingen waarop beslist wordt
const int intervalwaarde = 60;    // aantal seconden pause tussen metingen
const int motortin1Pin = 4;       // controller in1  D4
const int motortin2Pin = 5;       // controller in2  D5
const int magneetPin = A1;        // magneetswitch   A1
const int ldrPin = A2;            // LDR             A2

bool started = false;
bool ok = true;                   // alles ok
bool logit = false;

int LuikOpenMS = 1300;            // aantal milliseconden om Luik open te laten gaan
int LuikSluitMS = 3000;           // maximaal aantal milliseconden om Luik te sluiten

// variabelen
int intMeetMoment = 0;            // positie in array met lichtmetingen
uint16_t Licht[LuikMetingen];     // array met de lichtmetingen

unsigned long TimeLuikOpen = 0;
unsigned long TimeLuikGesloten = 0;

/*************************/
void setup(void) {
  /*************************/

  Serial.begin(9600);
  Serial.println("Kippenluik. original Copyright Techniek & Dier aangepast door peno");

  if (!started) {
    started = true;
  
    pinMode(motortin1Pin, OUTPUT);
    pinMode(motortin2Pin, OUTPUT);
    pinMode(ldrPin, INPUT);           //ldr+plus en ldr+port+10k
    pinMode(magneetPin, INPUT_PULLUP);       //tussen 0 en de pin
    pinMode(LED_BUILTIN, OUTPUT);

    SetLed();

    logit = true;
    
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
  
    SluitLuik(); // Door de magneetschakelaar weten hierna zeker dat het luik gesloten is
  
    ProcesLuik(true); // opent het luik indien het licht is en laat het gesloten indien het donker is
  }
}

void SetLed(void)
{
  if (ok)
    digitalWrite(LED_BUILTIN, LOW);
  else
    digitalWrite(LED_BUILTIN, HIGH);  
}

bool IsLuikGesloten() {
  return digitalRead(magneetPin) == 0;
}

void MotorUit(void)
{
  digitalWrite(motortin1Pin, LOW);
  digitalWrite(motortin2Pin, LOW);
}

void MotorOpenLuik(void)
{
  digitalWrite(motortin1Pin, LOW);
  digitalWrite(motortin2Pin, HIGH);  

  TimeLuikOpen = millis();
}

void MotorSluitLuik(void)
{
  digitalWrite(motortin1Pin, HIGH);
  digitalWrite(motortin2Pin, LOW);

  TimeLuikGesloten = millis();
}

/*************************/
void SluitLuik(void) {
  /*************************/

  Serial.println("Sluiten luik");

  if (!IsLuikGesloten()) {
    unsigned long StartTime = millis();
    int ElapsedTime = 0;
    bool FirstTime = true;
    while (!IsLuikGesloten() && ElapsedTime < LuikSluitMS) { // sluiten totdat magneetschakelaar zegt dat deur gesloten is maar ook max LuikSluitMS milliseconden (veiligheid)
      if (FirstTime)
      {
        FirstTime = false;
        MotorSluitLuik();
      }
      unsigned long CurrentTime = millis();
      if (CurrentTime < StartTime) // overflow veiligheid
        StartTime = CurrentTime;
      ElapsedTime = CurrentTime - StartTime;
    }

    MotorUit();

    delay(500); // wacht een halve seconde

    MotorSluitLuik(); // sluit nog eens goed het luik. Dit kan omdat een elastiek wordt gebruikt en die dus ook nog een beetje kan rekken

    delay(50); // voor maar 50 ms zodat deurtje goed aangespannen is

    MotorUit();

    // controle luik gesloten
    ok = false;
    for (int i = 0; i < 10 && !ok; i++) {
      delay(100);
      ok = IsLuikGesloten();
      if (!ok)
        Serial.println("Luik niet gesloten, nog eens controleren");
    }
    SetLed();
    if (ok)
      Serial.println("Luik gesloten");
    else
      Serial.println("Oops, luik *niet* gesloten");
  }
}

/*************************/
void OpenLuik(void) {
  /*************************/
  Serial.println("Openen luik");
  if (IsLuikGesloten()) {
    MotorOpenLuik();
    delay(LuikOpenMS); // Er is geen detectie op luik volledig open daarom wordt de motor vast LuikOpenMS milliseconden aangezet en dan zou het luik open moeten zijn
  }
  
  MotorUit();

  // controle luik open
  ok = false;
  for (int i = 0; i < 10 && !ok; i++) {
    delay(100);
    ok = !IsLuikGesloten();
    if (!ok)
      Serial.println("Luik niet open, nog eens controleren");
  }

  SetLed();
  if (ok)
    Serial.println("Luik open");
  else
    Serial.println("Oops, luik *niet* open");
}

/*************************/
void ProcesLuik(bool init) {
  /*************************/

  int teller;

  // doe een lichtmeting
  if (init)
    teller = LuikMetingen;
  else
    teller = 1;
  for (; teller >= 1; teller--) {
    //haal lichtmeting op
    Licht[intMeetMoment] = analogRead(ldrPin); // gemeten waarde in de array zetten
    intMeetMoment++;                           // verhoog positie in de array
    if (intMeetMoment >= LuikMetingen) intMeetMoment = 0; // als array gevuld is dan weer vooraf beginnen met vullen
  }

  ProcesLuikOpenSluit(true);
}

void ProcesLuikOpenSluit(bool openen)
{
  uint16_t gemiddelde;
  
  // gemiddelde van de array berekenen
  gemiddelde = 0;
  for (int teller = 0; teller < LuikMetingen; teller++)
    gemiddelde += Licht[teller];
  
  gemiddelde = gemiddelde / LuikMetingen;

  if (!ok)
    Serial.println("Iets is niet ok; idle");
    
  // beslissen of luik open, dicht of blijven moet
  else if (gemiddelde <= LuikWelterusten)
    SluitLuik();

  else if (openen && gemiddelde >= LuikGoedemorgen)
    OpenLuik();
}

/*************************/
void loop(void) {
  /*************************/

  //taakafhandeling. Moet elke minuut
  for (int intervalteller = intervalwaarde; intervalteller > 0; intervalteller--) {
    delay(1000);    

    ProcesLuikOpenSluit(false); // 's nachts elke seconde controleren als luik nog steeds toe is. Indien niet dan laten sluiten
    
    info(intervalteller, logit);

    Command();
  }

  ProcesLuik(false);
}

void info(int intervalteller, bool dolog) {
  char data[100];

  if (dolog) {
    if (intervalteller != 0) {  
      Serial.print(intervalteller);
      Serial.print(": ");
    }
    
    Serial.print("LDR waarde: ");
    Serial.print(analogRead(ldrPin));
  
    Serial.print(", Luikstatus: ");
    if (IsLuikGesloten())
      Serial.print("toe");
    else
      Serial.print("open");
  
    Serial.print(", Open LDR waarde: ");
    Serial.print(LuikGoedemorgen);
  
    Serial.print(", Toe LDR waarde: ");
    Serial.print(LuikWelterusten);
  
    Serial.print(", Alles ok: ");
    Serial.print(ok);
  
    unsigned long TimeNow = millis();
    
    Serial.print(", Tijd nu: ");
  
    sprintf(data, "%lu", TimeNow);
    Serial.print(data);
  
    Serial.print(", Tijd open: ");
    DiffTime(TimeLuikOpen, TimeNow);
  
    Serial.print(", Tijd toe: ");
    DiffTime(TimeLuikGesloten, TimeNow);
  
    Serial.println();
  }
}

void DiffTime(unsigned long Time, unsigned long TimeNow)
{
  char data[100];

  sprintf(data, "%lu", Time);
  Serial.print(data);

  if (TimeNow >= Time) {
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

String WachtOpInvoer(String Vraag) {
  Serial.println(Vraag);
 
  while(!Serial.available()) {
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

    if (OntvangenAntwoord.substring(0, 1) == "T")      // tijd in milliseconden om luik te openen
    {
      int x = OntvangenAntwoord.substring(1).toInt();

      if (x != 0)
      {
        LuikOpenMS = x;
        Serial.print("LuikOpenMS: ");
        Serial.println(LuikOpenMS);
      }
    }

    else if (OntvangenAntwoord.substring(0, 1) == "L") // log toggle
    {
      logit = !logit;
    }

    else if (OntvangenAntwoord.substring(0, 1) == "O") // open
    {
      OpenLuik();

      WachtOpInvoer("Druk enter om verder te gaan");
    }

    else if (OntvangenAntwoord.substring(0, 1) == "S") // sluit
    {
      SluitLuik();

      WachtOpInvoer("Druk enter om verder te gaan");
    }

    else if (OntvangenAntwoord.substring(0, 1) == "R") // repeat
    {
      int x = OntvangenAntwoord.substring(1).toInt();

      for (int i = 0; i < x; i++) {
        SluitLuik();
  
        delay(5000);
  
        OpenLuik();

        delay(5000);
      }
      WachtOpInvoer("Druk enter om verder te gaan");
    }

    else if (OntvangenAntwoord.substring(0, 1) == "I") // info
    {
      info(0, true);
      
      WachtOpInvoer("Druk enter om verder te gaan");
    }

    else if (OntvangenAntwoord.substring(0, 1) == "H") // help
    {
      Serial.println("T<ms>: milliseconden openen luik");
      Serial.println("O: Open luik");
      Serial.println("S: Sluit luik");
      Serial.println("R<aantal keer>: Herhaal openen en sluiten luik");
      Serial.println("I: Info");
      Serial.println("L: Log toggle");
      Serial.println("H: Deze help");
      
      WachtOpInvoer("Druk enter om verder te gaan");
    }

    return true;
  }

  return false;
}
