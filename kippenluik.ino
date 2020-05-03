/*
  Kippenluik
  ground vanaf controller
  Vin vanaf 5 volt controller

  Zorg ervoor dat het luik *niet* gesloten is de eerste keer en het systeem stelt zich automatisch goed in.
  De as zal beginnen draaien in de ene of andere richting en uiteindelijk de draad oprollen zodat het luik sluit.
  Bij openen draait de as de andere richting uit en dus de draad weer afrollen en dus het luik openen.

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

int LuikOpenMS = 1300;            // aantal milliseconden om Luik open te laten gaan
int LuikSluitMS = 3000;           // maximaal aantal milliseconden om Luik te sluiten

// variabelen
int intMeetMoment = 0;            // positie in array met lichtmetingen
uint16_t Licht[LuikMetingen];     // array met de lichtmetingen


/*************************/
void setup(void) {
  /*************************/

  Serial.begin(9600);
  Serial.println("Kippenluik. Copyright Techniek & Dier aangepast door peno");

  if (!started) {
    started = true;
  
    pinMode(motortin1Pin, OUTPUT);
    pinMode(motortin2Pin, OUTPUT);
    pinMode(ldrPin, INPUT);           //ldr+plus en ldr+port+10k
    pinMode(magneetPin, INPUT_PULLUP);       //tussen 0 en de pin
    pinMode(LED_BUILTIN, OUTPUT);

    SetLed();
    
    // Eerst 60 seconden de kans geven om commando's te geven
    for (int i = 60; i > 0; i--)
    {
        info(i);      
  
        if (Command())
          i = 60; // Als er een commando is gegeven dan opnieuw 60 seconden wachten op een antwoord
  
        delay(1000);
    }
  
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
}

void MotorSluitLuik(void)
{
  digitalWrite(motortin1Pin, HIGH);
  digitalWrite(motortin2Pin, LOW);
}

/*************************/
void SluitLuik(void) {
  /*************************/

  Serial.println("Sluiten luik");

  if (!IsLuikGesloten()) {
    unsigned long StartTime = millis();
    int ElapsedTime = 0;
    while (!IsLuikGesloten() && ElapsedTime < LuikSluitMS) { // sluiten totdat magneetschakelaar zegt dat deur gesloten is maar ook max LuikSluitMS milliseconden (veiligheid)
      MotorSluitLuik();
      unsigned long CurrentTime = millis();
      ElapsedTime = CurrentTime - StartTime;
    }

    MotorUit();

    delay(500); // wacht een halve seconde

    MotorSluitLuik(); // sluit nog eens goed het luik. Dit kan omdat een elastiek wordt gebruikt en die dus ook nog een beet kan rekken

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
    
    info(intervalteller);

    Command();
  }

  ProcesLuik(false);
}

void info(int intervalteller) {
  if (intervalteller != 0) {  
    Serial.print(intervalteller);
    Serial.print(": ");
  }
  
  Serial.print("LDR waarde: ");
  Serial.print(analogRead(ldrPin));

  Serial.print(", Luikstatus: ");
  if (IsLuikGesloten())
    Serial.print("gesloten");
  else
    Serial.print("open");

  Serial.print(", Openen bij LDR waarde: ");
  Serial.print(LuikGoedemorgen);

  Serial.print(", Sluiten bij LDR waarde: ");
  Serial.print(LuikWelterusten);

  Serial.print(", Alles ok: ");
  Serial.println(ok);
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
      info(0);
      
      WachtOpInvoer("Druk enter om verder te gaan");
    }

    else if (OntvangenAntwoord.substring(0, 1) == "H") // help
    {
      Serial.println("T<ms>: milliseconden openen luik");
      Serial.println("O: Open luik");
      Serial.println("S: Sluit luik");
      Serial.println("R<aantal keer>: Herhaal openen en sluiten luik");
      Serial.println("I: Info");
      Serial.println("H: Deze help");
      
      WachtOpInvoer("Druk enter om verder te gaan");
    }

    return true;
  }

  return false;
}
