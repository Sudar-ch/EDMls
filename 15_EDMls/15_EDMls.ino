/*  EDM Software by Sudar.ch
 *   
 *   
 *  Wann          Version   Was 
 *  25.11.2019              DebounceEncoder Library implementiert
 *  27.11.2019      1.5     Neue Interrupt Library -> Jeder Pin kann Interrupt sein
 *                          Encoder auf externen Interrupts / Taster auf PinChangeInterrupt
 *  29.11.2019      1.6     Debounce implementiert und Code sauber strukturiert              
 *  01.12.2019      1.7     Hellikeitstrim / Temperatur überwachung         
 *  13.12.2019      1.7.1   Installation Driver Enable
 *  15.12.2019      1.7.2   Pumpe angeschlossen
 *  16.12.2019      1.7.2   Hell/Dunkel forward geändert
 *  18.12.2019      1.8.1   Neuer Manuell Feed Mode. Hilfe neu entwickelt / Code cleanup
 *  23.12.2019      1.8.2   Bugfixes nach ersten Versuchen
 *  25.12.2019      1.8.3   Bugfix Max Speed über Poti /und umbau auf usek 
 *  02.01.2020      1.8.4   Bugfix Up/Down gedreht bei manuellem Mode
 *  15.01.2020      1.9.0   Code neu strukturiert
 *  
 *  28.01.2020      2.0.0   Im Automodus direkt die Rückzugzeit via Encoder einstellen während des Erodierens
 *  30.01.2020      2.0.1   Code beschleunigt und Bugs behoben Hilfe ergänzt                         
 *  01.03.202       2.0.2   Bug bei Tempanzeige behoben / Wartezeit auf 3ms max
 *                         
 *  
 *  
 *  
 *  Encoder to ground and +5V
 *  
 *  Encoder DATA Pin D2 Interrupt 0
 *  Encoder Clock Pin D3 Interrupt 1  
 *  Encoder PushButton Pin D4 ChangePin Interrupt 
 * 
 *  LED to Pin D5 
 *  Pump to Pin D6
 *  Driver Enable Pin D7
 *  Driver Dir Pin D8
 *  Driver Clock Pin D9
 *  
 *  Temp Sensor Pin A0 / NTC 10KOhm und 10KOhm Widerstand
 *  LDR pin A1 und GND / 10 KOhm zwischen +5V und Pin A1
 *  Poti for Speed to Pin A2 / GND / +5V
 *  Trim Poti für LDR Messung Hell/Dunkel Pin A3
 *  Display SDA A4
 *  Display SCL A5
 *  
 *  Schrittmotorentreiber Pinbelegung A4988
 *  MS1  MS2  MS3   Resolution
 *  L    L     L    Full step (2 phase)
 *  H    L     L    Half step
 *  L    H     L    Quarter step
 *  L    L     H    Eighth step
 *  L    H     H    Sixtinth step
 *  H    H     H    Sixtinth step
 *    
 *
 *
 * Debug with serial interface can be activated with define DEBUG
*/


//#define DEBUG    

#include <EnableInterrupt.h>
#include <DebouncedEncoder.h>
#include "HCMotor.h"
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#define MAXLINES 4    // defines the number of display lines
#define LCD_CHARACTERS 20

#define EncPinA 2 //Pin Encoder Interrupt
#define EncPinB 3 //Pin Encoder Interrupt
#define EncPushButton 4 // Ohne Interrupt
#define LEDPin 5 // LED für Hellikeitsabgleich
#define PumpPin 6 // Relay für Pumpe
#define DrvEnablePin 7 //Driver Enable Pin
#define DrvDirPin 8 //Connect to drive modules 'direction' input.
#define DrvClockPin 9 //Connect to drive modules 'step' or 'CLK' input.
#define NTCPin A0    // Temperatursensor
#define LDRPin A1 // LDR an 10kOhm Widerstand
#define PotiPin A2    // potentiometer für Geschwindigkeit
#define TrimPotiPin A3    // Trimpoti für Hellikeitsabgleich

#define Bounce_Duration 50 //Bounce Time in ms


String strVersion = "   Version 2.0.2"; //Software Relase Version

int lastPosition;
int encMinValue = -10;
int encMaxValue = 10;

int Pot_Value = 0;       // Potentiometer
int LDR_Value;  //LDR variable
int Speed;
int LDR_Dark = 364; // setzt den Wert für Dunkel. Kleinerer Wert: Motor fährt rückwärts

float ABSZERO = 273.15;
float MAXANALOGREAD = 1023.0;
float T0 = 25;    // Nenntemperatur des NTC-Widerstands in °C
float R0 = 10000; // Nennwiderstand des NTC-Sensors in Ohm
float B = 3977; // Materialkonstante B
float RV = 2000; // Vorwiderstand in Ohm  
float Temp; //finale Temperatur 
int NTC_Value;
int OverTemp = 45; //ab dieser Temp stopt der automatische Vorschub
int WaitTime =500 ; //0.5 Sek. Wartezeit bis der Rückzug gestartet wird //2500 ms Default Wert      //komisch damit wird er langsamer beim Rückzug......... der Wert 3 funktionierte gerade noch alles andere ist zu viel

volatile boolean turned;   // rotary was turned
volatile boolean buttonPressed;    // knob was pushed
volatile long bounceTime = 20;
unsigned long previousMillis = 0;


volatile boolean bPumpOnOff = false; // Pumpe ON oder OFF: Standardmässig OFF

int Seite = 0;

int CursorLine = 0;
int DisplayFirstLine = 0;
char* MenueLine[] = {"0 Man. Vorschub","1 Auto Vorschub","2 Pumpe Aus/Ein","3 Hilfe","4 \365ber"};
int MenueItems;

//****************************************************************************************************************
/* Create an instance of the library */

//Setup an encoder
DebouncedEncoder myEnc(EncPinA, EncPinB, encMaxValue, encMinValue, true);

//Setup Motor
HCMotor HCMotor;

//Setup Display
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

//****************************************************************************************************************
// Interrupt Service Routine for the EncPushButton
void ReadButton ()
{
    if(millis() > bounceTime)
    {
      if (!digitalRead (EncPushButton))
     
        buttonPressed = true;
        bounceTime = millis() + Bounce_Duration;
     }  
}
//****************************************************************************************************************
// Interrupt Service Routine for turning the Encoder
void ISREnc() 
{
  myEnc.encoderCheck();
  turned = true;
}
//****************************************************************************************************************
// Function -> Temperature calculation
float temperature_NTCB(float T0, float R0, float B, float RV, float VA_VB)
{
  T0+=ABSZERO;  // umwandeln Celsius in absolute Temperatur
  float RN=RV*VA_VB / (1-VA_VB); // aktueller Widerstand des NTC
  return T0 * B / (B + T0 * log(RN / R0))-ABSZERO;
}
//****************************************************************************************************************
void setup ()
{
  pinMode(DrvEnablePin,OUTPUT);
  pinMode(DrvDirPin, OUTPUT);            
  pinMode(DrvClockPin, OUTPUT);
  pinMode(LEDPin, OUTPUT);
  pinMode(PumpPin, OUTPUT);
  pinMode(EncPushButton, INPUT_PULLUP);
  pinMode(LDRPin, INPUT);
  pinMode(TrimPotiPin, INPUT);
  pinMode(NTCPin, INPUT);
 
  enableInterrupt(EncPinA, ISREnc, CHANGE); 
  enableInterrupt(EncPinB, ISREnc, CHANGE);
  enableInterrupt(EncPushButton, ReadButton, CHANGE); 
  
  HCMotor.Init();
 /* Attach motor 0 to digital pins 8 & 9. The first parameter specifies the motor number, the second is the motor type, and the third and forth are the digital pins that will control the motor */
  HCMotor.attach(0, STEPPER, DrvClockPin, DrvDirPin);
  HCMotor.Steps(0,CONTINUOUS);// Set the number of steps to continuous so the the motor is always turning

  digitalWrite(LEDPin, HIGH);
  digitalWrite(DrvEnablePin, HIGH); //Motortreiber ist deaktiviert bis er auf LOW gesetzt wird
  
  lcd.begin(20,4);
  lcd.backlight();

  lcd.setCursor(0,0);
  lcd.print(F("       EDMls"));
  lcd.setCursor(0,1);
  lcd.print(F("  (c) by sudar.ch"));
  lcd.setCursor(0,3);
  lcd.print(strVersion);
  delay(5000);

  MenueItems = sizeof(MenueLine)/sizeof(MenueLine[0]);
#ifdef DEBUG  
  Serial.begin (9600);
  Serial.println("Testing Menue!........");
  Serial.print("Number of Menue Items ");
  Serial.println(MenueItems);
#endif
  lcd.clear();
  lcd.cursor();
  lcd.blink();
  print_menue();
}
//****************************************************************************************************************
void loop ()
{
  if (buttonPressed)  //Pushbutton ist an Software Interrupt angeschlossen besser zuerst sonst hängt er!
  {
    buttonPressed = false;
    selection();
  }
  else if (turned)
  {
    int pos = myEnc.getPosition();
    int direc = myEnc.getDirection();
    if (lastPosition != pos) 
    {
      lastPosition = pos;
      if (direc == 1) 
      {
        #ifdef DEBUG
          Serial.println("CW");
        #endif
        move_down();
        turned = false;
      }
      else 
      {
        #ifdef DEBUG
          Serial.println("CCW");
        #endif
        move_up();
        turned = false;
      }
    }    
  }
  LDR_Dark = analogRead(TrimPotiPin); //Liest den aktuellen Trimpoti Wert aus und setz ihn als Dunkelschwelle
  LDR_Value = analogRead(LDRPin); //reads the LDR values
  if (LDR_Value > LDR_Dark)
  { 
    digitalWrite(LEDPin, LOW); 
  }
  else
  {
    digitalWrite(LEDPin, HIGH);
  }
  
 
  NTC_Value=analogRead(NTCPin);
  // Berechnen bei bekannter Materialkonstante B;
  Temp=temperature_NTCB(T0, R0, B, RV, NTC_Value/MAXANALOGREAD); 
}
//****************************************************************************************************************
void print_menue() {
  lcd.clear();
  for (int i=0;i<MAXLINES;i++) {
    lcd.setCursor(0,i);
    lcd.print(MenueLine[DisplayFirstLine + i]);
  }
  lcd.setCursor(0,(CursorLine-DisplayFirstLine));
}
//****************************************************************************************************************
void move_down() {
  if (CursorLine == (DisplayFirstLine+MAXLINES-1)) {
    DisplayFirstLine++;
  }
  if (CursorLine == (MenueItems-1)) {
    CursorLine = 0;
    DisplayFirstLine = 0;
  }
  else {
    CursorLine=CursorLine+1;
  }
  print_menue();
} 
//****************************************************************************************************************
void move_up() {
  if ((DisplayFirstLine == 0) & (CursorLine == 0)) {
    DisplayFirstLine = MenueItems-MAXLINES;   
  } 
  else if (DisplayFirstLine == CursorLine) {
    DisplayFirstLine--;
  }
  if (CursorLine == 0) {
    CursorLine = MenueItems-1;
  }
  else {
    CursorLine=CursorLine-1;
  }
  print_menue();
}

//****************************************************************************************************************
void selection() 
{
  // integrate here your calls to selected sub (eg. with switch/case)
  #ifdef DEBUG  
    Serial.print("Menueline ");
    Serial.print(CursorLine);
    Serial.println(" selected");
    Serial.print("..this is Menuetext ");
    Serial.println(MenueLine[CursorLine]);
  #endif
  switch (CursorLine)
  {
    case 0:
      Man_Feed();
      break;
    case 1:
      Auto_Feed();
      break;
    case 2:
      Pump_ON_OFF();
      break;
    case 3:
      Help();  
      break;
    case 4:
      About();  
      break;
    default:
      break;
  }     
  print_menue();  
}

//****************************************************************************************************************
void Man_Feed() 
{
  lcd.clear();
  lcd.noBlink();
  lcd.noCursor();
  lcd.setCursor(0,0);
  lcd.print(F("Manueller Vorschub"));
  lcd.setCursor(0,1);
  lcd.print(F("Encoder >> Richtung"));
  lcd.setCursor(0,2);
  lcd.print(F("Speed >> Speedpoti"));
  lcd.setCursor(0,3);
  lcd.print(F("      Z+ / Z-"));
  digitalWrite(LEDPin, LOW);
  digitalWrite(DrvEnablePin, LOW); //Motortreiber aktiviert
  //Pot_Value = analogRead(PotiPin);
  while (!buttonPressed) 
  {
    if (turned) 
      {

 /*       
       //halbautomatischer Vorschub sobald 1 click gedreht wird fährt er in diese Richtung mit Speed und weiterem click wird die Fahrtgschwindigkeit angepasst
        Pot_Value = analogRead(PotiPin);
        int pos = myEnc.getPosition();
        int direc = myEnc.getDirection();
        if (lastPosition != pos)  
        {
          lastPosition = pos;
          if (direc == 1) 
         {
          HCMotor.Direction(0, FORWARD);      
          Speed = map(Pot_Value,0 ,1023, 10, 1023); // 10 der untere Wert ist der MAX SPEED (wenn das Poti auf max ganz rechts steht)
          HCMotor.DutyCycle(0, Speed);
         }
        else
        {
         HCMotor.Direction(0, REVERSE);
         Speed = map(Pot_Value,0 ,1023, 10, 1023); // 10 der untere Wert ist der MAX SPEED (wenn das Poti auf max ganz rechts steht)
         HCMotor.DutyCycle(0, Speed); 
        }
 */ 
   //  /*          
         // Schritt um Schritt mit dem Rotary Encoder......
        int pos = myEnc.getPosition();
        int direc = myEnc.getDirection();
        if (lastPosition != pos) 
        Pot_Value = analogRead(PotiPin); 
        {
          lastPosition = pos;
          if (direc == 1) 
          {
            #ifdef DEBUG
              Serial.println("CW");
              Serial.println(pos);
             #endif
            Pot_Value = analogRead(PotiPin);
            int RPM = map(Pot_Value, 0, 1023, 1, 100000);
            digitalWrite(DrvDirPin,HIGH); //Vorwärts
            digitalWrite(DrvClockPin,HIGH);
            delayMicroseconds(RPM);  //delay auf delayMicroseconds
            digitalWrite(DrvClockPin,LOW);
            delayMicroseconds(RPM);
          }
           else 
          {
            #ifdef DEBUG
              Serial.println("CCW");
            #endif
            Pot_Value = analogRead(PotiPin);
            int RPM = map(Pot_Value, 0, 1023, 1, 100000);
            digitalWrite(DrvDirPin,LOW); //Rückwärts
            digitalWrite(DrvClockPin,HIGH);
            delayMicroseconds(RPM);
            digitalWrite(DrvClockPin,LOW);
            delayMicroseconds(RPM); 
          }
     //  */
        }       
        turned = false;
      }
   }
   buttonPressed = false;
   digitalWrite(DrvEnablePin, HIGH); //Motortreiber deaktiviert
   lcd.blink();
   lcd.cursor();
   print_menue();
}
//****************************************************************************************************************
void Auto_Feed() 
{
  lcd.clear();
  lcd.noBlink();
  lcd.noCursor();
  lcd.setCursor(0,0);
  lcd.print(F("Auto Vorschub"));
  lcd.setCursor(0,1);
  lcd.print(F("     Gestartet!"));
  lcd.setCursor(0,2);
  lcd.print(F("Wartezeit v. R\365ckzug"));
  lcd.setCursor(0,3);
  String strWaitTime;
  strWaitTime = WaitTime;
  strWaitTime = "       " + strWaitTime + "ms    ";
  lcd.print(strWaitTime);
  //lcd.setCursor(7,3);
  //lcd.print(WaitTime);
  //lcd.setCursor(11,3);
  //lcd.print(F("ms"));
   
  digitalWrite(DrvEnablePin, LOW); //Motortreiber aktiviert
  while (!buttonPressed) 
  {
    if (turned) 
    {
      int pos = myEnc.getPosition();
      int direc = myEnc.getDirection();
      if (lastPosition != pos) 
      {
        lastPosition = pos;
        if (direc == 1) 
        {
          #ifdef DEBUG
            Serial.println("CW");
            Serial.println(pos);
          #endif
          WaitTime++;
          if (WaitTime >=9999)
          {
            WaitTime = 9999;
          }
          /*
          lcd.setCursor(7,3);
          lcd.print(WaitTime);
          lcd.setCursor(11,3);
          lcd.print(F("ms")); */
          lcd.setCursor(0,3);
          String strWaitTime;
          strWaitTime = WaitTime;
          strWaitTime = "       " + strWaitTime + "ms    ";
          lcd.print(strWaitTime);
        }
        else 
        {
          #ifdef DEBUG
            Serial.println("CCW");
            Serial.println(pos);
          #endif
          WaitTime--;
          if (WaitTime <=0)
          {
            WaitTime = 0;
          }
            lcd.setCursor(0,3);
            String strWaitTime;
            strWaitTime = WaitTime;
            strWaitTime = "       " + strWaitTime + "ms    ";
            lcd.print(strWaitTime);         
          }
      }
      turned = false;
    }



    NTC_Value=analogRead(NTCPin); //reads the NTC values
    // Berechnen bei bekannter Materialkonstante B;
    Temp=temperature_NTCB(T0, R0, B, RV, NTC_Value/MAXANALOGREAD);
//    #ifdef DEBUG
//       Serial.print("Temp: "); Serial.println(Temp);
//    #endif
    
    LDR_Value = analogRead(LDRPin); //reads the LDR values
    Pot_Value = analogRead(PotiPin);    // read the value from the Potentiometer
//    #ifdef DEBUG  
//        Serial.print("LDR: "); Serial.println(LDR_Value);
      //  Serial.print("POT Value: "); Serial.println(Pot_Value);
//     #endif
    
    if(LDR_Value > LDR_Dark)   // Trimpoti mit einer grünen LED (Bedienung: nach dem Einschalten und ohne Funkenerosion muss die LED gerade nicht leuchten damit ist der Dunkelwert gesetzt. Sie geht an sobald Licht eintrifft....)
    {
      HCMotor.Direction(0, FORWARD);
      Speed = map(Pot_Value,0 ,1023, 10, 1023);
      HCMotor.DutyCycle(0, Speed);
      digitalWrite(LEDPin, LOW); 
    }
    else
    {
      // warte mal die Zeit ab und fahre solange nicht weiter
      HCMotor.DutyCycle(0, 0);
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= WaitTime) 
      {
        previousMillis = currentMillis;    
        HCMotor.Direction(0, REVERSE);
        Speed = map(Pot_Value,0 ,1023, 10, 1023);
        HCMotor.DutyCycle(0, Speed);
        digitalWrite(LEDPin, HIGH); 
      }
    }

    if(Temp > OverTemp)
    {
     lcd.setCursor(0,0);
     lcd.print(F(" Auto Vorschub STOP"));
     lcd.setCursor(0,1);
     lcd.print(F("                    "));
     lcd.setCursor(0,2);
     lcd.print(F(" Temp \365ber 45 Grad! "));
     lcd.setCursor(0,3);
     lcd.print(F("Aktuelle Temp: ")) & lcd.print(Temp);
     HCMotor.DutyCycle(0, 0); // da schaltet er nicht ab komisch...................................................
     break; //springt er damit aus der Schlaufe weil sonst schaltet der Motor nie ab
     //HCMotor.Direction(0, REVERSE);
     //HCMotor.Steps(0, 100); //100 Schritte retour
     //digitalWrite(DrvEnablePin, HIGH); //Motortreiber deaktiviert          evtl. besser
    }      
  }
  
   HCMotor.DutyCycle(0, 0);
   digitalWrite(DrvEnablePin, HIGH); //Motortreiber deaktiviert
   buttonPressed = false;
   lcd.blink();
   lcd.cursor();
   print_menue();
}
//****************************************************************************************************************
void Pump_ON_OFF() 
{
  lcd.clear();
  lcd.noBlink();
  lcd.noCursor();
  lcd.setCursor(0,0);
  lcd.print(F("Pumpe f\365r Fluid"));
    if (bPumpOnOff == false)
    {
      lcd.setCursor(0,2);
      lcd.print(F("        OFF"));
    }
    else
    {
      lcd.setCursor(0,2);
      lcd.print(F("        ON "));
     } 
 #ifdef DEBUG  
  Serial.println(bPumpOnOff);
 #endif     
 
  while (!buttonPressed) 
    {
    if (turned) 
      {
       if (bPumpOnOff == true)
         {
          bPumpOnOff = false;
          lcd.setCursor(0,2);
          lcd.print(F("        OFF ")); 
          digitalWrite(13,LOW); //Testhalber LED nachher Relay
          digitalWrite(PumpPin,LOW);
         }
       else
        {
         if (bPumpOnOff == false)
         {
          bPumpOnOff = true;
          lcd.setCursor(0,2);
          lcd.print(F("        ON ")); 
          digitalWrite(13,HIGH); //Testhalber LED nachher Relay
          digitalWrite(PumpPin,HIGH); //
         }
        }
        turned = false;
      }
    }
    
   buttonPressed = false;
   lcd.blink();
   lcd.cursor();
   print_menue();
}
//****************************************************************************************************************
void Help() 
{
  lcd.clear();
  lcd.noBlink();
  lcd.noCursor();
  HelpPage(0); //Zeigt die erste Seite an
  while (!buttonPressed) 
  {
    if (turned) 
      {
        int pos = myEnc.getPosition();
        int direc = myEnc.getDirection();
        if (lastPosition != pos) 
        {
          lastPosition = pos;
          if (direc == 1) 
          {
            #ifdef DEBUG
              Serial.println("CW");
              Serial.println(pos);
            #endif
            if (Seite >=7)
             {
              Seite = 7;
              HelpPage(Seite);              
             }
              else
              {
                Seite++;
                HelpPage(Seite);
              }
          }
            else 
          {
            #ifdef DEBUG
              Serial.println("CCW");
            #endif
            if (Seite <=0)
             {
              Seite = 0;
              HelpPage(Seite); 
             }
              else
             {
               Seite--;
               HelpPage(Seite);
             }         
          }           
        }       
        turned = false;
      }
   }
   buttonPressed = false;
   lcd.blink();
   lcd.cursor();
   print_menue();
}

//****************************************************************************************************************

void HelpPage(int Seite)
{
  if (Seite ==0)
  {
    lcd.setCursor(0,0);
    lcd.print(F("-->Vor jedem Start! "));
    lcd.setCursor(0,1);
    lcd.print(F("LED darf gerade     "));
    lcd.setCursor(0,2);
    lcd.print(F("nicht leuchten!     "));
    lcd.setCursor(0,3);
    lcd.print(F("So lange am Trimpoti"));
  }
  
  if (Seite ==1)
  {
    lcd.setCursor(0,0);
    lcd.print(F("drehen, bis die LED "));
    lcd.setCursor(0,1);
    lcd.print(F("gerade ausgeht!     "));
    lcd.setCursor(0,2);
    lcd.print(F("                    "));
    lcd.setCursor(0,3);
    lcd.print(F("                    "));
   }

  if (Seite ==2)
  {
    lcd.setCursor(0,0);
    lcd.print(F("0 -->Man. Vorschub  "));
    lcd.setCursor(0,1);
    lcd.print(F("Zum Positiononieren "));
    lcd.setCursor(0,2);
    lcd.print(F("der Elektrode mit   "));
    lcd.setCursor(0,3);
    lcd.print(F("dem Drehgeber. Vor- "));
  }
  
  if (Seite ==3)
  {
    lcd.setCursor(0,0);
    lcd.print(F("schub kann mit dem  "));
    lcd.setCursor(0,1);
    lcd.print(F("Speedpoti ein-      "));
    lcd.setCursor(0,2);
    lcd.print(F("gestellt werden.    "));
    lcd.setCursor(0,3);
    lcd.print(F("                    "));
  }
  
  if (Seite ==4)
  {
    lcd.setCursor(0,0);
    lcd.print(F("1 -->Auto Vorschub  "));
    lcd.setCursor(0,1);
    lcd.print(F("Autom. Prozess      "));
    lcd.setCursor(0,2);
    lcd.print(F("Vorschub kann mit   "));
    lcd.setCursor(0,3);
    lcd.print(F("dem Speedpoti ein-  "));
  }
 
  if (Seite ==5)
  {
    lcd.setCursor(0,0);
    lcd.print(F("gestellt werden.    "));
    lcd.setCursor(0,1);
    lcd.print(F("Der Prozess ist     "));
    lcd.setCursor(0,2);
    lcd.print(F("Temperatur \365ber-     "));
    lcd.setCursor(0,3);
    lcd.print(F("wacht. Max. 45 Grad "));
  }

  if (Seite ==6)
  {
    lcd.setCursor(0,0);
    lcd.print(F("Die R\365ckzugszeit    "));
    lcd.setCursor(0,1);
    lcd.print(F("kann mittels Encoder"));
    lcd.setCursor(0,2);
    lcd.print(F("angepasst werden.   "));
    lcd.setCursor(0,3);
    lcd.print(F("Default: 500ms      "));
  }

  if (Seite ==7)
  {
    lcd.setCursor(0,0);
    lcd.print(F("2 -->Pumpe          "));
    lcd.setCursor(0,1);
    lcd.print(F("Pumpe f\365r Fluid-    "));
    lcd.setCursor(0,2);
    lcd.print(F("f\357rderung.          "));
    lcd.setCursor(0,3);
    lcd.print(F("An- und Ausschaltbar"));
  }
}

//****************************************************************************************************************
void About() 
{
  lcd.clear();
  lcd.noBlink();
  lcd.noCursor();
  lcd.setCursor(0,0);
  lcd.print(F("       EDMls"));
  lcd.setCursor(0,1);
  lcd.print(F("  (c) by sudar.ch"));
  lcd.setCursor(0,3);
  lcd.print(strVersion);
  //lcd.print(F("   Version 1.7.3"));

  while (!buttonPressed) 
  {
  //drehen ist deaktiviert
    }
    
   buttonPressed = false;   
   lcd.blink();
   lcd.cursor();
   print_menue();
}
