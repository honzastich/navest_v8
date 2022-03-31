/* v 1.8 */
/* zmena inicializace serv pri prvnim spusteni */
#include <ButtonDebounce.h> //https://github.com/maykon/ButtonDebounce/blob/master/examples/simple_btn/simple_btn.ino
#include <Servo.h>
#include <EEPROM.h>

class Flasher
{
  private:
    // Class Member Variables
    // These are initialized at startup
    int ledPin;      // the number of the LED pin
    long OnTime;     // milliseconds of on-time
    //long OffTime;    // milliseconds of off-time

    // These maintain the current state
    int ledState;                 // ledState used to set the LED
    unsigned long previousMillis;   // will store last time LED was updated

    // Constructor - creates a Flasher
    // and initializes the member variables and state
  public:
    //Flasher(int pin, long on, long off)
    Flasher(int pin, long on)
    {
      ledPin = pin;
      pinMode(ledPin, OUTPUT);

      OnTime = on;
      //OffTime = off;

      ledState = LOW;
      previousMillis = 0;
    }
    void LEDflash()
    {
      // check to see if it's time to change the state of the LED
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= OnTime)
      {
        ledState = !ledState;
        previousMillis = currentMillis;  // Remember the time
        digitalWrite(ledPin, ledState);  // Update the actual LED
      }
    }

};


//----------------------------------------------------------------------------------------------------------
#define btnPrg  4 // btn mode - prog, next7
#define btnUp 3 // tbn UP (semafor jde nahoru)
#define btnDown 2 //btnDown (semafor jde dolu)
#define ledMode 13 // LED na idkaci programovani (2 sec), next (1sec)
#define ledUp 5 // zelenaLED - semafor jde nahoru
#define ledDown 6 //cervena - semafor jde dolu
#define servo1 7 // horni rameno
#define servo2 8 // dolni rameno
#define servo3 9
#define setServoPosDelay 20 // rychost hybani serva pri nastavovani
#define setPrgModeDelay 2000 // ceka 2 sec do prepniti do programovaciho modu
#define setPrgModeDelaySwitch 50 // ceka 0.05 sec na prepnuti na dlasi stav v PRG modu
#define delayChngStavShort 500 // 0.5 sec od prepnuti
#define delayChngStavLong 4000 // 4 sec od prepnuti stavu
#define bit0 A3 // stůj / volno
#define bit1 A2 // přímo / odbočka
#define jmp1 10 // jumper 1 za jak dlouho bude VOLNO (preklemovano pada to se zpodenim)
#define jmp2 11 // jumper 2 za jak dlouho bude STUJ (preklemovano pada to se zpodenim)
#define jmp3 12 // jumper 3 - preklemovany - programovani jednoramenneho navestidla

/*#define A0 A0 // bit 0
  #define A1 A1 // bit 1 (tim se ridi horni rameno)
*/
/*
    INICIALIZACE SERVO POHONU
    00 - 0 stůj
    01 - 1 volno
    11 - 3 volno do odbočky (hýbou se obě ramena)
    10 - 2 - neplatny znak (dame tam 0-  - stuj)


  jmp1 a jmp2 jsou preklemovani - je to predvest
  jumper: opožděné stavění na volno), tak by měl na volno přestavovat pouze, když je kombinace bitů 0-1 (není postaveno do odbočky).

*/


bool modePrg = false; // jestli to je, nebo neni v programovanim modu

byte modePrgStep = 1; // Krok programovani -  co zrovna nastavuji

byte lastButtonState = LOW;
byte buttonState = LOW;
byte btnState = 0;
byte _servo_aktPos = 0;  // aktualni pozoce serva
byte _servo1_pos = 0;    // startovni pozice serva 1
byte _servo2_pos = 0;    // startovni pozice serva 2

byte _servo_limtPos11 = 0;  // poloha serva 1 - VOLNO   0
byte _servo_limtPos12 = 0;  // poloha serva 1 - STUJ    1
byte _servo_limtPos21 = 0;  // poloha serva 2 - VOLNO   2
byte _servo_limtPos22 = 0;  // poloha serva 2 - STUJ    3

unsigned long lastDebounceTime = 0;
unsigned long previousMillis = 0;

unsigned long currentMillis = 0;

unsigned int bitDebounceTime = 0; //   jak dlouho to ceka na zmenu stavu vstupu

bool predvest = false; // jestli to je(1), nebo neni(0) prevest

/* nastaveni blikani diod */
Flasher f_ledMode(ledMode, 200 );
Flasher f_ledUp(ledUp, 200);
Flasher f_ledDown(ledDown, 200);

Servo _servo1;
Servo _servo2;

ButtonDebounce _btnUp(btnUp, 50);       // jak dlouho drzim UP
ButtonDebounce _btnDown(btnDown, 50);   // jak dlouho drzim DOWN

void servoSetDefaultPosition(Servo myServo, byte _end)
{
  /*
     nastavuje kazde servo zvlast pri prepinani modu programovani
  */

  /* rozsvitim diody pri pohybu serva na vychozi pozici pri ladeni */
  digitalWrite(ledUp, HIGH);
  digitalWrite(ledDown, HIGH);
  int aktPos = myServo.read();
  int i = 0;
  if (aktPos < _end)
  {
    for (i = aktPos; i <= _end; i += 1)
    {
      myServo.write(i);
      delay(setServoPosDelay);
    }
  }
  else
  {
    for (i = aktPos; i >= _end; i -= 1)
    {
      myServo.write(i);
      delay(setServoPosDelay);
    }
  }
}

void servoSetPosition(bool prgStart, byte servoStatus)
{
  // nastavuje obe serva podle stausu (0 - STUJ, 1 - VOLNO , 3 - VOLNO 40)

  byte aktPos1;
  byte aktPos2;
  if (prgStart == true)
  {
    aktPos1 = (_servo_limtPos11 + _servo_limtPos12) / 2;
    aktPos2 = (_servo_limtPos21 + _servo_limtPos22) / 2;
    /* servo skoci do startovaci polohy a pocka*/
    //Serial.println("Inicializace serva 1");
    _servo1.write(aktPos1);
    _servo1.attach(servo1);
    delay(500);
    //Serial.println("Inicializace serva 2");
    _servo2.write(aktPos2);
    _servo2.attach(servo2);
    delay(1000);
    //Serial.println("konec inicializace");
  }
else
{
   _servo1.attach(servo1);
  _servo2.attach(servo2);
  aktPos1 = _servo1.read();
  aktPos2 = _servo2.read();
}
  byte endPos1 = 0;
  byte endPos2 = 0;
  bool runMove = true; // jestli se to ma vubec hnout
  switch (servoStatus)
  {
    case 0: // stuj
      endPos1 = _servo_limtPos12;
      endPos2 = _servo_limtPos22;
      break;

    case 1: // volno
      endPos1 = _servo_limtPos11;
      endPos2 = _servo_limtPos22;
      break;

    case 3: // volno 40 km
      endPos1 = _servo_limtPos11;
      endPos2 = _servo_limtPos21;
      break;

    default:
      runMove = false;
  }

  int i1 = 0;
  int i2 = 0;
  if (runMove == true)
  {
    if (aktPos1 < endPos1)
    {

      for (i1 = aktPos1; i1 <= endPos1; i1 += 1)
      {
        i2 = myMap(i1, aktPos1, endPos1, aktPos2, endPos2);
        _servo1.write(i1);
        _servo2.write(i2);
        delay(setServoPosDelay);
      }
    }
    else
    {
      for (i1 = aktPos1; i1 >= endPos1; i1 -= 1)
      {
        i2 = myMap(i1, aktPos1, endPos1, aktPos2, endPos2);
        _servo1.write(i1);
        _servo2.write(i2);
        delay(setServoPosDelay);
      }
    }

    _servo1.detach();
    _servo2.detach();
    /* inicializace serv pri spusteni */
  }
}

void navestidloIni(bool prgStart)
{
  _servo_limtPos11 = EEPROM.read(1); // volno
  _servo_limtPos12 = EEPROM.read(2); // stuj
  /* inicializace, kdyz v EEPROM jeste nic neni*/
  if (_servo_limtPos11 == _servo_limtPos12 && _servo_limtPos11 > 180)
  {
    _servo_limtPos11 = 93;
    _servo_limtPos12 = 93;
  }
  // servo 2
  _servo_limtPos21 = EEPROM.read(3); // volno
  _servo_limtPos22 = EEPROM.read(4); // stuj
  /* inicializace, kdyz v EEPROM jeste nic neni*/
  if (_servo_limtPos21 == _servo_limtPos22 && _servo_limtPos21 > 180)
  {
    _servo_limtPos21 = 93;
    _servo_limtPos22 = 93;
  }

  /*
    INICIALIZACE SERVO POHONU
    00 - 0 stůj
    01 - 1 volno
    11 - 3 volno do odbočky (hýbou se obě ramena)
    10 - 2 - neplatny znak (dame tam 0-  - stuj)
  */
  byte value = EEPROM.read(0);

  if (value > 3 || value == 2)
    value = 0;

  //value = 3;
Serial.println("Kam to jde: " + String(value));
  switch (value)
  {
    case 0: // stuj
      _servo1_pos = 0;
      _servo2_pos = 0;

      servoSetPosition(prgStart, 0);
      break;

    case 1: // volno
      _servo1_pos = 1;
      _servo2_pos = 0;

      servoSetPosition(prgStart, 1);
      break;

    case 3: // volno do odbocky
      _servo1_pos = 1;
      _servo2_pos = 1;

      servoSetPosition(prgStart, 3);
      break;
  }
}

long myMap(long x, long in_min, long in_max, long out_min, long out_max)
{
  /* vlkastni funkce MAP */
  long in_size = in_max - in_min;
  long out_size = out_max - out_min;
  if ( abs(in_size) > abs(out_size) )
  {
    if ( in_size < 0 ) in_size--; else in_size++;
    if ( out_size < 0 ) out_size--; else out_size++;
  }
  return (x - in_min) * (out_size) / (in_size) + out_min;
}

void setup() {
//Serial.begin(9600);

randomSeed(analogRead(A5)); 
int d=random(1, 11)*500;
delay(d);
//delay(1000);
//Serial.println("Inicializace");


  
  pinMode(btnPrg, INPUT_PULLUP);
  pinMode(btnUp, INPUT_PULLUP);
  pinMode(btnDown, INPUT_PULLUP);

  pinMode(bit0, INPUT_PULLUP);
  pinMode(bit1, INPUT_PULLUP);

  pinMode(ledMode, OUTPUT);
  pinMode(ledUp, OUTPUT);
  pinMode(ledDown, OUTPUT);

  digitalWrite(ledMode, LOW);
  digitalWrite(ledUp, LOW);
  digitalWrite(ledDown, LOW);

  //Serial.begin(9600);
  pinMode (jmp1, INPUT_PULLUP);
  pinMode (jmp2, INPUT_PULLUP);
  pinMode (jmp3, INPUT_PULLUP);

  navestidloIni(true); // nactu a nastavim posledni aktualni stav navestidel
}

byte lastBit0 = 0;
byte bit1State = 0;
byte bitValue = 0;

void loop() {

  setPrgMode();
  if (modePrg == true)
  {
    prgMode();
  }
  else
  {
    /*bit0 - nan hrni rameno*/
    //bitDebounceTime = delayVOLNO; // ze STUJ na volno to ceka 0.5 sc
    bitDebounceTime = delayChngStavShort; // ze STUJ na volno to ceka 4 sec
    if (!digitalRead(jmp1) == 1)     // kdyz je PIN 2 preklemovany, pada to hned
      bitDebounceTime = delayChngStavLong;

    if (bit1State != 0)
    {
      bitDebounceTime = delayChngStavShort; // z VOLNO na STUJ to ceka 4 sec
      if (!digitalRead(jmp2) == 1)     // kdyz je PIN 1 preklemovany, pada to hned
        bitDebounceTime = delayChngStavLong;
    }

    if (!digitalRead(jmp1) == 1 && !digitalRead(jmp3) == 1)
      predvest = true;

    byte _bit0 = !digitalRead(bit0);
    byte _bit1;
    if (lastBit0 != _bit0)
    {
      lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) >= bitDebounceTime)  //pockam 0.5 sed od zmeny stavu
    {
      if (bit1State != _bit0)
      {
        bit1State = _bit0;
        _bit1 = 0;
        if (_bit0 == 1)
        {
          _bit1 = !digitalRead(bit1); // nacitam jen pri bit0=1
        }

        if (_bit0 == 0)
          bitValue = 0;
        if (_bit0 == 1 && _bit1 == 0)
          bitValue = 1;
        if (_bit0 == 1 && _bit1 == 1)
          bitValue = 3;

        if (predvest == true && bitValue == 3)
          bitValue = 99;  //ignoruji stav

        servoSetPosition(false, bitValue);
      }
    }
    lastBit0 = _bit0;
  }
}

void prgMode()
{

  /* programovaci mod */
  switch (modePrgStep)
  {
    case 1: //horni navest - volno
      digitalWrite(ledUp, HIGH);
      digitalWrite(ledDown, LOW);
      break;

    case 2: //horni navest - stuj
      digitalWrite(ledUp, LOW);
      digitalWrite(ledDown, HIGH);
      break;

    case 3: //dolni navest - volno
      digitalWrite(ledDown, LOW);
      f_ledUp.LEDflash();
      break;

    case 4: //dolni navest - stuj
      digitalWrite(ledUp, LOW);
      f_ledDown.LEDflash();
      break;
  }



  _btnUp.update();
  _btnDown.update();
  if (!_btnUp.state() == HIGH && !_btnDown.state() == LOW) {
    currentMillis = millis();
    if ( currentMillis - previousMillis >= setServoPosDelay) {
      if (_servo_aktPos < 180)
      {
        _servo_aktPos += 1;
        switch (modePrgStep)
        {
          case 1: case 2:
            _servo1.write(_servo_aktPos);
            break;

          case 3: case 4:
            _servo2.write(_servo_aktPos);
            break;
        }

        previousMillis = currentMillis;
      }
    }
  }


  if (!_btnUp.state() == LOW && !_btnDown.state() == HIGH) {
    currentMillis = millis();
    if ( currentMillis - previousMillis >= setServoPosDelay) {
      if (_servo_aktPos > 0)
      {
        _servo_aktPos -= 1;
        switch (modePrgStep)
        {
          case 1: case 2:
            _servo1.write(_servo_aktPos);
            break;

          case 3: case 4:
            _servo2.write(_servo_aktPos);
            break;
        }
        previousMillis = currentMillis;
      }
    }
  }
  //   break;
  //}
}

void setPrgMode()
{
  /* prepnu se do programovaciho modu */
  byte _btnPrg = !digitalRead(btnPrg); // jdu proti nule
  int debounceDelay = setPrgModeDelay; // cekam 1 sec na prepnuti do programovaciho modu
  if (btnState == 1) // blikam kdyz drzim tl dele nez 2 sec)
  {
    f_ledMode.LEDflash();
  }
  if (modePrg == true)
  {
    f_ledMode.LEDflash();
    debounceDelay = setPrgModeDelaySwitch; // kdyz to je v prg. modu, na prepnuti uz staci jen 50 milisec. Stejne se pouziva i pro osetreni hrany
  }


  if (_btnPrg != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay && buttonState == LOW) {
    if (_btnPrg != buttonState) {
      buttonState = _btnPrg;
      if (buttonState == HIGH) {
        btnState = 1;
        //Serial.println(btnState);
      }
    }
  }

  if (_btnPrg == LOW && buttonState == HIGH)
  {
    btnState = 2;
    //Serial.println(btnState);
    /* prepnu / vypnu do programovaciho modu */
    if (modePrg == false)
    {
      modePrg = true;
      //Serial.println("Programovaci mod zapnuty");
      //Serial.println("Krok: " + String(modePrgStep, DEC) + " pozice serva: " + String(_servo_aktPos, DEC));

      _servo1.attach(servo1);   // pripojim servo 1
      servoSetDefaultPosition(_servo1, _servo_limtPos11); // servo j dam do polohy VOLNO
      _servo_aktPos = _servo1.read();
      btnState = 0;
    }
    else
    {
      modePrgStep += 1;
      //if (!digitalRead(jmp3) == 1 && modePrgStep == 3)// pri propojeni JMP 3 vyhodim programovani druheho serva
      //{
      //  modePrgStep = 5;
      //}
      if (modePrgStep < 5)
      {
        switch (modePrgStep)
        {
          case 2:
            _servo_limtPos11 = _servo_aktPos; //
            servoSetDefaultPosition(_servo1, _servo_limtPos12); // servo 1 dam do polohy STUJ
            _servo_aktPos = _servo1.read();
            break;

          case 3: // inicializuji si polohu pro druhe servo
            _servo_limtPos12 = _servo_aktPos; // nahraju posledni pozici serva 1
            if (!digitalRead(jmp3) == 1)// pri propojeni JMP 3 vyhodim programovani druheho serva
            {
              modePrgStep = 5;
              _servo1.detach();
            }
            else
            {
              servoSetDefaultPosition(_servo1, _servo_limtPos11 ); // servo 1 dam do polohy volno - je to kvuli programovani druheho ramene
              _servo1.detach();

              // prepnu na druhe servo
              _servo2.attach(servo2);
              servoSetDefaultPosition(_servo2, _servo_limtPos21); // servo 2 dam do polohy VOLNO
              _servo_aktPos = _servo2.read();
            }
            break;

          case 4:
            _servo_limtPos21 = _servo_aktPos; //
            servoSetDefaultPosition(_servo2, _servo_limtPos22); // servo j dam do polohy STUJ
            _servo_aktPos = _servo2.read();
            break;
        }
        //Serial.println("Krok: " + String(modePrgStep, DEC) + " pozice serva: " + String(_servo_aktPos, DEC));
      }
      else
      {
        _servo_limtPos22 = _servo_aktPos; //  ulozim si polohu STUJ pro 2. servo
        _servo2.detach();
        modePrg = false;

        EEPROM.update(1, _servo_limtPos11);
        EEPROM.update(2, _servo_limtPos12);
        EEPROM.update(3, _servo_limtPos21);
        EEPROM.update(4, _servo_limtPos22);
        /*
                Serial.println("Programovaci mod vypnuty");
                Serial.println("Poloha serva 1 VOLNO: " + String(_servo_limtPos11));
                Serial.println("Poloha serva 1 STUJ: " + String(_servo_limtPos12));
                Serial.println("Poloha serva 2 VOLNO: " + String(_servo_limtPos21));
                Serial.println("Poloha serva 2 STUJ: " + String(_servo_limtPos22));
        */
        digitalWrite(ledUp, LOW);
        digitalWrite(ledDown, LOW);
        digitalWrite(ledMode, LOW);
        navestidloIni(false); // znovu nainicializuji navestidlo

        modePrgStep = 1;
      }
    }
    buttonState = LOW;
  };
  lastButtonState = _btnPrg;
}
