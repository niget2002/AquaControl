#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define LEDPIN 13
//Number of Items
#define numAnalogs 3
#define Channels_Count_Arduino 10
#define numFloats 5
#define numAlarms 4
#define startPin 2

//Analogs
#define PH0 A0
#define PH1 A1

//Floats
#define floatStart 15

// Tank  Range
#define tankTempLow 25  //in Celsius 77F
#define tankTempHigh  28  //in Celsius 82.4F 
#define tankPhLow 7.8
#define tankPhHigh 8.2  //not actually used
#define caPhLow 6.5
#define caPhHigh 6.7  //not actually used

//OneWire
#define ONE_WIRE_BUS 14

//Serial Comms
#define serialBuffer 8

//PumpOrder
#define pumpReturn 0
#define pumpRecirc 1
#define pumpCa 2
#define pumpTopOff 3

//Output Channel Mappings
int Ch[Channels_Count_Arduino + 1];
int myLights[] = { 0, 1 }; // Light1, Light2
int myPumps[] = { 2, 3, 4, 5 }; // return, recirc, CA, topoff
int myOther[] = { 6, 7, 8, 9 }; // heater, chiller, CO2, OM
//Arrays
//int incomingByte[serialBuffer];
char incomingByte[serialBuffer];
float analogs[numAnalogs];
int analogsTrigger = 0;
float analogsPin[] = { 0, 0 };
float phOffset[] = { 0.00, 0.00 };

int floatTrigger = 0;
int floatPin[numFloats];
int floatAverage[numFloats] = { 0, 0, 0, 0, 0 }; // Top-Off Low, Normal Sump, Sump Low, Tank High, Protein High
int alarms[numAlarms]; //  Top-Off Low, Sump Low, Tank High, Protein High

//Alarms
int alarmSet = 0;

// We don't need to trigger Alarms every iteration of the loop... every half second should be fine.
unsigned long previousMillis = 0;
const long interval = 1000;

//OneWire Setup for Temperature Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// **************************************************************************************************************************
void setup() {
  pinMode(LEDPIN, OUTPUT);
  Serial.begin(9600);                     // set up Serial at 9600 bps N81

  // initialize Timer1
  timerSetup();
  sensorSetup();
  digitalSetup();
  floatSetup();
  alarmSetup();

}

ISR(TIMER1_OVF_vect) {
  sensors.requestTemperatures();
}

void loop() {
  unsigned long currentMillis = millis();

  readPiData();
  floatCheck();
  analogGet();

  if ( currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    analogTemp();
    analogCheck();
    alarmCheck();
  }
}


//*************************************************************************
//  FUNCTIONS
//*************************************************************************

// ALARMS ***********************************************
void alarmSetup( void ) {
  for (int i = 0; i <= numAlarms; i++) {
    alarms[i] = 0;
  }
}

void alarmCheck( void ) {

  if ((floatAverage[1] == 1) && (floatAverage[0] == 0)) { // if normal sump is low, and top-off is high turn on topoff
    floatAlarm(1, myPumps[pumpTopOff]);
  } else {
    floatAlarm(0, myPumps[pumpTopOff]);
  }

  if (floatAverage[0] == 1) { // if top-off is low, turn off top-off
    floatAlarm(0, myPumps[pumpTopOff]);
    alarms[0] = 1;
  } else {
    alarms[0] = 0;
  }

  if (floatAverage[2] == 1) { // if Sump Low, turn off Return Pump
    floatAlarm(0, myPumps[pumpReturn]);
    alarms[1] = 1;
    alarmSet = 1;
    digitalWrite(LEDPIN, HIGH);
  } else {
    if (alarmSet == 0) {
      alarms[1] = 0;
      digitalWrite(LEDPIN, LOW);
    }
  }

  if (floatAverage[3] == 1) { // if Tank High, turn off Return Pump
    floatAlarm(0, myPumps[pumpReturn]);
    alarms[2] = 1;
    alarmSet = 1;
    digitalWrite(LEDPIN, HIGH);
  } else {
    if (alarmSet == 0) {
      alarms[2] = 0;
      digitalWrite(LEDPIN, LOW);
    }
  }

  if (floatAverage[4] == 1) { //Protein High
    alarms[3] = 1;
  } else {
    alarms[3] = 0;
  }
}


// ANALOGS **********************************************
void analogGet(void) {

  analogsTrigger++;
  analogsPin[0] += analogRead(PH0) * (5.0 / 1024);
  analogsPin[1] += analogRead(PH1) * (5.0 / 1024);

  if (analogsTrigger == 100) {
    analogs[0] = 3.5 * ((analogsPin[0] / 100)) + phOffset[0];
    analogs[1] = 3.5 * ((analogsPin[1] / 100)) + phOffset[1];
    analogsTrigger = 0;
    analogsPin[0] = 0;
    analogsPin[1] = 0;

  }


}

void analogTemp(void) {
  float val;
  val = sensors.getTempCByIndex(0);
  // Sanity check due to oddities with Temp Measurement
  if ( (10 < val) && (val < 50) ) {
    analogs[2] = val;
  }
}

void analogCheck(void) {
  // Turn Heater ON / OFF
  if ( tankTempLow > analogs[2] ) {
    ledMatrix_ON(myOther[0]);
  }
  else {
    ledMatrix_OFF(myOther[0]);
  }

  // Turn Chiller ON/ OFF
  if ( tankTempHigh < analogs[2] ) {
    ledMatrix_ON(myOther[1]);
  }
  else {
    ledMatrix_OFF(myOther[1]);
  }

  // Turn CO2 ON/ OFF
  if ( tankPhLow > analogs[0] || caPhLow > analogs[1]) {
    ledMatrix_OFF(myOther[2]);
  } else {
    ledMatrix_ON(myOther[2]);
  }

}

// DIGITALS *********************************************

void digitalSetup(void) {
  for (int i = 1; i <= Channels_Count_Arduino; i++) Ch[i] = startPin + i - 1 ;          // Starting pin assignment on Arduino incrementing up to total pin assigned
  for (int i = 1; i <= Channels_Count_Arduino; i++) {
    pinMode(Ch[i], OUTPUT); // declare all arduino pins as an OUTPUT
    ledMatrix_OFF(i - 1); // Set all outputs HIGH since Relays are active LOW
  }
}

void ledMatrix_ON(int index) {
  // Relays connected directly to Arduino
  digitalWrite(Ch[index + 1], LOW);

}

void ledMatrix_OFF(int index) {
  // Relays connected directly to Arduino
  digitalWrite(Ch[index + 1], HIGH);

}

// FLOATS *****************************************
void floatSetup(void) {
  for (int i = 0; i < numFloats; i++) {
    pinMode(floatStart + i, INPUT_PULLUP);
  }
}

void floatCheck(void) {
  //Check waterTopOff
  floatTrigger++;
  for (int i = 0; i < numFloats; i++) {
    floatPin[i] = floatPin[i] + digitalRead(floatStart + i);
  }
  if (floatTrigger == 100) {
    for (int i = 0; i < numFloats; i++) {
      floatAverage[i] = floatPin[i] / 100;
      floatPin[i] = 0;
    }
    floatTrigger = 0;
  }
}

void floatAlarm( int thisFloat, int thisPump ) {
  if (thisFloat == 1) {
    ledMatrix_ON(thisPump);
  }
  else {
    ledMatrix_OFF(thisPump);
  }

}

// ONEWIRE *****************************************
void sensorSetup(void) {
  sensors.begin();
  sensors.setWaitForConversion(false);
}

// SERIALS *****************************************

void readPiData() {
  int setChan = -1;
  byte i = 0;
  char probe;
  float val;
  //  if (Serial.available() >= serialBuffer ) {
  //    for (i = 0; i <= serialBuffer; i++) {
  //      incomingByte[i] = Serial.read(); // All the channels are read plus 1 for header starting with incomingByte[0]
  //    }

  if (Serial.available() > 0 ) {
    Serial.readBytesUntil('X', incomingByte, 8);


    int lookup = incomingByte[1] - '0';
    switch (incomingByte[0]) {
      case 'L':
        setChan = myLights[lookup];
        break;
      case 'P':
        setChan = myPumps[lookup];
        break;
      case 'O':
        setChan = myOther[lookup];
        break;
      case 'h':
        Serial.println(analogs[0]);
        break;
      case 'i':
        Serial.println(analogs[1]);
        break;
      case 't':
        Serial.println(analogs[2]);
        break;
      case 'R':
        lookup = incomingByte[2] - '0';
        switch (incomingByte[1]) {
          case 'L':
            Serial.println(digitalRead(Ch[myLights[lookup] + 1]));
            break;
          case 'P':
            Serial.println(digitalRead(Ch[myPumps[lookup] + 1]));
            break;
          case 'O':
            Serial.println(digitalRead(Ch[myOther[lookup] + 1]));
            break;
        }
        break;
      case 'A':
        lookup = incomingByte[2] - '0';
        switch (incomingByte[1]) {
          case 'R':
            alarmSet = 0;
            break;
          default:
            Serial.println(alarms[lookup]);
        }
        setChan = -1;
        break;
      case 'C':
        lookup = incomingByte[2] - '0';
        probe = incomingByte[1];
        val = 0.00;

        int decim;
        int tenths;
        int hundths;

        decim = incomingByte[3] - '0';
        tenths = incomingByte[4] - '0';
        hundths = incomingByte[5] - '0';
        val = decim + ((tenths * 10 + hundths) * .01);

        switch (probe) {
          case '0':
            phOffset[0] = val;
            break;
          case '1':
            phOffset[1] = val;
            break;
        }
        setChan = -1;
        break;

      default:
        setChan = -1;
        break;
    }
    if (setChan != -1) {
      switch (incomingByte[2]) {
        // Relays are active LOW, so ON = OFF and vice versa
        case '0':
          ledMatrix_OFF(setChan);
          break;
        case '1':
          ledMatrix_ON(setChan);
          break;
      }
    }
  }
}

// TIMER *****************************************************
void timerSetup(void) {
  cli();         // disable global interrupts
  TCCR1A = 0;    // set entire TCCR1A register to 0
  TCCR1B = 0;    // set entire TCCR1B register to 0
  // (as we do not know the initial  values)

  // enable Timer1 overflow interrupt:
  TIMSK1 |= (1 << TOIE1); //Atmega8 has no TIMSK1 but a TIMSK register

  // Set CS10 bit so timer runs at clock speed: (no prescaling)
  TCCR1B |= (1 << CS10); // Sets bit CS10 in TCCR1B
  TCCR1B |= (1 << CS12);
  // This is achieved by shifting binary 1 (0b00000001)
  // to the left by CS10 bits. This is then bitwise
  // OR-ed into the current value of TCCR1B, which effectively set
  // this one bit high. Similar: TCCR1B |= _BV(CS10);

  // enable global interrupts:
  sei();

}



